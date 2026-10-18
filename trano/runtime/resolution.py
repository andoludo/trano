"""From YAML fields to run-time parameters: which ones a field reaches and the values they take.

A field is resolved by difference: the specification is converted twice, as given and with the field
changed, and the parameter snapshots of the library adapter are compared. The run-time parameters
whose values differ are the field's fan-out (a material reaches every construction and zone that uses
it), their new values are what an override must set, Python transformations included (``n50`` gives the
infiltration ``ACH``, a radiator's power its mass and water volume). Comparing the Modelica text too tells
apart a field the library does not use (the model does not change) from a gap of the adapter (the model
changes, the snapshot does not).

:class:`SpecificationResolver` keeps the baseline and turns YAML-level changes into overrides of a
compiled model, the calibration entry point:

    resolver = SpecificationResolver.from_file(Path("house.yaml"), "Buildings")
    overrides = resolver.overrides({"material[EPS:001].thermal_conductivity": 0.035})
"""

import enum
import math
import re
import tempfile
from collections.abc import Mapping
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from trano.data_models.conversion import convert_network
from trano.elements.library.library import Library
from trano.exceptions import InvalidBuildingStructureError
from trano.runtime.adapters import Snapshot, get_adapter
from trano.runtime.expressions import NotNumericError, evaluate
from trano.runtime.specification import FieldValue, get_value, load_description, with_values, write_description

_ANNOTATION = re.compile(r"annotation\s*\((?:[^()]|\((?:[^()]|\((?:[^()]|\([^()]*\))*\))*\))*\)")
RELATIVE_TOLERANCE = 1e-12


class ResolutionKind(enum.StrEnum):
    direct = "direct"  # the run-time parameters take the YAML value itself
    derived = "derived"  # trano transforms the value before it reaches Modelica
    not_used = "not_used"  # the model of the library does not change with the field
    unresolved = "unresolved"  # the model changes but the adapter does not see which parameter
    structural = "structural"  # trano rejects the change or parameters appear or disappear


@dataclass(frozen=True)
class ParameterChange:
    name: str
    old: float
    new: float
    modelica_path: str
    formula: str | None = None


@dataclass(frozen=True)
class Resolution:
    """What a YAML field reaches in the compiled model."""

    path: str
    value: FieldValue
    new_value: FieldValue
    changes: tuple[ParameterChange, ...]
    model_changed: bool
    not_used_reason: str | None = None
    error: str | None = None  # why trano rejected the changed specification

    @property
    def structural_names(self) -> tuple[str, ...]:
        """Run-time parameters that appear or disappear with the change."""
        return tuple(change.name for change in self.changes if math.isnan(change.old) or math.isnan(change.new))

    @property
    def kind(self) -> ResolutionKind:
        if self.error is not None or self.structural_names:
            return ResolutionKind.structural
        if not self.changes:
            return ResolutionKind.unresolved if self.model_changed else ResolutionKind.not_used
        targets = _numbers(self.new_value)
        if all(any(_close(change.new, target) for target in targets) for change in self.changes):
            return ResolutionKind.direct
        return ResolutionKind.derived

    @property
    def runtime_names(self) -> tuple[str, ...]:
        return tuple(change.name for change in self.changes)

    @property
    def modelica_paths(self) -> tuple[str, ...]:
        return tuple(dict.fromkeys(change.modelica_path for change in self.changes))

    @property
    def overrides(self) -> dict[str, float]:
        return {change.name: change.new for change in self.changes}

    @property
    def transformation(self) -> str | None:
        """How the run-time values follow from the field, for a derived field."""
        if self.kind != ResolutionKind.derived:
            return None
        formulas = sorted({change.formula for change in self.changes if change.formula})
        ratios = sorted(
            {f"{change.name.rsplit('.', 1)[-1]}: {change.old:g} -> {change.new:g}" for change in self.changes}
        )
        described = "; ".join(formulas) if formulas else "computed in trano"
        return f"{described} ({', '.join(ratios[:4])}{', ...' if len(ratios) > 4 else ''})"


def _numbers(value: FieldValue) -> list[float]:
    if isinstance(value, int | float):
        return [float(value)]
    if isinstance(value, list):
        return [float(item) for item in value]
    try:
        evaluated = evaluate(value)
    except NotNumericError:
        return []
    return evaluated if isinstance(evaluated, list) else [evaluated]  # type: ignore[return-value]


def _close(a: float, b: float) -> bool:
    return math.isclose(a, b, rel_tol=1e-9, abs_tol=1e-12)


def model_text(model: str) -> str:
    """The Modelica text without annotations: the layout of a conversion is not deterministic."""
    return _ANNOTATION.sub("", model)


@dataclass
class ConvertedSpecification:
    """The snapshot and the model text of a specification for a library."""

    snapshot: Snapshot
    model: str
    model_name: str


def convert(description: Mapping[str, Any], name: str, library: str) -> ConvertedSpecification:
    """Convert a description (written to a temporary YAML file, as trano reads files) for the library."""
    adapter = get_adapter(library)
    with tempfile.TemporaryDirectory() as directory:
        path = write_description(dict(description), Path(directory) / f"{name}.yaml")
        network = convert_network(name, path, library=Library.from_configuration(library))
        model = network.model()
    return ConvertedSpecification(adapter.parameter_snapshot(network), model_text(model), adapter.model_name(network))


def diff(baseline: Snapshot, changed: Snapshot) -> tuple[ParameterChange, ...]:
    """The parameters whose values differ (or that appear or disappear: a structural change)."""
    changes = []
    for name in sorted(baseline.keys() | changed.keys()):
        old, new = baseline.get(name), changed.get(name)
        if old is not None and new is not None and math.isclose(old.value, new.value, rel_tol=RELATIVE_TOLERANCE):
            continue
        entry = new or old
        assert entry is not None
        changes.append(
            ParameterChange(
                name=name,
                old=old.value if old is not None else math.nan,
                new=new.value if new is not None else math.nan,
                modelica_path=entry.modelica_path,
                formula=entry.formula,
            )
        )
    return tuple(changes)


@dataclass
class SpecificationResolver:
    """Resolves YAML changes against a baseline specification for one library."""

    description: dict[str, Any]
    name: str
    library: str
    baseline: ConvertedSpecification = field(init=False)

    def __post_init__(self) -> None:
        self.baseline = convert(self.description, self.name, self.library)

    @classmethod
    def from_file(cls, path: Path, library: str) -> "SpecificationResolver":
        return cls(load_description(path), path.stem, library)

    def not_used_reason(self, path: str) -> str | None:
        adapter = get_adapter(self.library)
        return next((reason for pattern, reason in adapter.not_used_fields.items() if re.search(pattern, path)), None)

    def resolve(self, path: str, new_value: FieldValue) -> Resolution:
        """What changing the field at ``path`` to ``new_value`` changes in the compiled model."""
        try:
            changed = convert(with_values(self.description, {path: new_value}), self.name, self.library)
        except (InvalidBuildingStructureError, ValueError) as error:
            return Resolution(
                path=path,
                value=get_value(self.description, path),
                new_value=new_value,
                changes=(),
                model_changed=True,
                not_used_reason=self.not_used_reason(path),
                error=f"{type(error).__name__}: {error}",
            )
        return Resolution(
            path=path,
            value=get_value(self.description, path),
            new_value=new_value,
            changes=diff(self.baseline.snapshot, changed.snapshot),
            model_changed=changed.model != self.baseline.model,
            not_used_reason=self.not_used_reason(path),
        )

    def resolve_many(self, changes: Mapping[str, FieldValue], workers: int = 1) -> list[Resolution]:
        """Resolve fields one at a time, in parallel processes when ``workers`` > 1."""
        if workers <= 1:
            return [self.resolve(path, value) for path, value in changes.items()]
        with ProcessPoolExecutor(max_workers=workers) as pool:
            futures = [pool.submit(self.resolve, path, value) for path, value in changes.items()]
            return [future.result() for future in futures]

    def overrides(self, changes: Mapping[str, FieldValue]) -> dict[str, float]:
        """The run-time overrides equivalent to the YAML changes, applied together.

        Raises ValueError when a change is structural (parameters appear or disappear) or not seen by
        the adapter: no override can stand for it.
        """
        changed = convert(with_values(self.description, dict(changes)), self.name, self.library)
        differences = diff(self.baseline.snapshot, changed.snapshot)
        structural = [change.name for change in differences if math.isnan(change.old) or math.isnan(change.new)]
        if structural:
            raise ValueError(f"The changes alter the structure of the model: {structural[:5]}")
        if not differences and changed.model != self.baseline.model:
            raise ValueError(f"The changes {sorted(changes)} alter the model in a way no run-time parameter covers")
        return {change.name: change.new for change in differences}


__all__ = [
    "ConvertedSpecification",
    "ParameterChange",
    "Resolution",
    "ResolutionKind",
    "SpecificationResolver",
    "convert",
    "diff",
    "model_text",
]
