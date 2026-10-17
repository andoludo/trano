"""Build the calibration-parameter inventory of a YAML building for one library.

For every numeric field of the YAML, :func:`inventory` resolves the FMU parameters it sets
(:mod:`trano.fmu.inventory`, the library's envelope resolver), classifies them from the
``modelDescription.xml`` of the baseline FMU and, when asked, runs the readback, impact and
equivalence checks of :mod:`trano.fmu.checks`. Everything OpenModelica-related goes through
:func:`trano.fmu.export.export_fmu`; everything FMU-related through FMPy.

The perturbation of a field is done on the YAML itself: the building is regenerated from the
perturbed description and resolved again, which gives the perturbed value of every FMU parameter
the field reaches, whatever Python does to it in between (a square root for a window area, a
division for the airtightness, a schedule expanded into an array). The same perturbed description
is what the equivalence check rebuilds the FMU from.
"""

import copy
import logging
import re
import tempfile
from collections.abc import Callable, Iterable, Sequence
from pathlib import Path
from typing import Any

import pandas as pd
import yaml
from pydantic import BaseModel, ConfigDict, Field

from trano.data_models.conversion import convert_network
from trano.elements.library.library import Library
from trano.exceptions import FmuError
from trano.fmu.buildings import BuildingsEnvelopeResolver
from trano.fmu.checks import (
    EQUIVALENCE_TOLERANCE,
    READBACK_TOLERANCE,
    Outputs,
    Window,
    compare,
    impact_outputs,
    max_relative_rms,
    perturb,
    read_back,
    simulate_fmu,
)
from trano.fmu.export import FmuExportOptions, export_fmu
from trano.fmu.inventory import (
    EnvelopeResolver,
    InventoryRow,
    ResolvedParameter,
    Status,
    classify,
    parse_modelica_array,
    resolve_all,
)
from trano.fmu.model_description import ModelDescription, read_model_description
from trano.topology import Network

logger = logging.getLogger(__name__)

ENVELOPE_RESOLVERS: dict[str, Callable[[], EnvelopeResolver]] = {"Buildings": BuildingsEnvelopeResolver}
"""The envelope resolver of each library; IDEAS is to be added here."""
FRACTION_FIELDS = ("fraction", "efficiency", "effectiveness", "transmittance", "reflectance", "absorptivity")
"""Fields bounded by 1: perturbed downwards."""
PERTURBATION = 1.2
_SEGMENT = re.compile(r"^(?P<key>[^\[\]]+)(?:\[(?P<index>[^\]]+)\])?$")


def envelope_resolver(library: Library) -> EnvelopeResolver:
    try:
        return ENVELOPE_RESOLVERS[library.base_library()]()
    except KeyError as error:
        raise FmuError(f"No envelope resolver for the {library.name} library yet; Buildings only") from error


def load_description(yaml_path: Path) -> dict[str, Any]:
    data = yaml.safe_load(yaml_path.read_text())
    if not isinstance(data, dict):
        raise FmuError(f"{yaml_path} does not hold a building description")
    return data


def network_from_description(data: dict[str, Any], name: str, library: Library, directory: Path) -> Network:
    """The network of a (possibly perturbed) description, through the YAML conversion and validation."""
    path = directory / f"{name}.yaml"
    path.write_text(yaml.safe_dump(data, sort_keys=False))
    network = convert_network(name, path, library=library)
    network.model()  # renders the zones' boundary arrays the envelope resolver indexes
    return network


def get_at(data: Any, yaml_path: str) -> Any:  # noqa: ANN401
    """The value at a path of the inventory (``spaces[LIVING:001].emissions[0].radiator.parameters.x``)."""
    node = data
    for segment in yaml_path.split("."):
        node = _step(node, segment)
    return node


def set_at(data: dict[str, Any], yaml_path: str, value: Any) -> dict[str, Any]:  # noqa: ANN401
    """A deep copy of the description with the value at the path replaced."""
    perturbed = copy.deepcopy(data)
    *parents, last = yaml_path.split(".")
    node: Any = perturbed
    for segment in parents:
        node = _step(node, segment)
    match = _SEGMENT.match(last)
    if match is None:
        raise FmuError(f"Cannot parse {last!r} in {yaml_path}")
    key, index = match.group("key"), match.group("index")
    if index is None:
        node[key] = value
    else:
        items = node[key]
        items[_position(items, index)] = value
    return perturbed


def _step(node: Any, segment: str) -> Any:  # noqa: ANN401
    match = _SEGMENT.match(segment)
    if match is None:
        raise FmuError(f"Cannot parse {segment!r}")
    key, index = match.group("key"), match.group("index")
    node = node[key]
    return node if index is None else node[_position(node, index)]


def _position(items: Sequence[Any], index: str) -> int:
    """The position of an item of a list: an integer index, or the ``id`` of the item."""
    if index.isdigit():
        return int(index)
    for position, item in enumerate(items):
        if isinstance(item, dict) and item.get("id") == index:
            return position
    raise FmuError(f"No item with id {index!r}")


def perturbed_value(yaml_path: str, value: Any, factor: float = PERTURBATION) -> Any:  # noqa: ANN401
    """The perturbed YAML value: scaled by the factor, downwards for a fraction, every element of a list or array."""
    field = yaml_path.rsplit(".", 1)[-1]
    downwards = any(word in field for word in FRACTION_FIELDS)
    if isinstance(value, list):
        return [perturbed_value(yaml_path, item, factor) for item in value]
    if isinstance(value, str):
        array = parse_modelica_array(value)
        if array is None:
            return value
        # schedules: earlier switching times keep the sequence inside the day
        scaled = [item / factor for item in array]
        return "{" + ", ".join(f"{item:g}" for item in scaled) + "}"
    if isinstance(value, bool):
        return value
    if downwards:
        return perturb(float(value), 1 / factor, lower=0.0)
    return perturb(float(value), factor)


class CheckResults(BaseModel):
    """What the simulation checks found for one parameter."""

    model_config = ConfigDict(frozen=True)
    readback_ok: bool | None = None
    readback_detail: str = ""
    impact: str = ""
    impact_significant: bool | None = None
    equivalence_ok: bool | None = None
    equivalence_detail: str = ""


class InventoryCase(BaseModel):
    """One YAML field with its resolution on the baseline and on the perturbed description."""

    model_config = ConfigDict(arbitrary_types_allowed=True)
    resolved: ResolvedParameter
    row: InventoryRow
    perturbed_data: dict[str, Any] | None = None
    start_values: dict[str, float] = Field(default_factory=dict, description="FMU override reproducing the change")

    @property
    def candidate(self) -> bool:
        return self.row.status in (Status.calibratable.value, Status.derived.value)


class Inventory(BaseModel):
    """The inventory of a YAML building for one library, with the baseline FMU it was made against."""

    model_config = ConfigDict(arbitrary_types_allowed=True)
    library: str
    yaml_path: Path
    fmu: Path
    outputs: Outputs
    cases: list[InventoryCase]

    def rows(self) -> list[InventoryRow]:
        return [case.row for case in self.cases]


class InventorySettings(BaseModel):
    """What an inventory is built from and against."""

    model_config = ConfigDict(frozen=True)
    yaml_path: Path
    library_name: str = "Buildings"
    fmu: Path = Field(description="Baseline FMU exported from the unchanged description")
    work_directory: Path
    window: Window = Field(default_factory=Window)
    export_options: FmuExportOptions | None = Field(default=None, description="Options of the equivalence rebuilds")

    @property
    def library(self) -> Library:
        return Library.from_configuration(self.library_name)


class InventoryBuilder:
    """Resolves, classifies and checks the parameters of one YAML building against its baseline FMU."""

    def __init__(self, settings: InventorySettings) -> None:
        self.settings = settings
        self.yaml_path = settings.yaml_path
        self.library = settings.library
        self.fmu = settings.fmu
        self.work_directory = settings.work_directory
        self.window = settings.window
        self.export_options = settings.export_options or FmuExportOptions(library_name=settings.library_name)
        self.data = load_description(self.yaml_path)
        self.network = network_from_description(self.data, self.yaml_path.stem, self.library, self.work_directory)
        self.description: ModelDescription = read_model_description(self.fmu)
        self.resolver = envelope_resolver(self.library)
        self.outputs = impact_outputs(self.network)
        self._baseline: pd.DataFrame | None = None

    def resolve(self) -> list[InventoryCase]:
        """Every numeric field classified from the model description, with its perturbed override."""
        cases = []
        for resolved in resolve_all(self.network, self.data, self.resolver):
            row = classify(resolved, self.description)
            case = InventoryCase(resolved=resolved, row=row)
            if case.candidate:
                case = self._with_perturbation(case)
            cases.append(case)
        return cases

    def _with_perturbation(self, case: InventoryCase) -> InventoryCase:
        yaml_path = case.resolved.yaml_path
        value = get_at(self.data, yaml_path)
        perturbed = perturbed_value(yaml_path, value)
        if perturbed == value:
            return case.model_copy(update={"row": _with_reason(case.row, "value cannot be perturbed")})
        data = set_at(self.data, yaml_path, perturbed)
        with tempfile.TemporaryDirectory(dir=self.work_directory) as directory:
            network = network_from_description(data, self.yaml_path.stem, self.library, Path(directory))
        resolved = next((r for r in resolve_all(network, data, self.resolver) if r.yaml_path == yaml_path), None)
        if resolved is None or [t.path for t in resolved.targets] != case.resolved.paths:
            return case.model_copy(
                update={"row": _with_reason(case.row, "the perturbed description resolves to other parameters")}
            )
        start_values = {t.path: float(t.value) for t in resolved.targets if t.value is not None}
        return case.model_copy(update={"perturbed_data": data, "start_values": start_values})

    def baseline(self) -> pd.DataFrame:
        if self._baseline is None:
            self._baseline = simulate_fmu(self.fmu, self.outputs.names, self.window)
        return self._baseline

    def readback(self, case: InventoryCase) -> CheckResults:
        """Set the override before initialisation and read it back after."""
        if not case.start_values:
            return CheckResults(readback_ok=None, readback_detail="nothing to set")
        try:
            values = read_back(self.fmu, case.start_values, case.start_values)
        except Exception as error:
            return CheckResults(readback_ok=False, readback_detail=f"readback failed: {error}")
        wrong = {
            name: (expected, values[name])
            for name, expected in case.start_values.items()
            if abs(values[name] - expected) > READBACK_TOLERANCE * max(1.0, abs(expected))
        }
        if wrong:
            detail = "; ".join(f"{name}: set {expected:g}, read {read:g}" for name, (expected, read) in wrong.items())
            return CheckResults(readback_ok=False, readback_detail=detail)
        return CheckResults(readback_ok=True)

    def impact(self, case: InventoryCase) -> tuple[CheckResults, pd.DataFrame | None]:
        """Simulate the override and compare it to the baseline on the impact outputs."""
        if not case.start_values:
            return CheckResults(), None
        try:
            frame = simulate_fmu(self.fmu, self.outputs.names, self.window, case.start_values)
        except Exception as error:
            return CheckResults(impact=f"simulation failed: {error}", impact_significant=False), None
        impact = compare(self.baseline(), frame, self.outputs)
        return CheckResults(impact=impact.summary(), impact_significant=impact.significant), frame

    def equivalence(self, case: InventoryCase, override_run: pd.DataFrame, directory: Path) -> CheckResults:
        """Rebuild the FMU from the perturbed description and compare its run to the override run."""
        if case.perturbed_data is None:
            return CheckResults()
        directory.mkdir(parents=True, exist_ok=True)
        network = network_from_description(case.perturbed_data, self.yaml_path.stem, self.library, directory)
        try:
            fmu = export_fmu(directory, network, options=self.export_options, container_name=f"fmu_{directory.name}")
            rebuilt = simulate_fmu(fmu, self.outputs.names, self.window)
        except Exception as error:
            return CheckResults(equivalence_ok=False, equivalence_detail=f"rebuild failed: {error}")
        deviation = max_relative_rms(override_run, rebuilt)
        return CheckResults(
            equivalence_ok=deviation <= EQUIVALENCE_TOLERANCE,
            equivalence_detail=f"max rel_rms={deviation:.3g} (tolerance {EQUIVALENCE_TOLERANCE:g})",
        )


def _with_reason(row: InventoryRow, reason: str) -> InventoryRow:
    reasons = [part for part in (row.reason, reason) if part]
    return row.model_copy(update={"reason": "; ".join(reasons)})


def apply_results(row: InventoryRow, results: Iterable[CheckResults]) -> InventoryRow:
    """The row updated with the outcome of the checks; the status follows the first failing check."""
    merged = CheckResults()
    for result in results:
        merged = merged.model_copy(update={k: v for k, v in result.model_dump().items() if v not in (None, "")})
    update: dict[str, Any] = {
        "readback_ok": _yes_no(merged.readback_ok),
        "impact_metric": merged.impact,
        "equivalence_ok": _yes_no(merged.equivalence_ok),
    }
    reasons = [row.reason] if row.reason else []
    status = row.status
    if merged.readback_ok is False:
        status, reasons = Status.frozen.value, [*reasons, f"readback failed: {merged.readback_detail}"]
    elif merged.impact_significant is False:
        status = Status.no_impact.value
        reasons.append("no impact on the outputs over the window")
    elif merged.equivalence_ok is False:
        status = Status.frozen.value if row.status == Status.calibratable.value else row.status
        reasons.append(f"equivalence failed: {merged.equivalence_detail}")
    elif merged.equivalence_ok and merged.equivalence_detail:
        reasons.append(f"equivalence {merged.equivalence_detail}")
    update |= {"status": status, "reason": "; ".join(reasons)}
    return row.model_copy(update=update)


def _yes_no(value: bool | None) -> str:
    return "" if value is None else ("yes" if value else "no")
