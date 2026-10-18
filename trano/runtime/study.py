"""The parameter inventory of a specification, with the run-time checks (Docker and OpenModelica).

:func:`run_inventory` compiles the specification once, classifies every numeric YAML field from the init
file, then runs on that executable the readback of the candidates (in batches of fields that share no
run-time parameter), the impact runs of the fields of the priorities asked for, and the equivalence
checks of a sample (one field per mechanism, high priority first, see :func:`equivalence_sample`). The
progress is kept in ``<project>/inventory.json``: an interrupted study resumes where it stopped.
"""

import json
import logging
import math
import re
from collections.abc import Callable, Iterable, Mapping, Sequence
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from trano.data_models.conversion import convert_network
from trano.elements.library.library import Library
from trano.runtime.adapters import get_adapter
from trano.runtime.checks import (
    STEP,
    WARM_UP,
    WINDOW,
    ImpactResult,
    OutputSeries,
    ParameterDependencies,
    build_changed,
    check_equivalence,
    check_impact,
    check_readback,
    parameter_filter,
    readback_run,
    window_run,
)
from trano.runtime.init_xml import ModelVariable, ParameterKind, read_model_variables
from trano.runtime.inventory import (
    InventoryRow,
    Priority,
    Status,
    free_overrides,
    perturbation,
    static_row,
    write_inventory,
)
from trano.runtime.model import CompiledModel, ModelRunner, RunError, build_model
from trano.runtime.resolution import Resolution, SpecificationResolver
from trano.runtime.specification import numeric_fields
from trano.simulate.simulate import CONTAINER_MODEL_DIR, SimulationOptions

logger = logging.getLogger(__name__)

PROGRESS_FILE = "inventory.json"
READBACK_BATCH = 25
CANDIDATES = (Status.changeable, Status.derived)
POWER_PREFIXES = ("Q_", "P_")


def window_options(tolerance: float = 1e-4) -> SimulationOptions:
    """The experiment of the inventory: warm-up and impact window from the 1st of January."""
    return SimulationOptions(
        start_time=0, end_time=WARM_UP + WINDOW, tolerance=tolerance, number_of_intervals=int((WARM_UP + WINDOW) / STEP)
    )


@dataclass
class FieldStudy:
    """What the study found for one field."""

    resolution: Resolution
    row: InventoryRow
    overrides: dict[str, float]
    signature: tuple[str, ...]

    @property
    def key(self) -> str:
        """The key of the field's checks in the progress file: its path and perturbed value."""
        return f"{self.row.yaml_path}={self.resolution.new_value}"


@dataclass
class Progress:
    path: Path
    data: dict[str, dict[str, Any]] = field(default_factory=dict)

    @classmethod
    def load(cls, path: Path) -> "Progress":
        return cls(path, json.loads(path.read_text()) if path.exists() else {})

    def get(self, check: str, yaml_path: str) -> dict[str, Any] | None:
        return self.data.get(check, {}).get(yaml_path)

    def set(self, check: str, yaml_path: str, value: dict[str, Any]) -> None:
        self.data.setdefault(check, {})[yaml_path] = value
        self.path.write_text(json.dumps(self.data, indent=1, sort_keys=True))


def _mechanism_key(variable: ModelVariable, mechanisms: Mapping[str, str]) -> str:
    return next(
        (pattern for pattern in mechanisms if re.search(pattern, variable.name)),
        re.sub(r"\[\d+(,\d+)?\]", "", variable.name.rsplit(".", 1)[-1]),
    )


def signature(
    resolution: Resolution, row: InventoryRow, variables: Mapping[str, ModelVariable], mechanisms: Mapping[str, str]
) -> tuple[str, ...]:
    """The mechanism of a field: its group, how it reaches Modelica and why some of its names are not free.

    Fields with the same signature reach the executable the same way; the equivalence of one tells the others'.
    """
    blocked = sorted(
        {
            _mechanism_key(variables[name], mechanisms)
            for name in resolution.runtime_names
            if name in variables and variables[name].kind != ParameterKind.free
        }
    )
    return (row.group, resolution.kind.value, *blocked)


def equivalence_sample(studies: Sequence[FieldStudy], extra: Iterable[str] = ()) -> list[str]:
    """One field per signature among the candidates, the high-priority ones first; plus the paths given."""
    order = {Priority.high: 0, Priority.medium: 1, Priority.low: 2}
    chosen: dict[tuple[str, ...], str] = {}
    for study in sorted(studies, key=lambda study: order[Priority(study.row.priority)]):
        if study.row.status in CANDIDATES and study.overrides:
            chosen.setdefault(study.signature, study.row.yaml_path)
    return list(dict.fromkeys([*chosen.values(), *extra]))


def admissible_perturbation(yaml_field: Any, bounds: tuple[float, float] | None) -> Any:  # noqa: ANN401
    """The perturbation of :func:`perturbation`, or one inside ``bounds`` when it falls outside.

    Inside the bounds the compiled structure (the discretisation of the layers) stays valid, so that the
    checks measure what a calibration can change; the bounds are reported in the inventory.
    """
    proposed = perturbation(yaml_field)
    if bounds is None or not isinstance(proposed, int | float):
        return proposed
    value, (low, high) = float(yaml_field.value), bounds
    if low < proposed <= high:
        return proposed
    for edge in (high, low):
        if math.isfinite(edge):
            candidate = value + 0.9 * (edge - value)
            if abs(candidate / value - 1) >= 0.01 and low < candidate <= high:
                return candidate
    return proposed


def _readback_batches(studies: Sequence[FieldStudy], dependencies: ParameterDependencies) -> list[list[FieldStudy]]:
    """Batches of fields whose run-time parameters and dependents do not overlap."""
    batches: list[tuple[list[FieldStudy], set[str]]] = []
    for study in studies:
        names = set(study.overrides) | dependencies.of(study.overrides)
        batch = next((batch for batch in batches if len(batch[0]) < READBACK_BATCH and not batch[1] & names), None)
        if batch is None:
            batches.append(([study], names))
        else:
            batch[0].append(study)
            batch[1].update(names)
    return [batch for batch, _ in batches]


@dataclass
class InventoryStudy:
    """The study of one specification for one library."""

    yaml_path: Path
    library: str
    project: Path
    workers: int = 3
    tolerance: float = 1e-4
    impact_priorities: tuple[Priority, ...] = (Priority.high, Priority.medium)
    select: str | None = None  # regular expression of the YAML paths studied; all numeric fields when None
    log: Callable[[str], None] = print

    def __post_init__(self) -> None:
        self.project = self.project.resolve()
        self.project.mkdir(parents=True, exist_ok=True)
        self.adapter = get_adapter(self.library)
        self.progress = Progress.load(self.project / PROGRESS_FILE)
        self.resolver = SpecificationResolver.from_file(self.yaml_path.resolve(), self.library)
        self.options = window_options(self.tolerance)

    # -- static part -------------------------------------------------------------------------------------
    def compile_baseline(self) -> CompiledModel:
        directory = self.project / "build"
        if (directory / "runtime.json").exists():
            return CompiledModel.load(directory)
        network = convert_network(
            self.resolver.name, self.yaml_path.resolve(), Library.from_configuration(self.library)
        )
        network.set_weather_path(self.project, CONTAINER_MODEL_DIR)
        model = network.model()
        outputs = self.adapter.impact_outputs(network)
        paths = [entry.modelica_path for entry in self.adapter.parameter_snapshot(network).values()]
        options = self.options.model_copy(update={"variable_filter": parameter_filter(paths, outputs.values())})
        self.log("compiling the baseline")
        return build_model(self.project, network, options, model=model)

    def outputs(self) -> dict[str, str]:
        network = convert_network(
            self.resolver.name, self.yaml_path.resolve(), Library.from_configuration(self.library)
        )
        network.model()
        return self.adapter.impact_outputs(network)

    def classify(self, compiled: CompiledModel) -> list[FieldStudy]:
        fields_ = [
            yaml_field
            for yaml_field in numeric_fields(self.resolver.description)
            if self.select is None or re.search(self.select, yaml_field.path)
        ]
        network = convert_network(
            self.resolver.name, self.yaml_path.resolve(), Library.from_configuration(self.library)
        )
        network.model()
        ranges = {
            yaml_field.path: bounds
            for yaml_field in fields_
            if (bounds := self.adapter.admissible_range(network, yaml_field.path)) is not None
        }
        self.log(f"resolving {len(fields_)} fields")
        resolutions = self.resolver.resolve_many(
            {
                yaml_field.path: admissible_perturbation(yaml_field, ranges.get(yaml_field.path))
                for yaml_field in fields_
            },
            workers=self.workers + 1,
        )
        names = {name for resolution in resolutions for name in resolution.runtime_names}
        variables = read_model_variables(compiled.init_xml, names)
        mechanisms = self.adapter.frozen_mechanisms
        studies = []
        for resolution in resolutions:
            row = static_row(resolution, variables, mechanisms)
            if (bounds := ranges.get(resolution.path)) is not None:
                note = f"admissible range for the compiled discretisation: ({bounds[0]:.4g}, {bounds[1]:.4g}]"
                row.reason = f"{row.reason} | {note}" if row.reason else note
            studies.append(
                FieldStudy(
                    resolution,
                    row,
                    free_overrides(resolution, variables) if row.status in CANDIDATES else {},
                    signature(resolution, row, variables, mechanisms),
                )
            )
        return studies

    # -- run-time checks ---------------------------------------------------------------------------------
    def readback(self, runner: ModelRunner, studies: Sequence[FieldStudy], dependencies: ParameterDependencies) -> None:
        pending = [study for study in studies if study.overrides and self.progress.get("readback", study.key) is None]
        if not pending:
            return
        dependents = {study.row.yaml_path: dependencies.of(study.overrides) for study in pending}
        baseline_names = set().union(*dependents.values())
        self.log(f"readback of {len(pending)} fields")
        baseline = readback_run(runner, {}, "readback_baseline", baseline_names).reader() if baseline_names else None
        baseline_values = {}
        for name in baseline_names:
            try:
                baseline_values[name] = float(baseline.values(name)[1][0])  # type: ignore[union-attr]
            except (KeyError, ValueError, IndexError):
                continue

        def run_batch(index_batch: tuple[int, list[FieldStudy]]) -> None:
            index, batch = index_batch
            overrides = {name: value for study in batch for name, value in study.overrides.items()}
            names = set(overrides) | {name for study in batch for name in dependents[study.row.yaml_path]}
            try:
                run = readback_run(runner, overrides, f"readback_{index:03d}", names)
            except RunError:
                if len(batch) > 1:  # find the field that fails the initialisation
                    for offset, study in enumerate(batch):
                        run_batch((1000 * (index + 1) + offset, [study]))
                    return
                self.progress.set("readback", batch[0].key, {"ok": False, "reason": "the run fails"})
                return
            for study in batch:
                result = check_readback(run, study.overrides, dependents[study.row.yaml_path], baseline_values)
                self.progress.set(
                    "readback",
                    study.key,
                    {
                        "ok": result.ok,
                        "reason": result.reason,
                        "unchanged_dependents": result.unchanged_dependents,
                    },
                )

        batches = _readback_batches(pending, dependencies)
        with ThreadPoolExecutor(self.workers) as pool:
            list(pool.map(run_batch, enumerate(batches)))

    def impact(self, runner: ModelRunner, studies: Sequence[FieldStudy], outputs: Mapping[str, str]) -> OutputSeries:
        baseline_path = self.project / "runs" / "window_baseline_result.mat"
        if not baseline_path.exists():
            self.log("baseline window run")
            window_run(runner, {}, "window_baseline", outputs)
        baseline = OutputSeries.read(baseline_path, outputs)
        powers = [label for label in outputs if label.startswith(POWER_PREFIXES)]
        pending = [
            study
            for study in studies
            if study.overrides
            and (self.progress.get("readback", study.key) or {}).get("ok")
            and Priority(study.row.priority) in self.impact_priorities
            and self.progress.get("impact", study.key) is None
        ]
        self.log(f"impact runs of {len(pending)} fields")

        def run_one(index_study: tuple[int, FieldStudy]) -> None:
            _, study = index_study
            label = f"window_{_slug(study.row.yaml_path)}"
            try:
                run = window_run(runner, study.overrides, label, outputs)
            except RunError as error:
                self.progress.set("impact", study.key, {"error": str(error)[-500:]})
                return
            result = check_impact(OutputSeries.read(run.result_path, outputs), baseline, powers)
            self.progress.set(
                "impact",
                study.key,
                {
                    "metrics": result.metrics,
                    "energy": result.energy,
                    "summary": result.summary(),
                    "has_impact": result.has_impact,
                    "result": str(run.result_path),
                },
            )

        with ThreadPoolExecutor(self.workers) as pool:
            list(pool.map(run_one, enumerate(pending)))
        return baseline

    def equivalence(
        self,
        compiled: CompiledModel,
        studies: Sequence[FieldStudy],
        sample: Sequence[str],
        outputs: Mapping[str, str],
        baseline: OutputSeries,
    ) -> None:
        powers = [label for label in outputs if label.startswith(POWER_PREFIXES)]
        by_path = {study.row.yaml_path: study for study in studies}
        pending = [path for path in sample if self.progress.get("equivalence", by_path[path].key) is None]
        self.log(f"equivalence of {len(pending)} fields")

        def run_one(index_path: tuple[int, str]) -> None:
            index, path = index_path
            study = by_path[path]
            slug = _slug(path)
            project = self.project / "equivalence" / slug
            try:
                recorded = self.progress.get("impact", study.key) or {}
                override_result = Path(recorded["result"]) if "result" in recorded else None
                if override_result is None or not override_result.exists():
                    with ModelRunner(compiled, container_name=f"trano_eq_override_{index}") as runner:
                        override_result = window_run(runner, study.overrides, f"window_{slug}", outputs).result_path
                    impact = check_impact(OutputSeries.read(override_result, outputs), baseline, powers)
                    self.progress.set("impact", study.key, _impact_record(impact, override_result))
                recompiled = build_changed(
                    self.resolver.description,
                    {path: study.resolution.new_value},
                    project,
                    self.resolver.name,
                    self.library,
                    self.options,
                    container_name=f"trano_eq_build_{index}",
                )
                with ModelRunner(recompiled, container_name=f"trano_eq_run_{index}") as runner:
                    recompiled_result = window_run(runner, {}, "window", outputs).result_path
                equivalence = check_equivalence(
                    OutputSeries.read(override_result, outputs),
                    OutputSeries.read(recompiled_result, outputs),
                    compiled,
                    recompiled,
                    study.overrides,
                )
                prune_build(recompiled)
                self.progress.set(
                    "equivalence",
                    study.key,
                    {
                        "ok": equivalence.ok,
                        "largest": equivalence.largest,
                        "label": equivalence.label,
                        "missed": equivalence.missed,
                        "evaluated": equivalence.evaluated,
                        "summary": equivalence.summary(),
                    },
                )
            except Exception as error:  # a failed build or run is a result of the check
                logger.exception("equivalence of %s", path)
                self.progress.set(
                    "equivalence", study.key, {"ok": False, "error": f"{type(error).__name__}: {str(error)[-500:]}"}
                )

        with ThreadPoolExecutor(max(1, self.workers - 1)) as pool:
            list(pool.map(run_one, enumerate(pending)))

    # -- result ------------------------------------------------------------------------------------------
    def finalise(self, studies: Sequence[FieldStudy]) -> list[InventoryRow]:
        verified = {
            study.signature: (study.row.yaml_path, self.progress.get("equivalence", study.key))
            for study in studies
            if self.progress.get("equivalence", study.key) is not None
        }
        for study in studies:
            _finalise_row(study, self.progress, verified)
        return [study.row for study in studies]

    def run(self, extra_equivalence: Iterable[str] = (), checks: bool = True) -> list[InventoryRow]:
        compiled = self.compile_baseline()
        studies = self.classify(compiled)
        if checks:
            outputs = self.outputs()
            dependencies = ParameterDependencies(compiled.info_json)
            with ModelRunner(compiled) as runner:
                self.readback(runner, studies, dependencies)
                baseline = self.impact(runner, studies, outputs)
            self.equivalence(compiled, studies, equivalence_sample(studies, extra_equivalence), outputs, baseline)
        rows = self.finalise(studies)
        write_inventory(rows, self.project / f"parameter_inventory_{self.library.lower()}.csv")
        return rows


KEPT_BUILD_FILES = ("build.log", "build.mos", "runtime.json")


def prune_build(compiled: CompiledModel) -> None:
    """Remove what a build directory holds besides its logs, script and model (hundreds of MB per build)."""
    for path in compiled.directory.iterdir():
        if path.is_file() and path.name not in KEPT_BUILD_FILES and path.suffix != ".mo":
            path.unlink()


def _impact_record(result: ImpactResult, path: Path) -> dict[str, Any]:
    return {
        "metrics": result.metrics,
        "energy": result.energy,
        "summary": result.summary(),
        "has_impact": result.has_impact,
        "result": str(path),
    }


def _slug(path: str) -> str:
    return re.sub(r"[^A-Za-z0-9]+", "_", path).strip("_").lower()[:120]


def _finalise_row(  # noqa: C901, PLR0912
    study: FieldStudy, progress: Progress, verified: Mapping[tuple[str, ...], tuple[str, dict[str, Any] | None]]
) -> None:
    row = study.row
    readback = progress.get("readback", study.key)
    impact = progress.get("impact", study.key)
    equivalence = progress.get("equivalence", study.key)
    reasons = [row.reason] if row.reason else []
    if equivalence is not None:
        row.equivalence_ok = str(bool(equivalence.get("ok"))).lower()
        reasons.append(f"equivalence: {equivalence.get('summary') or equivalence.get('error')}")
    if row.status not in CANDIDATES:
        row.readback_ok = row.readback_ok or "n/a"
        row.reason = " | ".join(reasons)
        return
    if readback is None:
        row.readback_ok = "not run"
    else:
        row.readback_ok = str(bool(readback["ok"])).lower()
        if readback.get("unchanged_dependents"):
            reasons.append(f"dependents unchanged: {', '.join(readback['unchanged_dependents'][:3])}")
        if not readback["ok"]:
            row.status = Status.frozen
            reasons.append(f"readback: {readback.get('reason')}")
    if impact is not None:
        row.impact_metric = impact.get("summary") or f"run failed: {impact.get('error', '')[:120]}"
    elif row.status in CANDIDATES:
        row.impact_metric = "not run (low priority)" if Priority(row.priority) == Priority.low else "not run"
    if equivalence is None and row.status in CANDIDATES:
        source = verified.get(study.signature)
        if source is not None and source[1] is not None:
            ok = bool(source[1].get("ok"))
            row.equivalence_ok = f"inferred_{str(ok).lower()}"
            reasons.append(f"equivalence inferred from {source[0]} (same mechanism)")
        else:
            row.equivalence_ok = "not run"
    if row.status in CANDIDATES:
        has_impact = impact is not None and impact.get("has_impact")
        equivalent = row.equivalence_ok in ("true", "inferred_true")
        if row.equivalence_ok in ("false", "inferred_false"):
            row.status = Status.frozen
            if impact is not None and not has_impact:
                reasons.append("the override is taken without effect while the recompiled model changes")
        elif impact is not None and not has_impact:
            row.status = Status.no_impact
            if row.equivalence_ok == "true":
                reasons.append("inert in this scenario: the recompiled model does not change either")
            elif equivalent:
                reasons.append("below the impact threshold; the override is taken (equivalence inferred)")
            else:
                reasons.append("below the impact threshold")
    row.reason = " | ".join(reasons)


__all__ = ["FieldStudy", "InventoryStudy", "equivalence_sample", "prune_build", "signature", "window_options"]
