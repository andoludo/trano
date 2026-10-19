"""The run-time checks that settle whether a YAML field can be changed in a compiled model.

All checks use the same compiled executable:

1. **Readback**: run with the perturbed values and read the parameters back from the result file. Each
   equals the value set, the parameters bound to them (from the parameter equations of ``_info.json``)
   change, and the log reports none of them as not applied.
2. **Impact**: compare the outputs of the perturbed run to the baseline over the impact window. The metric
   of an output is its normalised root-mean-square difference (NRMSD: RMS difference divided by the range
   of the baseline output), on a common time grid after the warm-up. A field moves the outputs when the
   largest NRMSD reaches :data:`IMPACT_THRESHOLD`.
3. **Equivalence**: change the field in the YAML, regenerate the model, recompile and simulate without
   overrides; the outputs match the override run on the baseline executable within
   :data:`EQUIVALENCE_TOLERANCE`. The start values of the two init files tell which run-time parameters
   the recompiled model changes that the override did not set.
"""

import json
import logging
import math
import re
from collections import defaultdict
from collections.abc import Collection, Iterable, Mapping
from dataclasses import dataclass, field
from functools import cached_property
from pathlib import Path

import numpy as np
from buildingspy.io.outputfile import Reader  # type: ignore

from trano.data_models.conversion import convert_network
from trano.elements.library.library import Library
from trano.runtime.init_xml import ParameterKind, read_model_variables
from trano.runtime.model import CompiledModel, ModelRunner, RunResult, build_model
from trano.runtime.overrides import OverrideValue
from trano.runtime.specification import FieldValue, with_values, write_description
from trano_simulate.simulate import SimulationOptions

logger = logging.getLogger(__name__)

DAY = 24 * 3600
WARM_UP = 2 * DAY  # [s] simulated before the impact window
WINDOW = 14 * DAY  # [s] impact window: two January weeks with heating, setback and heat recovery
STEP = 900.0  # [s] output interval and comparison grid
READBACK_STOP_TIME = 3600.0  # [s] a readback run only needs the initialisation
READBACK_FLAGS = ("-ignoreHideResult", "-emit_protected")
"""Store every variable asked for: the copies of the records in the zones are marked HideResult."""
IMPACT_THRESHOLD = 1e-3
"""Largest NRMSD below which a perturbation does not move the outputs (0.1 % of the output's range)."""
EQUIVALENCE_TOLERANCE = 1e-6
"""Largest NRMSD between an override run and the recompiled model for them to be equivalent."""
RELATIVE_READBACK_TOLERANCE = 1e-9
MAX_FILTER_LENGTH = 50_000
"""Longest variable filter given on the command line; longer lists store every variable instead."""


def variable_filter(names: Iterable[str], patterns: Iterable[str] = ()) -> str:
    """A variable filter (POSIX extended regular expression) keeping the given names and patterns."""
    escaped = [re.escape(name) for name in dict.fromkeys(names)]
    return "^(" + "|".join([*escaped, *patterns]) + ")$"


def parameter_filter(modelica_paths: Iterable[str], outputs: Iterable[str]) -> str:
    """Build-time filter: the outputs and every scalar of the parameters set from the specification."""
    patterns = [rf"{re.escape(path)}(\[[0-9,]+\])?(\..*)?" for path in dict.fromkeys(modelica_paths)]
    return variable_filter(outputs, patterns)


@dataclass
class ParameterDependencies:
    """The parameter equations of an executable (``_info.json``): which parameters each one defines."""

    info_json: Path

    @cached_property
    def dependents(self) -> dict[str, set[str]]:
        dependents: dict[str, set[str]] = defaultdict(set)
        for equation in json.loads(self.info_json.read_text())["equations"]:
            if equation.get("section") != "parameter":
                continue
            for used in equation.get("uses", []):
                dependents[used].update(equation.get("defines", []))
        return dependents

    def of(self, names: Collection[str]) -> set[str]:
        """The parameters bound to the given ones (directly), the given ones excluded."""
        return {dependent for name in names for dependent in self.dependents.get(name, ())} - set(names)


def _parameter_value(reader: Reader, name: str) -> float | None:
    try:
        return float(reader.values(name)[1][0])
    except (KeyError, ValueError, IndexError):
        return None


@dataclass
class ReadbackResult:
    ok: bool
    reason: str = ""
    unchanged_dependents: list[str] = field(default_factory=list)


def check_readback(
    run: RunResult,
    overrides: Mapping[str, float],
    dependents: Collection[str],
    baseline: Mapping[str, float],
) -> ReadbackResult:
    """Readback of the overrides of a run (see the module documentation); the run may set other fields too."""
    ignored = [message for message in run.messages if message.name in overrides]
    if ignored:
        listed = ", ".join(f"{message.name} ({message.issue})" for message in ignored)
        return ReadbackResult(False, f"the executable ignored overrides: {listed}")
    reader = run.reader()
    wrong = []
    for name, value in overrides.items():
        read = _parameter_value(reader, name)
        if read is None or not math.isclose(read, value, rel_tol=RELATIVE_READBACK_TOLERANCE, abs_tol=1e-14):
            wrong.append(f"{name}={read} (set {value})")
    if wrong:
        return ReadbackResult(False, f"read back differs: {'; '.join(wrong[:5])}")
    unchanged = []
    for dependent in sorted(dependents):
        read, before = _parameter_value(reader, dependent), baseline.get(dependent)
        if read is not None and before is not None and math.isclose(read, before, rel_tol=1e-12, abs_tol=1e-14):
            unchanged.append(dependent)
    return ReadbackResult(True, "", unchanged)


@dataclass
class OutputSeries:
    """The outputs of a run on the comparison grid of the impact window."""

    values: dict[str, np.ndarray]

    @classmethod
    def read(
        cls, result: Path, outputs: Mapping[str, str], start: float = WARM_UP, step: float = STEP
    ) -> "OutputSeries":
        reader = Reader(str(result), "dymola")
        series = {}
        for label, name in outputs.items():
            time, value = reader.values(name)
            grid = np.arange(start, time[-1] + step / 2, step)
            series[label] = np.interp(grid, time, value)
        return cls(series)

    def nrmsd(self, other: "OutputSeries") -> dict[str, float]:
        """Normalised RMS difference of each output to ``other`` (the reference)."""
        metrics = {}
        for label, series in other.values.items():
            value = self.values[label][: len(series)]
            reference = series[: len(value)]
            span = float(np.ptp(reference)) or max(float(np.max(np.abs(reference))), 1.0)
            metrics[label] = float(np.sqrt(np.mean((value - reference) ** 2))) / span
        return metrics

    def energy_change(self, other: "OutputSeries", label: str) -> float | None:
        """Relative change of the integral of an output (a power) to ``other``."""
        reference = float(np.sum(other.values[label]))
        if abs(reference) < 1e-9:
            return None
        return float(np.sum(self.values[label][: len(other.values[label])])) / reference - 1


@dataclass
class ImpactResult:
    metrics: dict[str, float]
    energy: dict[str, float | None]

    @property
    def largest(self) -> tuple[str, float]:
        label = max(self.metrics, key=self.metrics.__getitem__)
        return label, self.metrics[label]

    @property
    def has_impact(self) -> bool:
        return self.largest[1] >= IMPACT_THRESHOLD

    def summary(self) -> str:
        label, value = self.largest
        powers = ", ".join(
            f"{name} {change:+.2%}"
            for name, change in self.energy.items()
            if change is not None and abs(change) >= 1e-4
        )
        return f"max NRMSD {value:.2e} ({label}){'; energy ' + powers if powers else ''}"


def check_impact(run: OutputSeries, baseline: OutputSeries, power_labels: Iterable[str]) -> ImpactResult:
    return ImpactResult(run.nrmsd(baseline), {label: run.energy_change(baseline, label) for label in power_labels})


@dataclass
class EquivalenceResult:
    ok: bool
    largest: float
    label: str
    missed: list[str] = field(default_factory=list)  # free parameters the recompiled model changes, not overridden
    evaluated: list[str] = field(default_factory=list)  # evaluated parameters the recompiled model changes

    def summary(self) -> str:
        text = f"max NRMSD {self.largest:.1e} ({self.label})"
        if self.missed:
            text += f"; not overridden: {', '.join(self.missed[:4])}{' ...' if len(self.missed) > 4 else ''}"
        if self.evaluated:
            text += f"; evaluated in the executable: {', '.join(self.evaluated[:4])}"
            text += " ..." if len(self.evaluated) > 4 else ""
        return text


def changed_start_values(baseline: CompiledModel, changed: CompiledModel) -> dict[str, ParameterKind]:
    """The parameters whose start values differ between two init files, with their kind in the baseline."""
    before = {variable.name: variable for variable in read_model_variables(baseline.init_xml).values()}
    differences = {}
    for name, variable in read_model_variables(changed.init_xml).items():
        old = before.get(name)
        if old is None or variable.variability != "parameter" or old.start == variable.start:
            continue
        try:
            if (
                old.start is not None
                and variable.start is not None
                and math.isclose(float(old.start), float(variable.start), rel_tol=1e-12)
            ):
                continue
        except ValueError:
            pass
        differences[name] = old.kind
    return differences


def build_changed(  # noqa: PLR0913
    description: Mapping[str, object],
    changes: Mapping[str, FieldValue],
    project_path: Path,
    name: str,
    library: str,
    options: SimulationOptions,
    container_name: str,
) -> CompiledModel:
    """Regenerate the model from the description with the YAML changes and compile it in its own project."""
    project_path.mkdir(parents=True, exist_ok=True)
    yaml_path = write_description(with_values(dict(description), dict(changes)), project_path / f"{name}.yaml")
    network = convert_network(name, yaml_path, library=Library.from_configuration(library))
    return build_model(project_path, network, options, container_name=container_name)


def check_equivalence(
    override_run: OutputSeries,
    recompiled_run: OutputSeries,
    baseline: CompiledModel,
    recompiled: CompiledModel,
    overrides: Collection[str],
) -> EquivalenceResult:
    metrics = recompiled_run.nrmsd(override_run)
    label = max(metrics, key=metrics.__getitem__)
    changed = changed_start_values(baseline, recompiled)
    missed = sorted(name for name, kind in changed.items() if kind == ParameterKind.free and name not in overrides)
    evaluated = sorted(name for name, kind in changed.items() if kind == ParameterKind.evaluated)
    return EquivalenceResult(metrics[label] <= EQUIVALENCE_TOLERANCE, metrics[label], label, missed, evaluated)


def window_run(
    runner: ModelRunner, overrides: Mapping[str, OverrideValue], label: str, outputs: Mapping[str, str]
) -> RunResult:
    """A run over the warm-up and the impact window, storing the outputs only."""
    return runner.run(
        overrides,
        label=label,
        start_time=0.0,
        stop_time=WARM_UP + WINDOW,
        step_size=STEP,
        variable_filter=variable_filter(outputs.values()),
    )


def readback_run(
    runner: ModelRunner, overrides: Mapping[str, OverrideValue], label: str, names: Iterable[str]
) -> RunResult:
    """A short run storing the given parameters."""
    return runner.run(
        overrides,
        label=label,
        start_time=0.0,
        stop_time=READBACK_STOP_TIME,
        variable_filter=filter_ if len(filter_ := variable_filter(names)) <= MAX_FILTER_LENGTH else ".*",
        flags=READBACK_FLAGS,
        check=True,
    )


__all__ = [
    "EQUIVALENCE_TOLERANCE",
    "IMPACT_THRESHOLD",
    "WARM_UP",
    "WINDOW",
    "EquivalenceResult",
    "ImpactResult",
    "OutputSeries",
    "ParameterDependencies",
    "ReadbackResult",
    "build_changed",
    "changed_start_values",
    "check_equivalence",
    "check_impact",
    "check_readback",
    "parameter_filter",
    "readback_run",
    "variable_filter",
    "window_run",
]
