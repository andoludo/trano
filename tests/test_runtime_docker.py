"""Compile once, change parameters at run time: the checks on the compiled executable (Docker, OpenModelica).

    uv run pytest -m runtime

The baseline of examples/residential_after_renovation.yaml is compiled once per session and library
(Buildings only so far) for two January weeks after two days of warm-up; every check of a parameter
runs that executable. The parameters are a sample covering every group of the inventory, the
calibration targets first (see RUNTIME_PARAMETERS.md); the full inventory is written by
``trano parameter-inventory``.
"""

import math
import tempfile
from collections.abc import Iterator
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pytest
from buildingspy.io.outputfile import Reader  # type: ignore

from trano.data_models.conversion import convert_network
from trano.elements.library.library import Library
from trano.runtime.adapters import get_adapter
from trano.runtime.checks import (
    DAY,
    WARM_UP,
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
from trano.runtime.init_xml import read_model_variables
from trano.runtime.inventory import free_overrides
from trano.runtime.model import CompiledModel, ModelRunner, build_model
from trano.runtime.resolution import SpecificationResolver
from trano.runtime.study import window_options
from trano.simulate.simulate import SimulationOptions, simulate
from trano.utils.utils import is_success

pytestmark = pytest.mark.runtime

EXAMPLE = Path(__file__).parents[1].joinpath("examples", "residential_after_renovation.yaml")
LIBRARIES = ["Buildings"]
SMOKE_END = 4 * DAY  # two days of warm-up and two January days


def docker_available() -> bool:
    try:
        import docker  # type: ignore

        docker.from_env().ping()
    except Exception:  # any failure means no usable daemon
        return False
    return True


requires_docker = pytest.mark.skipif(not docker_available(), reason="Docker is not available")


@dataclass
class Baseline:
    library: str
    compiled: CompiledModel
    resolver: SpecificationResolver
    outputs: dict[str, str]
    runner: ModelRunner
    series: OutputSeries
    dependencies: ParameterDependencies


@pytest.fixture(scope="session", params=LIBRARIES)
def baseline(request: pytest.FixtureRequest, tmp_path_factory: pytest.TempPathFactory) -> Iterator[Baseline]:
    """The example compiled once for the impact window, with a container running its executable."""
    if not docker_available():
        pytest.skip("Docker is not available")
    library = request.param
    project = tmp_path_factory.mktemp(f"runtime_{library}")
    network = convert_network(EXAMPLE.stem, EXAMPLE, library=Library.from_configuration(library))
    model = network.model()
    adapter = get_adapter(library)
    outputs = adapter.impact_outputs(network)
    paths = [entry.modelica_path for entry in adapter.parameter_snapshot(network).values()]
    options = window_options().model_copy(update={"variable_filter": parameter_filter(paths, outputs.values())})
    compiled = build_model(project, network, options, model=model, specification=EXAMPLE)
    with ModelRunner(compiled, container_name=f"trano_runtime_test_{library}") as runner:
        series = OutputSeries.read(window_run(runner, {}, "window_baseline", outputs).result_path, outputs)
        yield Baseline(
            library,
            compiled,
            SpecificationResolver.from_file(EXAMPLE, library),
            outputs,
            runner,
            series,
            ParameterDependencies(compiled.info_json),
        )


@requires_docker
def test_the_executable_runs_again_and_matches_the_regular_simulation(baseline: Baseline) -> None:
    """Smoke test: a short heating-season run of the compiled executable, finite, plausible, and the same as
    trano's regular simulation of the same model."""
    run = baseline.runner.run(label="smoke", start_time=0, stop_time=SMOKE_END, step_size=900, strict=True)
    smoke = OutputSeries.read(run.result_path, baseline.outputs)
    for label, values in smoke.values.items():
        assert np.all(np.isfinite(values)), label
    for label in (label for label in baseline.outputs if label.startswith("T_")):
        assert 285 < float(np.mean(smoke.values[label])) < 300, label  # 12-27 degC in heated rooms
    assert float(np.mean(smoke.values["Q_generator_001"])) > 0  # the heat pump heats in January
    assert float(np.mean(smoke.values["P_generator_001"])) > 0

    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as project:
        network = convert_network(EXAMPLE.stem, EXAMPLE, library=Library.from_configuration(baseline.library))
        options = SimulationOptions(end_time=SMOKE_END, tolerance=1e-4, number_of_intervals=int(SMOKE_END / 900))
        assert is_success(simulate(Path(project), network, options=options, container_name="trano_runtime_regular"))
        result = next(Path(project).joinpath("results").glob("*.mat"))
        regular = OutputSeries.read(result, baseline.outputs)
    differences = smoke.nrmsd(regular)
    assert max(differences.values()) < 1e-6, differences


@requires_docker
def test_an_ignored_override_never_passes_as_a_change(baseline: Baseline) -> None:
    til = "building.solar1.pv_001.til"  # Evaluate=true in Buildings
    run = readback_run(baseline.runner, {til: 0.5}, "ignored", [til])
    assert [message.name for message in run.ignored] == [til]
    assert not check_readback(run, {til: 0.5}, [], {}).ok


@dataclass(frozen=True)
class Case:
    """A field of the example, its perturbed value and what the checks must find."""

    path: str
    value: float | list[float] | str
    impact: bool
    equivalent: bool

    def __str__(self) -> str:
        return self.path


SPACE = "spaces[LIVING:001]"
CONTROL = f"{SPACE}.emissions[VALVE_LIVING:001].valve.control.emission_control.parameters"
CASES = [
    # high priority: calibration targets
    Case("material[EPS:001].thermal_conductivity", 0.0338, impact=True, equivalent=True),
    Case(f"{SPACE}.parameters.n50", 3.6, impact=True, equivalent=True),
    Case(f"{SPACE}.external_boundaries.floor_on_grounds[0].ground_temperature", 288.65, impact=True, equivalent=True),
    Case("glass_material[GLASS:001].solar_transmittance", [0.664], impact=True, equivalent=True),
    Case("glazings[WINDOW:001].u_value_frame", 1.32, impact=True, equivalent=True),
    Case(f"{CONTROL}.temperature_heating_setpoint", 296.15, impact=True, equivalent=True),
    Case(
        "systems[GENERATOR:001].boiler.parameters.temperature_supply_setpoint_heat_pump",
        315.15,
        impact=True,
        equivalent=True,
    ),
    Case("systems[GENERATOR:001].boiler.parameters.nominal_heating_power", 3600.0, impact=True, equivalent=True),
    Case(
        "systems[AHU:001].air_handling_unit.parameters.heat_exchanger_effectiveness", 0.64, impact=True, equivalent=True
    ),
    # medium priority
    Case("material[BRICK_INNER:001].density", 1920.0, impact=True, equivalent=True),
    Case("constructions[EXTERNAL_WALL:001].layers[3].thickness", 0.154, impact=True, equivalent=True),
    Case(f"{SPACE}.parameters.floor_area", 48.0, impact=True, equivalent=True),
    Case(f"{CONTROL}.schedule", "{27000, 84600}", impact=True, equivalent=True),
    Case("solar[PV:001].photovoltaic.parameters.efficiency", 0.16, impact=True, equivalent=True),
    # inert in this scenario: the rooms never cool down to the setback in two January weeks
    Case(f"{CONTROL}.temperature_heating_setback", 287.15, impact=False, equivalent=True),
    # frozen: the override cannot stand for the change
    Case("material[EPS:001].thermal_conductivity", 0.0384, impact=True, equivalent=False),  # nSta 2 -> 1
    Case(
        f"{SPACE}.emissions[RADIATOR_LIVING:001].radiator.parameters.nominal_heating_power_positive_for_heating",
        4680.0,
        impact=True,
        equivalent=False,
    ),
]


@requires_docker
@pytest.mark.parametrize("case", CASES, ids=[f"{case.path}={case.value}" for case in CASES])
def test_a_yaml_change_is_a_run_time_override(baseline: Baseline, case: Case) -> None:
    """Readback, impact and equivalence of one field on the baseline executable."""
    resolution = baseline.resolver.resolve(case.path, case.value)
    variables = read_model_variables(baseline.compiled.init_xml, resolution.runtime_names)
    overrides = free_overrides(resolution, variables)
    assert overrides, f"no free run-time parameter for {case.path}"
    label = f"case_{abs(hash((case.path, str(case.value)))):x}"

    dependents = baseline.dependencies.of(overrides)
    reference = readback_run(baseline.runner, {}, f"{label}_reference", dependents).reader() if dependents else None
    before = {name: float(reference.values(name)[1][0]) for name in dependents if _has(reference, name)}
    readback = check_readback(
        readback_run(baseline.runner, overrides, f"{label}_readback", [*overrides, *dependents]),
        overrides,
        dependents,
        before,
    )
    assert readback.ok, readback.reason

    run = window_run(baseline.runner, overrides, f"{label}_window", baseline.outputs)
    series = OutputSeries.read(run.result_path, baseline.outputs)
    impact = check_impact(
        series, baseline.series, [label for label in baseline.outputs if label.startswith(("Q_", "P_"))]
    )
    assert impact.has_impact == case.impact, impact.summary()

    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as project:
        recompiled = build_changed(
            baseline.resolver.description,
            {case.path: case.value},
            Path(project),
            baseline.resolver.name,
            baseline.library,
            window_options(),
            container_name=f"trano_runtime_test_eq_{baseline.library}",
        )
        with ModelRunner(recompiled, container_name=f"trano_runtime_test_eq_run_{baseline.library}") as runner:
            recompiled_series = OutputSeries.read(
                window_run(runner, {}, "window", baseline.outputs).result_path, baseline.outputs
            )
        equivalence = check_equivalence(series, recompiled_series, baseline.compiled, recompiled, overrides)
    assert equivalence.ok == case.equivalent, equivalence.summary()
    if case.equivalent:
        assert not equivalence.missed


def _has(reader: Reader | None, name: str) -> bool:
    try:
        return reader is not None and not math.isnan(float(reader.values(name)[1][0]))
    except (KeyError, ValueError, IndexError):
        return False


@requires_docker
def test_runs_start_after_the_warm_up(baseline: Baseline) -> None:
    assert next(iter(baseline.series.values.values())).size == int((14 * DAY) / 900) + 1
    assert WARM_UP == 2 * DAY
