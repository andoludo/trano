"""FMU export and calibration checks on the Buildings FMU of the renovated house (Docker, OpenModelica, FMPy).

Run with ``pytest -m fmu``. The baseline FMU is built once per session (as long as the model takes to
compile in OpenModelica, tens of minutes for six zones); ``TRANO_FMU_BASELINE=<path>`` reuses a built one.
The library is a parameter of the session fixture so that IDEAS can be added without restructuring.
"""

from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

import pytest

from trano.data_models.conversion import convert_network
from trano.elements.library.library import Library
from trano.fmu.calibration import InventoryBuilder, InventorySettings
from trano.fmu.checks import DAY, Window, max_relative_rms, simulate_fmu
from trano.fmu.export import FmuExportOptions, export_fmu
from trano.fmu.model_description import fmu_resources, read_model_description
from trano.simulate.simulate import SimulationOptions, simulate
from trano.utils.utils import is_success
from validation.fmu.study import run_study

EXAMPLE = Path(__file__).parents[1] / "examples" / "residential_after_renovation.yaml"
LIBRARIES = ["Buildings"]
WINDOW = Window(start_time=0.0, warm_up=1 * DAY, duration=2 * DAY, output_interval=600.0)
"""Short horizon for the tests: the full two-week January window is run by ``python -m validation.fmu``."""
OMC_TOLERANCE = 0.02
"""Relative RMS difference (powers) tolerated between the FMU and the OpenModelica run of the same model."""
READBACK_SAMPLE = [
    "material[EPS:001].thermal_conductivity",
    "spaces[LIVING:001].parameters.n50",
    "spaces[LIVING:001].external_boundaries.floor_on_grounds[0].ground_temperature",
    "spaces[LIVING:001].emissions[0].radiator.parameters.nominal_heating_power_positive_for_heating",
    "spaces[LIVING:001].emissions[1].valve.control.emission_control.parameters.temperature_heating_setpoint",
    "systems[0].boiler.parameters.nominal_heating_power",
    "systems[5].air_handling_unit.parameters.heat_exchanger_effectiveness",
    "solar[0].photovoltaic.parameters.area",
]
EQUIVALENCE_SAMPLE = [
    "spaces[LIVING:001].emissions[1].valve.control.emission_control.parameters.temperature_heating_setpoint",
]

pytestmark = pytest.mark.fmu


def docker_available() -> bool:
    if shutil.which("docker") is None:
        return False
    try:
        return subprocess.run(["docker", "info"], capture_output=True, check=False, timeout=30).returncode == 0  # noqa: S607
    except (OSError, subprocess.TimeoutExpired):
        return False


@pytest.fixture(scope="session", params=LIBRARIES)
def baseline(request: pytest.FixtureRequest, tmp_path_factory: pytest.TempPathFactory) -> InventoryBuilder:
    """The baseline FMU of the example for one library, with the builder that checks parameters against it."""
    if not docker_available():
        pytest.skip("Docker is not available")
    pytest.importorskip("fmpy")
    library_name: str = request.param
    directory = tmp_path_factory.mktemp(f"fmu_{library_name.lower()}")
    reused = os.environ.get("TRANO_FMU_BASELINE")
    if reused and Path(reused).exists():
        fmu = Path(reused)
    else:
        library = Library.from_configuration(library_name)
        network = convert_network(EXAMPLE.stem, EXAMPLE, library=library)
        fmu = export_fmu(directory, network, options=FmuExportOptions(library_name=library_name))
    settings = InventorySettings(
        yaml_path=EXAMPLE, library_name=library_name, fmu=fmu, work_directory=directory, window=WINDOW
    )
    return InventoryBuilder(settings)


def test_the_fmu_is_a_portable_fmi2_co_simulation_fmu(baseline: InventoryBuilder) -> None:
    description = read_model_description(baseline.fmu)
    assert description.fmi_version == "2.0"
    assert description.co_simulation
    assert description.generation_tool and "OpenModelica" in description.generation_tool
    # the weather file of the model is packed into the FMU, not read from a host path
    resources = fmu_resources(baseline.fmu)
    assert any(name.endswith(".mos") for name in resources), resources
    assert description.parameters(), "no settable parameter in the FMU"


def test_the_fmu_simulates_from_another_directory_and_matches_openmodelica(
    baseline: InventoryBuilder, tmp_path: Path
) -> None:
    run = baseline.baseline()  # FMPy extracts the FMU into a scratch directory: nothing from the build is used
    assert len(run) > 0
    assert run.notna().all().all()
    temperatures = run[list(baseline.outputs.temperatures)]
    assert ((temperatures > 273.15) & (temperatures < 310)).all().all()

    network = convert_network(EXAMPLE.stem, EXAMPLE, library=baseline.library)
    options = SimulationOptions(
        start_time=int(WINDOW.start_time),
        end_time=int(WINDOW.stop_time),
        tolerance=WINDOW.tolerance,
        number_of_intervals=int((WINDOW.stop_time - WINDOW.start_time) / WINDOW.output_interval),
    )
    result = simulate(tmp_path, network, options=options, container_name="omc_fmu_reference")
    assert is_success(result)
    reference = _read_openmodelica(tmp_path / "results" / f"{EXAMPLE.stem}.building_res.mat", run.columns, run.index)
    assert max_relative_rms(reference, run) < OMC_TOLERANCE


def _read_openmodelica(path: Path, columns: list[str], times: object) -> object:
    import numpy as np
    import pandas as pd
    from buildingspy.io.outputfile import Reader

    reader = Reader(str(path), "dymola")
    frame = {}
    for column in columns:
        time, values = reader.values(column)
        frame[column] = np.interp(np.asarray(times, dtype=float), time, values)
    return pd.DataFrame(frame, index=times)


@pytest.mark.parametrize("yaml_path", READBACK_SAMPLE)
def test_parameter_readback_and_impact(baseline: InventoryBuilder, yaml_path: str) -> None:
    """The override is read back after initialisation and moves the outputs."""
    case = next(case for case in baseline.resolve() if case.resolved.yaml_path == yaml_path)
    assert case.candidate, case.row.reason
    assert case.start_values, "no FMU parameter to set"
    readback = baseline.readback(case)
    assert readback.readback_ok, readback.readback_detail
    impact, run = baseline.impact(case)
    assert run is not None, impact.impact
    assert impact.impact_significant, impact.impact


@pytest.mark.parametrize("yaml_path", EQUIVALENCE_SAMPLE)
def test_parameter_equivalence(baseline: InventoryBuilder, yaml_path: str, tmp_path: Path) -> None:
    """Changing the YAML and rebuilding the FMU gives the run of the baseline FMU with the override."""
    inventory = run_study(baseline, tmp_path / "results.json", equivalence=[yaml_path], impact=False)
    row = next(row for row in inventory.rows() if row.yaml_path == yaml_path)
    assert row.equivalence_ok == "yes", row.reason


def test_simulate_fmu_rejects_a_missing_output(baseline: InventoryBuilder) -> None:
    with pytest.raises(Exception, match=r"does not exist|not found|Unknown|KeyError|variable"):
        simulate_fmu(baseline.fmu, ["building.not_a_variable"], WINDOW)
