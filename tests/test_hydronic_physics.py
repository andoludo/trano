"""Simulated regressions of the hydronic findings behind the checks of `trano/validation/hydronics.py`.

Three winter days of `tests/models/single_zone_hydronic.yaml` with one change each (about two minutes per
simulation with the libraries installed, see `TRANO_MODELICA_LIBRARIES`).
"""

import copy
import tempfile
import warnings
from collections.abc import Iterator
from pathlib import Path
from typing import Any

import numpy as np
import pytest
import yaml
from buildingspy.io.outputfile import Reader  # type: ignore

from trano.data_models import conversion
from trano.data.include import Loader
from trano.data_models.conversion import convert_network
from trano.simulate.simulate import SimulationOptions, simulate
from trano.utils.utils import is_success
from trano.validation.issues import ModelWarning

MODEL = Path(__file__).parent / "models" / "single_zone_hydronic.yaml"
THREE_DAYS = 3 * 24 * 3600
ROOM = "building.bus1.dataBus.TZonSpace_001"
THREE_WAY_VALVE = "building.distribution1.three_way_valve_001.y"


def _model(boiler_supply: float, **radiator: float) -> dict[str, Any]:
    model = yaml.load(MODEL.read_text(), Loader=Loader)  # noqa: S506
    model = copy.deepcopy(model)
    boiler = next(entry["boiler"] for entry in model["systems"] if "boiler" in entry)
    boiler["control"]["boiler_control"]["parameters"]["tsup_nominal"] = boiler_supply
    emissions = model["spaces"][0]["emissions"]
    emissions[0]["radiator"]["parameters"].update(radiator)
    return model


def _simulate(model: dict[str, Any], project: Path, name: str) -> Any:  # noqa: ANN401
    path = project / f"{name}.yaml"
    path.write_text(yaml.safe_dump(model, sort_keys=False))
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", ModelWarning)
        network = convert_network(name, path)
    return simulate(project, network, options=SimulationOptions(end_time=THREE_DAYS))


@pytest.fixture
def project() -> Iterator[Path]:
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as directory:
        yield Path(directory)


def _occupied(reader: Reader, variable: str) -> np.ndarray:
    time, values = reader.values(variable)
    occupied = (time % 86400 >= 10 * 3600) & (time % 86400 <= 20 * 3600) & (time > 86400)
    return values[occupied]


@pytest.mark.simulate
def test_diverting_three_way_valve_keeps_the_rooms_warm_with_the_default_boiler(project: Path) -> None:
    """With the 80 degC default boiler set point, the tank top reaches 95 degC. Before the valve set point was
    raised above it, the valve closed (y -> 0) and the room fell to 14.5 degC (20.2 degC on average)."""
    results = _simulate(_model(boiler_supply=353.15), project, "diverting_80")
    assert is_success(results)
    reader = Reader(str(next(project.joinpath("results").glob("*.mat"))), "dymola")
    assert reader.values(THREE_WAY_VALVE)[1].min() > 0.99
    assert _occupied(reader, ROOM).mean() > 273.15 + 21


@pytest.mark.simulate
def test_a_model_that_fails_to_build_reports_why(project: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """A radiator with equal nominal inlet and outlet temperatures, past the validation: the simulation fails
    on the assertion of the Modelica library, which is reported instead of "Failed to build model"."""
    monkeypatch.setattr(conversion, "validate_semantics", lambda *args, **kwargs: None)
    results = _simulate(
        _model(boiler_supply=343.15, water_inlet_temperature_at_nominal_condition=333.15), project, "equal_dt"
    )
    assert not is_success(results)
    assert "T_a_nominal must be higher than T_b_nominal" in results.output.decode()
