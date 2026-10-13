"""The FastModel library: Modelica components and their YAML/jinja element templates, as Buildings and IDEAS.

trano writes ``building_mpc`` from the templates of ``trano/elements/library/models/fast_model``, with the
components of ``trano/library/FastModel.mo``; rumoca translates it into a CasADi ODE for the optimization package.
"""

import math
import warnings
from functools import cache
from pathlib import Path

import numpy as np
import pytest

from trano.data_models.conversion import convert_network
from trano.elements.jinja import UnsupportedElementError
from trano.elements.library.library import Library
from trano.optimization import KELVIN, CasadiModel, OptimalControl, OptimalControlSettings, Roles

ROOT = Path(__file__).parents[1]
MODELS = ROOT / "tests" / "models"
BESTEST = ROOT / "validation" / "bestest" / "cases"
WEATHER = (
    "weather_inputs.TDryBul",
    "weather_inputs.TBlaSky",
    "weather_inputs.winSpe",
    "weather_inputs.winDir",
    "weather_inputs.HDirNor",
    "weather_inputs.HDifHor",
    "weather_inputs.HGloHor",
)


def _model(path: Path) -> str:
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        network = convert_network(path.stem, path, library=Library.from_configuration("FastModel"))
        return network.model()


@cache
def _export(path: Path) -> CasadiModel:
    return CasadiModel.from_modelica(_model(path), f"{path.stem}.building_mpc")


def test_fast_model_is_templates_and_modelica() -> None:
    assert not (ROOT / "trano" / "fast_model").exists()
    templates = ROOT / "trano" / "elements" / "library" / "models" / "fast_model"
    assert {path.stem for path in templates.glob("*.yaml")} >= {"space", "envelope", "hydronics", "ventilation"}
    assert (ROOT / "trano" / "library" / "FastModel.mo").read_text().startswith("package FastModel")


def test_the_package_holds_the_model_and_its_runnable_wrapper() -> None:
    model = _model(MODELS / "single_zone_hydronic.yaml")
    assert "package FastModel" in model
    assert "model building_mpc" in model
    assert "building_mpc mpc(weather(lat=weather_reader.lat" in model
    assert "Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weather_reader" in model
    building_mpc = model[model.index("model building_mpc") : model.index("end building_mpc;")]
    assert "Buildings." not in building_mpc  # the physics only: no table, no event, no Buildings component


@pytest.mark.parametrize(
    ("path", "states"),
    [(BESTEST / "case_600FF.yaml", 25), (MODELS / "single_zone_hydronic.yaml", 27)],
    ids=["600FF", "single_zone_hydronic"],
)
def test_rumoca_translates_the_model(path: Path, states: int) -> None:
    description = _export(path).description
    assert len(description.states) == states
    assert description.input_names[:0] == []
    assert set(WEATHER) <= set(description.input_names)
    assert all(np.isfinite(description.initial_state))


def test_the_roles_come_from_the_model() -> None:
    model = _export(MODELS / "single_zone_hydronic.yaml")
    roles = Roles.infer(model.description)
    assert roles.comfort == ("space_001.TAir",)
    gains = set(roles.disturbances) - set(WEATHER)
    assert len(gains) == 2
    assert {name.rsplit(".", 1)[-1] for name in gains} == {"QCon", "QRad"}
    assert all(name.endswith("_y.y") for name in roles.controls)
    for name in roles.controls:
        variable = model.description.input(name)
        assert (variable.minimum, variable.maximum) == (0, 1)
    assert set(roles.meters.values()) <= {"fuel", "electricity"}
    assert roles.meters


def _radiator_conductance(power: float, t_in: float, t_out: float, fraction: float, exponent: float) -> float:
    """UAEle of one element of Buildings RadiatorEN442_2, solved independently of the Modelica function."""
    from scipy.optimize import brentq

    flow = power / 4184 / (t_in - t_out)

    def delivered(conductance: float) -> float:
        def balance(t_water: float) -> float:
            emitted = conductance * (
                fraction * max(t_water - 293.15, 0) ** exponent + (1 - fraction) * max(t_water - 293.15, 0) ** exponent
            )
            return t_water - t_in + emitted / flow / 4184

        t_water = brentq(balance, 293.15, t_in)
        return conductance * max(t_water - 293.15, 0) ** exponent

    return float(brentq(lambda conductance: delivered(conductance) - power, 1e-3, 1e3))


def test_the_radiator_conductance_is_solved_as_buildings() -> None:
    model = _export(MODELS / "single_zone_hydronic.yaml")
    values = dict(zip(model.description.parameter_names, model.description.parameter_values, strict=True))
    name = next(name for name in values if name.endswith(".UAEle"))
    radiator = name.rsplit(".", 1)[0]
    expected = _radiator_conductance(
        values[f"{radiator}.Q_flow_nominal"],
        values[f"{radiator}.T_a_nominal"],
        values[f"{radiator}.T_b_nominal"],
        values[f"{radiator}.fraRad"],
        values[f"{radiator}.n"],
    )
    if values[f"{radiator}.nEle"] == 1:
        assert values[name] == pytest.approx(expected, rel=1e-6)
    assert values[name] > 0


def test_unmodelled_systems_are_reported() -> None:
    with pytest.raises(UnsupportedElementError, match="not modelled by the FastModel library"):
        _model(MODELS / "single_zone_air_handling_unit_complex_vav.yaml")


def test_the_optimal_control_heats_the_zone() -> None:
    model = _export(MODELS / "single_zone_hydronic.yaml")
    ocp = OptimalControl(model, settings=OptimalControlSettings(time_step=3600))
    hours = 8
    disturbances = np.zeros((len(ocp.roles.disturbances), hours))
    weather = dict(zip(WEATHER, [KELVIN, KELVIN - 10, 3.0, math.pi, 0.0, 0.0, 0.0], strict=True))
    for row, name in enumerate(ocp.roles.disturbances):
        disturbances[row] = weather.get(name, 0.0)
    initial = model.description.initial_state
    comfort = np.full(hours, KELVIN + 21)
    result = ocp.solve(initial, disturbances, comfort, price=np.full(hours, 0.1), start_time=0.0)
    assert result.success, result.status
    zone = model.description.state_names.index("space_001.TAir")
    assert result.states[zone, 3:].min() > KELVIN + 21 - 0.1
    simulated = ocp.simulate(initial, result.controls, disturbances)
    assert simulated[zone] == pytest.approx(result.states[zone], abs=0.2)


@pytest.mark.compile
def test_openmodelica_compiles_the_runnable_model(tmp_path: Path) -> None:
    from trano.simulate.simulate import SimulationOptions, simulate
    from trano.utils.utils import is_success

    path = MODELS / "single_zone_hydronic.yaml"
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        network = convert_network(path.stem, path, library=Library.from_configuration("FastModel"))
    options = SimulationOptions(check_only=True)
    results = simulate(tmp_path, network, options=options, container_name="compile_fast_model")
    assert is_success(results, options=options), results.output.decode(errors="replace")[-3000:]
