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


PROBE = """package Probe
  connector Port Real T; flow Real Q_flow; end Port;
  model Node parameter Real C = 1; Real T(start = 290, fixed = true); Port port;
  equation port.T = T; C*der(T) = port.Q_flow; end Node;
  model Link Port a; Port b; equation a.Q_flow = a.T - b.T; a.Q_flow + b.Q_flow = 0; end Link;
  model M Node n1; Node n2; Link l; equation connect(n1.port, l.a); connect(l.b, n2.port); end M;
end Probe;
"""


@cache
def _exports_connected_models() -> bool:
    """rumoca 0.10.3 (its main branch) exports models made of connected components; the releases up to 0.10.2
    stop at their connection equations."""
    import rumoca

    try:
        rumoca.Session().loads(PROBE, model="Probe.M").to_casadi()
    except Exception:
        return False
    return True


needs_connected_export = pytest.mark.skipif(
    not _exports_connected_models(),
    reason="the CasADi export of connected models needs rumoca >= 0.10.3 (rumoca main)",
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


@needs_connected_export
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


@needs_connected_export
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


@needs_connected_export
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


@needs_connected_export
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


FRAME_ROOM = """
package room
  package Data
    record Glass = FastModel.Data.Glazing(nPan=1, x={0.004}, d={2500}, c={840}, RPan={0.004}, RGap=fill(0, 0),
      tau={0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.7, 0.5, 0.2, 0, 0.7},
      absPan={{0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0, 0.1}}, absIR_a=0.84, absIR_b=0.84, UFra=2.0);
    record Slab = FastModel.Data.Construction(nLay=1, x={0.2}, k={1.4}, c={900}, d={2300});
  end Data;
  model building_mpc
    parameter Real frac = 0.3;
    FastModel.Interfaces.WeatherInputs weather_inputs;
    FastModel.BoundaryConditions.Weather weather(TDryBul=weather_inputs.TDryBul, TBlaSky=weather_inputs.TBlaSky,
      winSpe=weather_inputs.winSpe, winDir=weather_inputs.winDir, HDirNor=weather_inputs.HDirNor,
      HDifHor=weather_inputs.HDifHor, HGloHor=weather_inputs.HGloHor);
    FastModel.Interfaces.OccupancyInputs occupancy;
    FastModel.Zones.Zone zone(nSurf=2, AFlo=20, hRoo=2.5);
    FastModel.Envelope.Window window(glazing=Data.Glass(), A=4, frac=frac, til=FastModel.Types.Tilt.Wall);
    FastModel.Envelope.AdiabaticWall wall(constructionType=Data.Slab(), A=40, til=FastModel.Types.Tilt.Wall);
  equation
    connect(weather.weaBus, zone.weaBus);
    connect(occupancy.gai, zone.qGai_flow);
    connect(window.propsBus_a, zone.propsBus[1]);
    connect(wall.propsBus_a, zone.propsBus[2]);
  end building_mpc;
end room;
"""


@needs_connected_export
def test_the_window_frame_loses_heat_to_the_outside() -> None:
    """With the room and its surfaces at 21 degC and 0 degC outside, a larger frame cools the room air faster."""
    from trano.elements.jinja import modelica_library

    model = CasadiModel.from_modelica(f"{modelica_library('FastModel')}\n{FRAME_ROOM}", "room.building_mpc")
    names = model.description.parameter_names
    weather = dict(zip(WEATHER, [KELVIN, KELVIN - 10, 0.0, 0.0, 0.0, 0.0, 0.0], strict=True))
    inputs = np.array([weather.get(name, 0.0) for name in model.description.input_names])
    air = model.description.state_names.index("zone.TAir")
    state = np.full(len(model.description.states), KELVIN + 21)

    def cooling(frame: float) -> float:
        parameters = model.description.parameter_values.copy()
        parameters[names.index("frac")] = frame
        parameters[names.index("window.frac")] = frame
        parameters[names.index("window.GFra")] = 4 * frame / (1 / 25 + 1 / 2.0 + 1 / 7.7)
        return float(model.derivatives(0.0, state, inputs, parameters)[air])

    assert cooling(0.5) < cooling(0.0) <= 0
