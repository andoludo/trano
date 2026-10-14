"""The FastModel library: Modelica components and their YAML/jinja element templates, as Buildings and IDEAS.

trano writes ``building_mpc`` from the templates of ``trano/elements/library/models/fast_model``, with the
components of ``trano/library/FastModel.mo``; rumoca translates it into a CasADi ODE for the optimization package.
"""

import math
import re
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


def _model(path: Path, library: str = "FastModelMPC") -> str:
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        network = convert_network(path.stem, path, library=Library.from_configuration(library))
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


@needs_connected_export
@pytest.mark.parametrize(
    "path",
    [MODELS / "single_zone_hydronic.yaml", MODELS / "house_infiltration_air_water_heat_pump.yaml"],
    ids=lambda path: path.stem,
)
def test_rumoca_translates_the_closed_loop_model(path: Path) -> None:
    """The closed-loop model is an explicit ODE too: weather and gains in, no control."""
    model = CasadiModel.from_modelica(_model(path, "FastModel"), f"{path.stem}.building_mpc")
    description = model.description
    assert set(WEATHER) <= set(description.input_names)
    assert not Roles.infer(description).controls
    assert all(np.isfinite(description.initial_state))


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


@pytest.mark.parametrize("library", ["FastModel", "FastModelMPC"])
def test_unmodelled_systems_are_reported(library: str) -> None:
    with pytest.raises(UnsupportedElementError, match="not modelled by the FastModel library"):
        _model(MODELS / "single_zone_air_handling_unit_complex_vav.yaml", library)


def test_fast_model_mpc_is_fast_model_under_another_name() -> None:
    closed_loop, mpc = Library.from_configuration("FastModel"), Library.from_configuration("FastModelMPC")
    assert mpc.name == "FastModelMPC"
    assert mpc.base_library() == "FastModel"
    assert mpc.templates == closed_loop.templates
    assert mpc.package_template == closed_loop.package_template
    assert mpc.parameter_library == closed_loop.parameter_library


def _building_mpc(model: str) -> str:
    return model[model.index("model building_mpc") : model.index("end building_mpc;")]


@pytest.mark.parametrize(
    "path",
    [
        MODELS / "single_zone_hydronic.yaml",
        MODELS / "three_zones_ideal_heaters.yaml",
        MODELS / "house_infiltration_air_water_heat_pump.yaml",
        ROOT / "examples" / "residential_before_renovation.yaml",
        ROOT / "examples" / "residential_after_renovation.yaml",
    ],
    ids=lambda path: path.stem,
)
def test_the_closed_loop_model_runs_its_own_controllers(path: Path) -> None:
    """FastModel has no control input: its controllers drive the systems, the circuits are pressure-driven."""
    closed_loop = _building_mpc(_model(path, "FastModel"))
    assert "FastModel.Interfaces.ControlInput" not in closed_loop
    assert "FastModel.Interfaces.HeatFlowInput" not in closed_loop
    assert "FastModel.Controls." in closed_loop or "FastModel.HVAC.IdealHeater" in closed_loop
    assert "pressureDriven=true" in closed_loop or "Hydronics." not in closed_loop
    mpc = _building_mpc(_model(path))
    assert "FastModel.Controls." not in mpc
    assert "pressureDriven=true" not in mpc
    assert "FastModel.Interfaces.ControlInput" in mpc or "FastModel.Interfaces.HeatFlowInput" in mpc


def test_the_heat_pump_follows_its_control() -> None:
    model = _building_mpc(_model(MODELS / "house_infiltration_air_water_heat_pump.yaml", "FastModel"))
    assert "FastModel.Controls.HeatPumpControl" in model
    assert "FastModel.Controls.CollectorControl" in model
    assert "y=boiler_001_control.y" in model.replace(" ", "")


def test_the_monitored_set_point_is_held_without_external_data() -> None:
    model = _model(MODELS / "single_zone_hydronic_emission_from_data.yaml", "FastModel")
    assert 'FastModel.Interfaces.RealInput emission_control_001_TSet(unit="K")' in _building_mpc(model)
    assert "FastModel.Controls.EmissionMonitored emission_control_001(" in model
    assert "Modelica.Blocks.Sources.Constant emission_control_001_TSet(k=294.15)" in model
    assert "connect(emission_control_001_TSet.y, mpc.emission_control_001_TSet);" in model


def test_the_monitored_set_point_replays_its_column(tmp_path: Path) -> None:
    from trano.elements.bus import DataBus

    data = tmp_path / "set_point.csv"
    data.write_text(",other,temp_01\n2024-01-01 00:00:00,0,294.15\n2024-01-01 12:00:00,0,290.15\n")
    path = MODELS / "single_zone_hydronic_emission_from_data.yaml"
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        network = convert_network(path.stem, path, library=Library.from_configuration("FastModel"))
        model = network.model(data_bus=DataBus(external_data=data))
    assert "Modelica.Blocks.Sources.CombiTimeTable data_bus(" in model
    assert "connect(data_bus.y[2], mpc.emission_control_001_TSet);" in model
    assert "Sources.Constant emission_control_001_TSet" not in model


def _co2_occupancy_model(tmp_path: Path, data: Path | None) -> str:
    from trano.elements.bus import DataBus

    path = tmp_path / "co2_occupancy.yaml"
    source = (MODELS / "single_zone_hydronic_occupancy_from_data.yaml").read_text()
    path.write_text(
        source.replace("        data:\n", "        floor_area: 100.0\n        ach: 0.5\n        data:\n", 1)
    )
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        network = convert_network(path.stem, path, library=Library.from_configuration("FastModel"))
        return network.model(data_bus=DataBus(external_data=data) if data else None)


def test_the_co2_occupancy_replays_its_column(tmp_path: Path) -> None:
    data = tmp_path / "co2.csv"
    data.write_text(",other,co2_01\n2024-01-01 00:00:00,0,420\n2024-01-01 12:00:00,0,900\n")
    model = _co2_occupancy_model(tmp_path, data)
    assert "Trano.Occupancy.OccupancyCo2 occupancy_1(" in model
    assert "co2(u=data_bus.y[2])" in model
    assert "AFlo=100.0" in model


def test_the_co2_occupancy_follows_its_schedule_without_external_data(tmp_path: Path) -> None:
    """Without the CO2 column, the schedule is used without the parameters of the CO2 balance it does not have."""
    model = _co2_occupancy_model(tmp_path, None)
    declaration = model[model.index("Trano.Occupancy.SimpleOccupancy occupancy_1(") :].split(";")[0]
    assert "AFlo" not in declaration
    assert "ACH" not in declaration
    assert "OccupancyCo2 occupancy_1" not in model


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
@pytest.mark.parametrize(
    ("library", "path"),
    [
        ("FastModelMPC", MODELS / "single_zone_hydronic.yaml"),
        ("FastModel", MODELS / "single_zone_hydronic.yaml"),
        ("FastModel", MODELS / "single_zone_hydronic_emission_from_data.yaml"),
        ("FastModel", MODELS / "house_infiltration_air_water_heat_pump.yaml"),
        ("FastModel", ROOT / "examples" / "residential_after_renovation.yaml"),
    ],
    ids=lambda value: value.stem if isinstance(value, Path) else value,
)
def test_openmodelica_compiles_the_runnable_model(tmp_path: Path, library: str, path: Path) -> None:
    from trano.simulate.simulate import SimulationOptions, simulate
    from trano.utils.utils import is_success

    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        network = convert_network(path.stem, path, library=Library.from_configuration(library))
    options = SimulationOptions(check_only=True)
    results = simulate(tmp_path, network, options=options, container_name=f"compile_fast_model_{tmp_path.name}")
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


@pytest.mark.parametrize("library", ["FastModel", "FastModelMPC"])
def test_the_heat_meter_reads_the_return_sensor_without_its_water(library: str) -> None:
    """In Buildings the heat meter reads the return temperature as a signal (TExt): FastModel must not plumb the
    return line into the meter, which would let the water bypass the boiler."""
    model = _building_mpc(_model(MODELS / "house_complex_heat_meter.yaml", library)).replace(" ", "")
    meter = re.search(r"FastModel\.Hydronics\.TemperatureSensorheat_meter_sensor_001\((.*?)\)", model, re.DOTALL)
    assert meter is not None
    assert "nPorts_a=1" in meter.group(1) and "nPorts_b=1" in meter.group(1)
    assert re.findall(r"connect\((\w+)\.port_b\[\d\],heat_meter_sensor_001\.port_a\[\d\]\)", model) == ["boiler_001"]
    assert re.findall(r"connect\(heat_meter_sensor_001\.port_b\[\d\],(\w+)\.port_a\[\d\]\)", model) == ["pump_001"]


@pytest.mark.parametrize("library", ["FastModel", "FastModelMPC"])
def test_the_fans_take_the_nominal_pressure_rise_by_default(tmp_path: Path, library: str) -> None:
    """Without supply_dp_nominal and return_dp_nominal, both fans raise dp_nominal, as in Buildings and IDEAS
    (an unset parameter used to render an empty modifier, ``dpSup_nominal=,``, which OpenModelica cannot parse)."""
    import yaml

    data = yaml.safe_load((ROOT / "examples" / "residential_after_renovation.yaml").read_text())
    [ahu] = [system["air_handling_unit"] for system in data["systems"] if "air_handling_unit" in system]
    del ahu["parameters"]["supply_dp_nominal"], ahu["parameters"]["return_dp_nominal"]
    path = tmp_path / "ahu_default_fan_pressures.yaml"
    path.write_text(yaml.safe_dump(data))
    model = _building_mpc(_model(path, library)).replace(" ", "")
    unit = re.search(r"FastModel\.Ventilation\.HeatRecoveryUnitahu_001\((.*?)\)", model, re.DOTALL)
    assert unit is not None
    assert "dpSup_nominal=100," in unit.group(1) and "dpRet_nominal=100," in unit.group(1)


def test_the_heat_pump_control_takes_the_heating_curve_of_the_yaml(tmp_path: Path) -> None:
    """minimum_supply_temperature, heating_limit_temperature and buffer_temperature_band reach the control, as in
    Buildings; they used to be dropped, leaving the curve at its 30 degC floor and 16 degC limit."""
    import yaml

    data = yaml.safe_load((MODELS / "house_infiltration_air_water_heat_pump.yaml").read_text())
    [boiler] = [system["boiler"] for system in data["systems"] if "boiler" in system]
    boiler["parameters"].update(
        {"minimum_supply_temperature": 298.15, "heating_limit_temperature": 291.15, "buffer_temperature_band": 3}
    )
    path = tmp_path / "heat_pump_heating_curve.yaml"
    path.write_text(yaml.safe_dump(data))
    model = _building_mpc(_model(path, "FastModel")).replace(" ", "")
    control = re.search(r"FastModel\.Controls\.HeatPumpControlboiler_001_control\((.*?)\)", model, re.DOTALL)
    assert control is not None
    assert "TSet=318.15," in control.group(1)
    assert "TSupMin=298.15," in control.group(1)
    assert "TOutHeaLim=291.15," in control.group(1)
    assert "dTHys=3" in control.group(1)


def test_the_heat_pump_control_keeps_its_defaults_when_the_yaml_sets_none() -> None:
    model = _building_mpc(_model(MODELS / "house_infiltration_air_water_heat_pump.yaml", "FastModel")).replace(" ", "")
    control = re.search(r"FastModel\.Controls\.HeatPumpControlboiler_001_control\((.*?)\)", model, re.DOTALL)
    assert control is not None
    assert not re.search(r"\b(TSupMin|TOutHeaLim|dTHys)=", control.group(1))
