"""The FastModel library: Modelica components and their YAML/jinja element templates, as Buildings and IDEAS.

trano writes ``building_mpc`` from the templates of ``trano/elements/library/models/fast_model``, with the
components of ``trano/library/FastModel.mo``; rumoca translates it into a CasADi ODE for trano-optimization. The
tests of that translation run when trano-optimization is installed (it is not a dependency of trano).
"""

from __future__ import annotations

import math
import re
import tempfile
import warnings
from functools import cache
from pathlib import Path

import numpy as np
import pytest

from trano.data_models.conversion import convert_network
from trano.elements.jinja import UnsupportedElementError
from trano.elements.library.library import Library
from trano.simulate.simulate import SimulationOptions, simulate

try:
    from trano_optimization import CasadiModel, OptimalControl, OptimalControlSettings, Roles
except ImportError:  # trano-optimization (with rumoca) is not a dependency of trano: its tests are skipped
    CasadiModel = OptimalControl = OptimalControlSettings = Roles = None  # type: ignore[assignment,misc]

KELVIN = 273.15

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
    """trano-optimization is installed, with rumoca 0.10.3 (its main branch), which exports models made of connected
    components; the releases up to 0.10.2 stop at their connection equations."""
    if CasadiModel is None:
        return False
    import rumoca

    try:
        rumoca.Session().loads(PROBE, model="Probe.M").to_casadi()
    except Exception:
        return False
    return True


needs_connected_export = pytest.mark.skipif(
    not _exports_connected_models(),
    reason="the CasADi export of connected models needs trano-optimization, with rumoca >= 0.10.3 (rumoca main)",
)


def _model(path: Path, library: str = "FastModelMPC", include_container: bool = True) -> str:
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        network = convert_network(path.stem, path, library=Library.from_configuration(library))
        return network.model(include_container=include_container)


@cache
def _export(path: Path) -> CasadiModel:
    return CasadiModel.from_modelica(_model(path), f"{path.stem}.building_mpc")


def test_fast_model_is_templates_and_modelica() -> None:
    assert not (ROOT / "trano" / "fast_model").exists()
    templates = ROOT / "trano" / "elements" / "library" / "models" / "fast_model"
    assert {path.stem for path in templates.glob("*.yaml")} >= {"space", "envelope", "hydronics", "ventilation"}
    assert (ROOT / "trano" / "library" / "FastModel.mo").read_text().startswith("package FastModel")


def _fast_model_classes() -> dict[str, tuple[bool, str, str]]:
    """Models and connectors of FastModel.mo by name: whether they are partial, model or connector, and their body."""
    text = (ROOT / "trano" / "library" / "FastModel.mo").read_text()
    classes = {}
    for match in re.finditer(r"^( *)(partial )?(model|connector) (\w+)\b(?! =)", text, re.MULTILINE):
        indent, partial, kind, name = match.groups()
        end = re.compile(rf"^{indent}end {name};", re.MULTILINE).search(text, match.end())
        assert end, name
        classes[name] = (bool(partial), kind, text[match.end() : end.start()])
    return classes


def _has_icon(name: str, classes: dict[str, tuple[bool, str, str]]) -> bool:
    body = classes[name][2]
    bases = [base.split(".")[-1] for base in re.findall(r"^\s*extends ([\w.]+)", body, re.MULTILINE)]
    return "Icon(" in body or any(_has_icon(base, classes) for base in bases if base in classes)


def test_every_component_has_the_icon_of_its_buildings_model() -> None:
    classes = _fast_model_classes()
    missing = [name for name, (partial, _, _) in classes.items() if not partial and not _has_icon(name, classes)]
    assert not missing


def test_every_port_is_placed_on_the_icon() -> None:
    classes = _fast_model_classes()
    connectors = {"RealInput", "RealOutput"} | {name for name, (_, kind, _) in classes.items() if kind == "connector"}
    unplaced = [
        f"{name}.{component}"
        for name, (_, kind, body) in classes.items()
        if kind == "model"
        for declaration in re.findall(r"^\s*((?:Interfaces\.|Ventilation\.)?\w+ \w+[^;]*);", body, re.MULTILINE)
        if declaration.split()[0].split(".")[-1] in connectors and "Placement" not in declaration
        for component in [declaration.split()[1].split("(")[0].split("[")[0]]
    ]
    assert not unplaced


def test_the_components_are_grouped_in_containers() -> None:
    """The containers and their assembly of the other libraries (containers.yaml, main_building.jinja2)."""
    model = _model(MODELS / "single_zone_hydronic.yaml", "FastModel")
    assembly = model[model.index("model building\n") : model.index("end building;")]
    compact = assembly.replace(" ", "").replace("\n", "")
    assert "extends Components.Containers.building;" in model[model.index("model building_mpc") :]
    for container in ["envelope", "emission", "distribution", "production", "bus"]:
        assert f"    model {container}\n" in model
        assert f"Components.Containers.{container} {container}1" in assembly
    # the inputs of the model at the top of the assembly, connected to the data bus
    assert "FastModel.Interfaces.WeatherInputs weather_inputs" in assembly
    assert "FastModel.Interfaces.GainInputs occupancy_1_gai" in assembly
    assert "connect(weather_inputs.TDryBul, bus1.dataBus.TDryBulWeather);" in assembly
    # array ports between the containers, the containers on one data bus
    assert "connect(emission1.heatPortCon[1],envelope1.heatPortCon[1])" in compact
    assert "connect(emission1.dataBus,envelope1.dataBus)" in compact
    # the expandable data bus with the signals of the network declared; a control reads its zone on it
    assert "expandable connector DataBus" in model
    assert "Real TZonSpace_001;" in model
    assert "connect(dataBus.TZonSpace_001, TZon);" in model


@pytest.mark.parametrize("library", ["FastModel", "FastModelMPC"])
def test_the_containers_can_be_left_out(library: str) -> None:
    flat = _model(MODELS / "single_zone_hydronic.yaml", library, include_container=False)
    assert 'package Containers "Containers of building_mpc"' not in flat
    assert "FastModel.Zones.Zone space_001(" in flat[flat.index("model building_mpc") :]


@needs_connected_export
@pytest.mark.parametrize("library", ["FastModel", "FastModelMPC"])
@pytest.mark.parametrize(
    "path",
    [MODELS / "single_zone_hydronic.yaml", MODELS / "house_infiltration_air_water_heat_pump.yaml"],
    ids=lambda path: path.stem,
)
def test_the_containers_leave_the_model_unchanged(path: Path, library: str) -> None:
    """Same states, inputs and derivatives with and without containers."""
    name = f"{path.stem}.building_mpc"
    flat = CasadiModel.from_modelica(_model(path, library, include_container=False), name).description
    grouped_model = CasadiModel.from_modelica(_model(path, library), name)
    grouped = grouped_model.description
    states = [re.sub(r"^[a-z]+1\.", "", state) for state in grouped.state_names]
    assert sorted(states) == sorted(flat.state_names)
    assert grouped.input_names == flat.input_names
    order = [states.index(state) for state in flat.state_names]
    np.testing.assert_allclose(grouped.initial_state[order], flat.initial_state)
    flat_model = CasadiModel.from_modelica(_model(path, library, include_container=False), name)
    inputs = np.array([KELVIN + 5 if "TDryBul" in n or "TBlaSky" in n else 1.0 for n in flat.input_names])
    np.testing.assert_allclose(
        np.asarray(grouped_model.derivatives(0.0, grouped.initial_state, inputs, grouped.parameter_values)).ravel()[
            order
        ],
        np.asarray(flat_model.derivatives(0.0, flat.initial_state, inputs, flat.parameter_values)).ravel(),
        rtol=1e-9,
        atol=1e-12,
    )


def test_the_package_holds_the_model_and_its_runnable_wrapper() -> None:
    model = _model(MODELS / "single_zone_hydronic.yaml")
    assert "package FastModel" in model
    assert "model building_mpc" in model
    assert "building_mpc mpc(envelope1(weather(lat=weather_reader.lat" in model
    assert "Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weather_reader" in model
    assert "Buildings." not in _building_mpc(model)  # the physics only: no table, no event, no Buildings component


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
    assert roles.comfort == ("envelope1.space_001.TAir",)
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


@needs_connected_export
def test_the_controllers_expose_their_smoothing_as_parameters() -> None:
    """The width of every smooth transition and the switching time of every hysteresis are parameters of the
    closed-loop model (a calibration widens them), at the values of the switching controllers by default."""
    path = MODELS / "house_infiltration_air_water_heat_pump.yaml"
    model = CasadiModel.from_modelica(_model(path, "FastModel"), f"{path.stem}.building_mpc")
    values = dict(zip(model.description.parameter_names, model.description.parameter_values, strict=True))
    smoothing = {name for name in values if name.endswith((".dU", ".dY", ".dMax", ".dT", ".dtSet", ".tau"))}

    def value(suffix: str) -> float:
        (name,) = [name for name in smoothing if name.endswith(suffix)]
        return float(values[name])

    assert len(smoothing) == 27
    assert value("boiler_001_control.bufCha.tau") == 10.0
    assert value("boiler_001_control.bufCha.dU") == pytest.approx(0.5)  # dTHys = 5 K: (uHigh - uLow) / 10
    assert value("boiler_001_control.dT") == pytest.approx(0.1)
    assert value("boiler_control_001.hys.dU") == pytest.approx(0.049)  # demand: uLow 0.01, uHigh 0.5
    assert value("control_1.dMax") == pytest.approx(1e-3)
    assert all(values[name] == pytest.approx(60.0) for name in smoothing if name.endswith(".dtSet"))
    assert all(values[name] == pytest.approx(1e-3) for name in smoothing if name.endswith("emission_control_001.dY"))


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
    # the MPC model keeps one node per layer boundary; the closed-loop model discretises the layers
    assert mpc.templates.model_copy(update={"construction": None}) == closed_loop.templates.model_copy(
        update={"construction": None}
    )
    assert mpc.templates.construction != closed_loop.templates.construction
    assert mpc.package_template == closed_loop.package_template
    assert mpc.parameter_library == closed_loop.parameter_library


def test_the_closed_loop_model_discretises_the_layers_as_buildings() -> None:
    """BESTEST heavy wall: wood siding, foam and 0.1 m of concrete blocks with 18 states per 0.2 m of reference
    concrete (number_of_states). The closed-loop model splits each layer into the states Buildings gives it, at most
    three; the MPC model keeps one node per layer boundary. The steady-state resistance and the capacity of the wall
    are the same."""
    records = {}
    for library in ("FastModel", "FastModelMPC"):
        model = _model(BESTEST / "case_940.yaml", library)
        start = model.index("record heavy_wall_001")
        records[library] = model[start : model.index(";", start)]
    for record in records.values():  # the layers of the YAML: FastModel.Data.Construction splits them
        assert "nLay=3" in record
        assert "x={0.009, 0.0615, 0.1}" in record
    assert "nSta={2, 3, 3}" in records["FastModel"]
    assert "nNod=8" in records["FastModel"]
    assert "lay={1, 1, 2, 2, 2, 3, 3, 3}" in records["FastModel"]
    assert "nod={1, 3, 6, 9}" in records["FastModel"]  # the layer boundaries: the states T of both models
    assert "sub={2, 4, 5, 7, 8}" in records["FastModel"]
    assert "nSta={1, 1, 1}" in records["FastModelMPC"]
    assert "lay={1, 2, 3}" in records["FastModelMPC"]
    assert "nod={1, 2, 3, 4}" in records["FastModelMPC"]
    assert "sub=fill(0, 0)" in records["FastModelMPC"]


WALL_ROOM = """
package room
  package Data
    record Wall = FastModel.Data.Construction(nLay=2, x={0.02, 0.15}, k={0.04, 1.4}, c={1400, 900}, d={10, 2300},
      nSta={2, 3}, nNod=5, lay={1, 1, 2, 2, 2}, nod={1, 3, 6}, sub={2, 4, 5});
  end Data;
  model building_mpc
    FastModel.Interfaces.WeatherInputs weather_inputs;
    FastModel.BoundaryConditions.Weather weather(TDryBul=weather_inputs.TDryBul, TBlaSky=weather_inputs.TBlaSky,
      winSpe=weather_inputs.winSpe, winDir=weather_inputs.winDir, HDirNor=weather_inputs.HDirNor,
      HDifHor=weather_inputs.HDifHor, HGloHor=weather_inputs.HGloHor);
    FastModel.Interfaces.OccupancyInputs occupancy;
    FastModel.Zones.Zone zone(nSurf=1, AFlo=20, hRoo=2.5);
    FastModel.Envelope.AdiabaticWall wall(constructionType=Data.Wall(), A=10, til=FastModel.Types.Tilt.Wall);
  equation
    connect(weather.weaBus, zone.weaBus);
    connect(occupancy.gai, zone.qGai_flow);
    connect(wall.propsBus_a, zone.propsBus[1]);
  end building_mpc;
end room;
"""


@needs_connected_export
def test_the_construction_splits_its_layers_into_sublayers() -> None:
    """rumoca exports a wall whose two layers are split into 2 and 3 sublayers: 6 nodes, the three layer boundaries
    T (the states of FastModelMPC) and three nodes inside the layers TSub, the resistance and the capacity of the
    layers."""
    from trano.elements.jinja import modelica_library

    model = CasadiModel.from_modelica(f"{modelica_library('FastModel')}\n{WALL_ROOM}", "room.building_mpc")
    description = model.description
    assert {name for name in description.state_names if name.startswith("wall.T")} == {
        "wall.T[1]",
        "wall.T[2]",
        "wall.T[3]",
        "wall.TSub[1]",
        "wall.TSub[2]",
        "wall.TSub[3]",
    }
    parameters = dict(zip(description.parameter_names, description.parameter_values, strict=True))
    resistance = sum(1 / parameters[f"wall.G[{i}]"] for i in range(1, 6))
    assert resistance == pytest.approx((0.02 / 0.04 + 0.15 / 1.4) / 10)
    capacity = sum(parameters[f"wall.C[{i}]"] for i in range(1, 7))
    assert capacity == pytest.approx(10 * (0.02 * 10 * 1400 + 0.15 * 2300 * 900))
    assert parameters["wall.C[3]"] == pytest.approx(10 * (0.01 * 10 * 1400 + 0.05 * 2300 * 900) / 2)


def _building_mpc(model: str) -> str:
    """building_mpc and its containers."""
    start = model.index("package Components") if "package Components" in model else model.index("model building_mpc")
    return model[start : model.index("end building_mpc;")]


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
    assert "extends FastModel.Controls.EmissionMonitored;" in model
    assert "BaseClasses.EmissionMonitoredEmission_control_001 emission_control_001(" in model
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
    zone = model.description.state_names.index("envelope1.space_001.TAir")
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
    record Glass = FastModel.Data.Glazing(nPan=1, x={0.004}, k={1.0}, d={2500}, c={840}, xGas=fill(0, 0),
      kGas=fill(0, 0), muGas_a=fill(0, 0), muGas_b=fill(0, 0), cGas=fill(0, 0), MGas=fill(0, 0), epsGap_a=fill(0, 0),
      epsGap_b=fill(0, 0), tau={0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.7, 0.5, 0.2, 0, 0.7},
      absPan={{0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0, 0.1}}, absIR_a=0.84, absIR_b=0.84, UFra=2.0);
    record Slab = FastModel.Data.Construction(nLay=1, x={0.2}, k={1.4}, c={900}, d={2300}, nSta={1}, nNod=1,
      lay={1}, nod={1, 2}, sub=fill(0, 0));
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
    model = _building_mpc(_model(MODELS / "house_complex_heat_meter.yaml", library, include_container=False))
    model = model.replace(" ", "")
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
    unit = re.search(r"BaseClasses\.HeatRecoveryUnitAhu_001ahu_001\((.*?)\)", model, re.DOTALL)
    assert unit is not None
    assert re.search(r"dpSup_nominal=100(,|$)", unit.group(1)) and re.search(r"dpRet_nominal=100(,|$)", unit.group(1))


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


@pytest.mark.simulate
def test_simulate_fast_model() -> None:
    """The runnable model of the containers simulates in OpenModelica, its zones at plausible temperatures."""
    from buildingspy.io.outputfile import Reader

    from trano.utils.utils import is_success

    path = MODELS / "house_infiltration_air_water_heat_pump.yaml"
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        network = convert_network(path.stem, path, library=Library.from_configuration("FastModel"))
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as project_path:
        results = simulate(Path(project_path), network, options=SimulationOptions(end_time=4 * 3600))
        assert is_success(results), results.output.decode(errors="replace")[-3000:]
        result = Reader(str(next(Path(project_path).joinpath("results").glob("*.mat"))), "dymola")
        zones = result.varNames(r"^mpc\.envelope1\.\w+\.TAir$")
        assert len(zones) == 6
        for zone in zones:
            _, temperature = result.values(zone)
            assert temperature.min() > KELVIN + 10, zone
            assert temperature.max() < KELVIN + 40, zone


def test_the_zone_takes_the_night_ventilation_schedule() -> None:
    """BESTEST 950FF: the scheduled outdoor air of the zone (Buildings' ventilationSchedule) reaches the FastModel zone;
    a zone without a schedule keeps the default of the library."""
    model = _model(BESTEST / "case_950FF.yaml", "FastModel", include_container=False)
    assert (
        "ventilationSchedule=[0, 0.391389; 25200, 0.391389; 25200, 0; 64800, 0; 64800, 0.391389; 86400, 0.391389]"
        in model
    )
    assert "ventilationSchedule=" not in _model(BESTEST / "case_600FF.yaml", "FastModel", include_container=False)


def test_a_saturated_ideal_system_gives_no_heat() -> None:
    """smoothLimit reaches its bounds exactly: a saturated PI of the ideal system gives no heat. When it saturated at
    dY and 1 - dY, heating and cooling each leaked 1e-3 of their capacity (1 kW for BESTEST)."""
    text = (ROOT / "trano" / "library" / "FastModel.mo").read_text()
    ideal = text[text.index("model IdealHeatingCoolingPI") : text.index("end IdealHeatingCoolingPI;")]
    assert "QHea_flow = conHea.y*QHea_flow_max;" in ideal
    assert "QCoo_flow = conCoo.y*QCoo_flow_max;" in ideal


@pytest.mark.parametrize(
    ("x", "expected"),
    [(-1.0, 0.0), (-0.01, 0.0), (0.0, 0.0025), (0.005, 0.005625), (0.02, 0.02), (0.5, 0.5), (1.0, 0.9975), (2.0, 1.0)],
)
def test_the_smooth_limit_reaches_its_bounds_and_stays_inside(x: float, expected: float) -> None:
    """The smooth limit of the controllers, evaluated by rumoca: exactly 0 and 1 beyond the blends of half width
    0.01, never outside [0, 1], continuous with its derivative at the ends of the blends."""
    if CasadiModel is None:
        pytest.skip("trano-optimization (rumoca) is not installed")
    from trano.elements.jinja import modelica_library

    probe = """
model Limit
  input Real x;
  Real s(start=0, fixed=true);
equation
  der(s) = FastModel.Utilities.smoothLimit(x, 0, 1, 0.01);
end Limit;
"""
    model = CasadiModel.from_modelica(f"{modelica_library('FastModel')}\n{probe}", "Limit")
    assert model.derivatives(0.0, np.zeros(1), np.array([x]))[0] == pytest.approx(expected, abs=1e-12)


GAP_PROBE = """
model Gap
  input Real T_a;
  input Real T_b;
  input Real eps;
  Real s(start=0, fixed=true);
equation
  der(s) = FastModel.Envelope.Functions.gapConductance(T_a, T_b, 0.012, 0.025, 3.723e-6, 4.94e-8, 1006, 0.02897, eps,
    0.84, 2.0);
end Gap;
"""


def test_the_gas_gap_conducts_with_the_temperatures_of_its_panes() -> None:
    """The conductance of a 12 mm air gap, evaluated by rumoca: the EN 673 value at the EN 673 state (mean 10 degC,
    15 K across), a fifth more between panes at 40 and 30 degC (the long-wave exchange grows as T^3), most of the
    radiation gone with a low-e face."""
    if CasadiModel is None:
        pytest.skip("trano-optimization (rumoca) is not installed")
    from types import SimpleNamespace

    from trano.elements.glazing import gap_conductance
    from trano.elements.jinja import modelica_library

    model = CasadiModel.from_modelica(f"{modelica_library('FastModel')}\n{GAP_PROBE}", "Gap")

    def conductance(t_a: float, t_b: float, eps: float = 0.84) -> float:
        return float(model.derivatives(0.0, np.zeros(1), np.array([t_a, t_b, eps]))[0])

    air = SimpleNamespace(
        thermal_conductivity=0.025, density=1.2, specific_heat_capacity=1006, viscosity_coefficients=(3.723e-6, 4.94e-8)
    )
    reference = conductance(290.5, 275.5)
    assert reference == pytest.approx(gap_conductance(0.012, air, 0.84, 0.84, 15.0), rel=0.02)
    assert conductance(313.15, 303.15) == pytest.approx(1.18 * reference, rel=0.02)
    assert conductance(278.15, 268.15) < reference
    assert conductance(290.5, 275.5, eps=0.04) < 0.45 * reference
    assert conductance(293.15, 293.15) == pytest.approx(conductance(293.16, 293.14), rel=1e-4)  # smooth at dT = 0


@needs_connected_export
def test_the_closed_loop_and_mpc_models_share_the_layer_boundaries() -> None:
    """The closed-loop model splits the layers, the MPC model does not: both name T the temperatures of the layer
    boundaries, so that a state copied from one model into the other by its name (mpctrano's spin-up) stays at its
    place; the nodes inside the layers are TSub, in the closed-loop model only."""
    path = MODELS / "single_zone_hydronic.yaml"
    mpc = _export(path).description.state_names
    closed_loop_model = CasadiModel.from_modelica(_model(path, "FastModel"), f"{path.stem}.building_mpc")
    closed_loop = closed_loop_model.description.state_names
    boundaries = {name for name in mpc if ".T[" in name}
    assert boundaries
    assert boundaries <= set(closed_loop)
    assert any(".TSub[" in name for name in closed_loop)
    assert not any(".TSub[" in name for name in mpc)
