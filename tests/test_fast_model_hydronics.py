"""Hydronic circuits of the ``FastModel`` library (``--envelope multilayer`` with radiators, valves and production).

The agreement with Buildings is checked by ``python -m validation.mpc`` (replay of the branch flows, valve
positions and supply temperatures); these tests check the radiator sizing, the mapping of the YAML graph onto
components and the CasADi export (rumoca >= 0.10.3).
"""

import warnings
from pathlib import Path

import numpy as np
import pytest
import rumoca

from trano.data_models.conversion import convert_network
from trano.elements.library.library import Library
from trano.fast_model.components.envelope import multilayer_model
from trano.fast_model.components.hydronics import (
    HydronicSettings,
    hydronic_circuit,
    hydronic_graph,
    radiator_conductance,
)
from trano.fast_model.modelica import network_interface
from trano.topology import Network

MODELS = Path(__file__).parent / "models"


def _exports_connected_models() -> bool:
    from tests.test_fast_model_connected import _exports_connected_models as probe

    return probe()


needs_connected_export = pytest.mark.skipif(
    not _exports_connected_models(), reason="the CasADi export of connected models needs rumoca >= 0.10.3"
)


def _network(name: str) -> Network:
    path = MODELS / f"{name}.yaml"
    library = Library.from_configuration("FastModel")
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        return convert_network(path.stem, path, library=library)


def _ports(zone: str) -> tuple[str, str]:
    return f"{zone}_air.port", f"{zone}_rad.rad"


# ----------------------------------------------------------------------------------------- radiators
def test_radiator_conductance_is_the_one_buildings_computes() -> None:
    """UAEle of RadiatorEN442_2 (3900 W at 75/65 degC, fraRad 0.3), read from a Buildings simulation."""
    conductance = radiator_conductance(3900, 348.15, 338.15, radiant_fraction=0.3)
    assert conductance == pytest.approx(34.760105823691056, rel=1e-8)


@pytest.mark.parametrize("elements", [1, 3, 5])
def test_radiator_conductance_gives_the_nominal_power(elements: int) -> None:
    power, t_in, t_out, cp = 2500.0, 353.15, 333.15, 4184.0
    conductance = radiator_conductance(power, t_in, t_out, radiant_fraction=0.35, elements=elements)
    flow, t_upstream, total = power / cp / (t_in - t_out), t_in, 0.0
    for _ in range(elements):  # the nominal state of the elements, by fixed point
        t_water = t_upstream
        for _ in range(200):
            q = conductance * (t_water - 293.15) ** 1.24
            t_water = t_upstream - q / flow / cp
        total, t_upstream = total + q, t_water
    assert total == pytest.approx(power, rel=1e-6)
    assert t_upstream == pytest.approx(t_out, abs=1e-4)


# ------------------------------------------------------------------------------------------ circuits
@pytest.mark.parametrize("name", ["single_zone_hydronic", "three_zones_hydronic", "house_complex"])
def test_circuits_have_one_reference_and_set_flows(name: str) -> None:
    import networkx as nx

    network = _network(name)
    graph = hydronic_graph(network)
    circuit = hydronic_circuit(network, _ports)
    components = [instance.component for instance in circuit.instances]
    assert components.count("Hydronics.Reference") == nx.number_weakly_connected_components(graph)
    flows = [top for top in circuit.inputs if top.unit == "kg/s"]
    assert flows
    assert all(top.control and top.zone for top in flows)
    assert {top.zone for top in flows} == circuit.heated_zones


def test_the_mixing_valve_takes_the_supply_on_port_1() -> None:
    network = _network("single_zone_hydronic")
    connections = dict(hydronic_circuit(network, _ports).connections)
    assert connections["pump_001.outlet"] == "three_way_valve_001.port_1"
    assert any(target == "three_way_valve_001.port_3" for target in connections.values())


def test_diverting_valves_send_their_bypass_to_the_return() -> None:
    network = _network("three_zones_hydronic")
    circuit = hydronic_circuit(network, _ports)
    diverting = [i.name for i in circuit.instances if i.component == "Hydronics.DivertingValve"]
    assert sorted(diverting) == ["three_way_valve_001", "three_way_valve_002"]
    connections = dict(circuit.connections)
    assert connections["three_way_valve_002.port_2"] == "temperature_sensor_002.inlet"
    assert connections["three_way_valve_002.port_3"].startswith("split_valve_002")


def test_hydronic_zones_lose_their_heat_input() -> None:
    model = multilayer_model(_network("single_zone_hydronic"))
    assert "space_001_QHea" not in model.input_names
    assert {"valve_003_m_flow", "three_way_valve_001_y", "boiler_001_y"} <= set(model.input_names)
    connections = set(model.connections)
    assert ("radiator_003.heatPortRad", "space_001_rad.rad") in connections


def test_the_opening_can_be_the_valve_input() -> None:
    model = multilayer_model(_network("single_zone_hydronic"), hydronics=HydronicSettings(valve_input="opening"))
    valve = next(instance for instance in model.instances if instance.name == "valve_003")
    assert valve.component == "Hydronics.Valve"
    assert valve.parameters["a"] == pytest.approx(10000 / 15000)
    assert "valve_003_y" in model.input_names


# -------------------------------------------------------------------------------------------- export
@needs_connected_export
@pytest.mark.parametrize("name", ["single_zone_hydronic", "three_zones_hydronic"])
def test_the_hydronic_model_exports_and_heats(name: str) -> None:
    network = _network(name)
    interface = network_interface(network)
    export = rumoca.Session().loads(network.model(), model=interface.model).to_casadi()
    assert interface.state_names == list(export.state_names)
    assert interface.input_names == list(export.input_names)
    assert interface.parameter_names == list(export.parameter_names)

    values = {"TOut": 273.15, "TBlaSky": 263.15, "winSpe": 2.0, "winDir": 0.0}
    for control in interface.controls:
        values[control.name] = {"kg/s": 0.05, "1": 0.8, "K": 343.15}[control.unit]
    u = [values.get(name, 0.5 if name.startswith("cosInc_") else 0.0) for name in interface.input_names]
    x = np.array(export.default_states).ravel()
    xdot = np.array(export.rhs(0, x, u, export.default_parameters)).ravel()
    assert np.all(np.isfinite(xdot))
    radiators = [i for i, state in enumerate(interface.states) if state.name.startswith("radiator_")]
    assert radiators
    assert np.all(xdot[radiators] > 0)  # hot water flows into the radiators
    for zone in interface.zones:
        assert zone.heating_input in interface.input_names


@pytest.mark.compile
def test_openmodelica_compiles_the_hydronic_model(tmp_path: Path) -> None:
    """The runnable model, with the hydronic controls as top-level inputs, checks in OpenModelica (Docker)."""
    from trano.simulate.simulate import SimulationOptions, simulate
    from trano.utils.utils import is_success

    options = SimulationOptions(check_only=True)
    results = simulate(tmp_path, _network("three_zones_hydronic"), options=options, container_name="compile_hydronic")
    assert is_success(results, options=options), results.output.decode(errors="replace")[-3000:]


# ---------------------------------------------------------------------------------------- heat pumps
def test_air_water_heat_pumps_become_heat_pump_components() -> None:
    model = multilayer_model(_network("house_infiltration_air_water_heat_pump"))
    heat_pump = next(instance for instance in model.instances if instance.component == "Hydronics.HeatPump")
    assert heat_pump.inputs == {"y": "boiler_001_y", "TEva": "TOut"}
    assert heat_pump.parameters["UA"] > 0  # the buffer tank loses heat to its room
    assert "boiler_001.EEle" in model.state_names  # electricity meter


@needs_connected_export
@pytest.mark.parametrize("source", [258.15, 266.15, 270.15, 280.15, 288.15])
@pytest.mark.parametrize("condenser", [308.15, 318.15, 323.15, 328.15])
def test_the_heat_pump_follows_the_buildings_tables(source: float, condenser: float) -> None:
    """Heating capacity and power at full speed against the interpolated Vitocal tables, scaled as Buildings."""
    from trano.fast_model.components import ConnectedModel, Instance, TopInput, package_source
    from trano.fast_model.components.heat_pump import VITOCAL_HEATING, VITOCAL_POWER, bilinear, heat_pump_instance

    heat_pump = next(
        node
        for node in _network("house_infiltration_air_water_heat_pump").graph.nodes
        if getattr(node, "variant", None) == "air_water_heat_pump"
    )
    instance = heat_pump_instance(heat_pump, "hp", "y")
    model = ConnectedModel(
        name="M",
        inputs=[TopInput(name="y"), TopInput(name="TOut", unit="K")],
        instances=[
            instance,
            Instance(name="val", component="Hydronics.FlowValve", inputs={"m_flow": "0"}),
            Instance(name="ref", component="Hydronics.Reference"),
        ],
        connections=[("hp.outlet", "val.inlet"), ("val.outlet", "ref.inlet"), ("ref.outlet", "hp.inlet")],
    )
    library = package_source()
    export = rumoca.Session().loads(f"package T\n{library}\n{model.render()}\nend T;\n", model="T.M").to_casadi()
    p = dict(model.parameters)
    buffer = condenser - p["hp.dTCon"]  # condenser outlet at ``condenser`` at full speed
    xdot = np.array(export.rhs(0, [buffer, 0.0], [1.0, source], export.default_parameters)).ravel()
    heat = p["hp.C"] * xdot[0] - p["hp.UA"] * (p["hp.TAmb"] - buffer)
    scale = p["hp.scaFac"]
    assert heat == pytest.approx(scale * bilinear(VITOCAL_HEATING, condenser, source), rel=0.03)
    assert xdot[1] * 3.6e6 == pytest.approx(scale * bilinear(VITOCAL_POWER, condenser, source), rel=0.05)
