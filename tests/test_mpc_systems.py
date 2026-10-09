"""The energy systems of the ``mpc`` library: YAML mapping, Modelica rendering, interface, CasADi export."""

from pathlib import Path

import numpy as np
import pytest
import rumoca
from pydantic import ValidationError

from trano.data_models.conversion import convert_network
from trano.elements.library.library import Library
from trano.mpc import (
    Battery,
    Chiller,
    DHWTank,
    EVCharger,
    EVSessions,
    GasBoiler,
    HeatPump,
    HeatPumpSpec,
    MPCModelInterface,
    Orientation,
    Photovoltaic,
    RCBuilding,
    RCModelType,
    RCZone,
    SolarAperture,
    StorageTankSpec,
    network_interface,
    rc_building_from_network,
)
from trano.mpc.estimation import reference_zone_parameters
from trano.mpc.systems import DrawOffProfile

MODELS = Path(__file__).parent / "models"
HOUSE = MODELS / "house_mpc_systems.yaml"
SOUTH = Orientation(azimuth=0, tilt=90)


def mpc_network(model_type: RCModelType = RCModelType.r4c3, path: Path = HOUSE):  # noqa: ANN201
    library = Library.from_configuration("mpc").model_copy(update={"rc_model_type": model_type})
    return convert_network(path.stem, path, library=library)


def zone(name: str, model_type: RCModelType = RCModelType.r3c2) -> RCZone:
    return RCZone(
        name=name,
        parameters=reference_zone_parameters(model_type),
        solar_apertures=[SolarAperture(orientation=SOUTH, window=3.0)],
        design_heating_power=5000.0,
    )


def to_casadi(source: str, model: str):  # noqa: ANN201
    compiled = rumoca.Session().loads(source, model=model)
    assert "0 algebraic" in compiled.summary(), compiled.summary()
    return compiled.to_casadi()


# --------------------------------------------------------------------------- system models
def test_heat_pump_cop_polynomial_matches_the_modelica_expression() -> None:
    heat_pump = HeatPump(name="hp", zones=["z"], cop_nominal=4.5)
    assert heat_pump.cop(280.15, 308.15) == pytest.approx(4.5)
    assert heat_pump.cop(266.15, 308.15) == pytest.approx(4.5 - 14 * 0.11)
    assert heat_pump.cop(280.15, 328.15) == pytest.approx(4.5 - 20 * 0.075)
    expression = heat_pump.cop_expression("TOut", "hp_TSup")
    assert expression.startswith("(hp_cop0 + hp_copA*(TOut - 280.15)")
    assert "hp_copX*(TOut - 280.15)*(hp_TSup - 308.15)" in expression


def test_draw_off_profile_is_normalised_and_sized() -> None:
    profile = DrawOffProfile(daily_energy_kwh=6.0, hourly_fractions=tuple([1.0] * 24))
    assert sum(profile.hourly_fractions) == pytest.approx(1.0)
    assert sum(profile.hourly_power()) == pytest.approx(6000.0)
    with pytest.raises(ValidationError):
        DrawOffProfile(hourly_fractions=(1.0, 2.0))


def test_ev_sessions_driving_profile() -> None:
    sessions = EVSessions(arrival_hour=18, departure_hour=7, energy_per_day_kwh=11.0)
    assert sessions.hours_away == 11
    power = sessions.hourly_driving_power()
    assert power[8] == pytest.approx(1000.0)
    assert power[20] == 0.0
    assert sum(power) == pytest.approx(11000.0)


def test_building_validates_system_references() -> None:
    with pytest.raises(ValidationError, match="unknown zones"):
        RCBuilding(zones=[zone("a")], systems=[HeatPump(name="hp", zones=["b"])])
    with pytest.raises(ValidationError, match="heated by both"):
        RCBuilding(zones=[zone("a")], systems=[HeatPump(name="hp", zones=["a"]), GasBoiler(name="boi", zones=["a"])])
    with pytest.raises(ValidationError, match="unknown heat pump"):
        RCBuilding(zones=[zone("a")], systems=[DHWTank(name="tank", zone="a", heat_pump="hp")])
    with pytest.raises(ValidationError, match="name of a zone"):
        RCBuilding(zones=[zone("a")], systems=[Battery(name="a")])


# --------------------------------------------------------------------------- Modelica rendering
def _house(model_type: RCModelType = RCModelType.r4c3) -> RCBuilding:
    return RCBuilding(
        zones=[zone("day", model_type), zone("night", model_type)],
        systems=[
            HeatPump(name="hp", zones=["day", "night"], max_electrical_power=2500.0),
            Chiller(name="ch", zones=["day"]),
            DHWTank(name="tank", zone="night", heat_pump="hp"),
            Battery(name="bat"),
            EVCharger(name="ev"),
            Photovoltaic(name="pv", area=30.0),
        ],
    )


@pytest.mark.parametrize("model_type", list(RCModelType))
def test_systems_are_inlined_and_translate_to_casadi(model_type: RCModelType) -> None:
    building = _house(model_type)
    source = building.to_modelica("House")
    export = to_casadi(source, "House.building_mpc")
    assert export.state_names == building.state_names
    assert building.state_names[-3:] == ["tank_T", "bat_E", "ev_E"]
    inputs = list(export.input_names)
    assert "day_PHea" in inputs and "night_PHea" in inputs and "day_PCoo" in inputs
    assert "day_QHea" not in inputs
    assert inputs[-7:] == ["tank_PHea", "tank_QDraw", "bat_PCha", "bat_PDis", "ev_PCha", "ev_PDri", "pv_P"]
    model = source[source.index("model building_mpc") : source.index("end building_mpc;")]
    assert "Modelica." not in model and "connect(" not in model
    assert "der(tank_T) = (tank_PHea*(hp_cop0" in model
    assert "der(bat_E) = bat_etaCha*bat_PCha - bat_PDis/bat_etaDis;" in model
    assert "der(ev_E) = ev_etaCha*ev_PCha - ev_PDri;" in model
    assert "tank_UA*(tank_T - night_Ti)" in model
    assert "- day_PCoo*(ch_eer0 + ch_eerA*(TOut - 308.15))" in model
    if model_type == RCModelType.r4c3:
        assert "day_PHea*(hp_cop0 + hp_copA*(TOut - 280.15) + hp_copS*((day_Th + hp_dTSup) - 308.15)" in model
    else:
        assert "day_PHea*(hp_cop0 + hp_copA*(TOut - 280.15) + hp_copS*(hp_TSup - 308.15)" in model


def test_heat_pump_delivers_cop_times_electrical_power() -> None:
    """One step of the exported ODE: the zone receives COP * P, the tank what the draw-off does not take."""
    building = _house(RCModelType.r3c2)
    export = to_casadi(building.to_modelica("House"), "House.building_mpc")
    names = list(export.input_names)
    u = np.zeros(len(names))
    u[names.index("TOut")] = 280.15
    u[names.index("day_PHea")] = 1000.0
    u[names.index("tank_PHea")] = 500.0
    u[names.index("tank_QDraw")] = 300.0
    x = np.array(export.default_states, dtype=float)
    states = list(export.state_names)
    x[states.index("tank_T")] = x[states.index("night_Ti")]  # no standing losses
    xdot = np.array(export.rhs(0, x, u, export.default_parameters), dtype=float).reshape(-1)
    heat_pump = building.systems[0]
    assert isinstance(heat_pump, HeatPump)
    day = building.zones[0].parameters
    tank = building.systems[2]
    assert isinstance(tank, DHWTank)
    expected_zone = 1000.0 * heat_pump.cop(280.15, heat_pump.supply_temperature)
    # Envelope and outdoor exchange at the default (equal) temperatures cancel except the outdoor term.
    air_terms = (280.15 - x[states.index("day_Ti")]) / day.indoor_outdoor_resistance  # type: ignore[union-attr]
    assert xdot[states.index("day_Ti")] * day.air_capacitance == pytest.approx(expected_zone + air_terms, rel=1e-6)  # type: ignore[union-attr]
    expected_tank = 500.0 * heat_pump.cop(280.15, x[states.index("tank_T")] + tank.supply_offset) - 300.0
    assert xdot[states.index("tank_T")] * tank.capacitance == pytest.approx(expected_tank, rel=1e-6)


def test_interface_describes_the_systems() -> None:
    building = _house()
    interface = building.interface("House")
    assert interface.version == "2"
    assert [system.kind for system in interface.systems] == [
        "heat_pump", "chiller", "dhw_tank", "battery", "ev_charger", "photovoltaic",
    ]  # fmt: skip
    heat_pump = interface.system("hp")
    assert isinstance(heat_pump, HeatPumpSpec)
    assert heat_pump.inputs == ["day_PHea", "night_PHea"]
    assert heat_pump.tank_inputs == ["tank_PHea"]
    assert heat_pump.max_electrical_power == 2500.0
    tank = interface.system("tank")
    assert isinstance(tank, StorageTankSpec)
    assert tank.state == "tank_T" and tank.heat_pump == "hp"
    day = interface.zones[0]
    assert day.heating_input == "day_PHea" and day.heating_carrier == "electricity" and day.heating_system == "hp"
    assert day.cooling_input == "day_PCoo"
    assert interface.zones[1].cooling_input is None
    signals = {signal.name: signal for signal in interface.inputs}
    assert signals["pv_P"].source.kind == "photovoltaic"  # type: ignore[union-attr]
    assert signals["tank_QDraw"].source.kind == "draw_off"  # type: ignore[union-attr]
    assert signals["ev_PDri"].source.kind == "ev_driving"  # type: ignore[union-attr]
    assert [signal.name for signal in interface.electrical_controls] == [
        "day_PHea", "day_PCoo", "night_PHea", "tank_PHea", "bat_PCha", "bat_PDis", "ev_PCha",
    ]  # fmt: skip
    states = {state.name: state for state in interface.states}
    assert states["bat_E"].system == "bat" and states["bat_E"].unit == "J"
    round_trip = MPCModelInterface.model_validate_json(interface.model_dump_json())
    assert round_trip == interface


def test_version_1_interface_is_still_read() -> None:
    envelope_only = RCBuilding(zones=[zone("a")]).interface("House")
    payload = envelope_only.model_dump_json().replace('"version":"2"', '"version":"1"')
    interface = MPCModelInterface.model_validate_json(payload)
    assert interface.systems == []
    assert interface.zones[0].heating_input == "a_QHea"


# --------------------------------------------------------------------------- from the YAML
def test_systems_are_mapped_from_the_yaml() -> None:
    network = mpc_network()
    building = rc_building_from_network(network)
    systems = {system.name: system for system in building.systems}
    assert set(systems) == {"heatpump_001", "tank_001", "chiller_001", "battery_001", "ev_001", "pv_001"}
    heat_pump = systems["heatpump_001"]
    assert isinstance(heat_pump, HeatPump)
    assert heat_pump.zones == ["space_001", "space_002"]  # not connected: serves every zone
    assert heat_pump.cop_nominal == 4.6
    assert heat_pump.max_electrical_power == 2500.0
    assert heat_pump.supply_temperature == 308.15  # floor heating radiators
    assert heat_pump.supply_offset == pytest.approx(2.5)
    tank = systems["tank_001"]
    assert isinstance(tank, DHWTank)
    assert tank.zone == "space_002" and tank.heat_pump == "heatpump_001"
    assert tank.capacitance == pytest.approx(0.3 * 4.186e6)
    assert tank.draw_off.daily_energy_kwh == 6.0
    chiller = systems["chiller_001"]
    assert isinstance(chiller, Chiller)
    assert chiller.max_electrical_power == pytest.approx(5000 / 3.2)
    battery = systems["battery_001"]
    assert isinstance(battery, Battery)
    assert battery.capacity == pytest.approx(8 * 3.6e6)
    charger = systems["ev_001"]
    assert isinstance(charger, EVCharger)
    assert charger.sessions.energy_per_day_kwh == 12.0
    pv = systems["pv_001"]
    assert isinstance(pv, Photovoltaic)
    assert pv.peak_power == pytest.approx(30 * 0.19 * 1000)
    # The floor heating temperatures size the R4C3 emitter: 12.5 K above the air at nominal power.
    emitter = building.zones[0].parameters
    assert emitter.model_type == RCModelType.r4c3
    assert emitter.emitter_resistance * building.zones[0].design_heating_power == pytest.approx(12.5)  # type: ignore[union-attr]


def test_connected_heat_pump_serves_the_zones_downstream() -> None:
    network = mpc_network(RCModelType.r3c2, MODELS / "three_zones_hydronic_reduced_orders_heat_pump.yaml")
    building = rc_building_from_network(network)
    heat_pumps = [system for system in building.systems if isinstance(system, HeatPump)]
    assert len(heat_pumps) == 1
    assert heat_pumps[0].zones == ["space_001", "space_002", "space_003"]
    assert heat_pumps[0].supply_temperature == 353.15  # high temperature radiators of the YAML


def test_gas_boiler_keeps_the_thermal_inputs() -> None:
    network = mpc_network(RCModelType.r3c2, MODELS / "three_zones_hydronic_reduced_orders.yaml")
    interface = network_interface(network)
    boilers = [system for system in interface.systems if system.kind == "boiler"]
    assert len(boilers) == 1
    assert boilers[0].efficiency == pytest.approx(0.9)  # type: ignore[union-attr]
    assert interface.zones[0].heating_input == "space_001_QHea"
    assert interface.zones[0].heating_carrier == "gas"


def test_runnable_model_wires_the_systems() -> None:
    network = mpc_network()
    source = network.model()
    runnable = source[source.index("model building ") : source.index("end building;")]
    assert "Modelica.Blocks.Interfaces.RealInput space_001_PHea" in runnable
    assert "connect(space_001_PHea, rc.space_001_PHea);" in runnable
    assert "CombiTimeTable tank_001_drawOff" in runnable
    assert "connect(tank_001_drawOff.y[1], rc.tank_001_QDraw);" in runnable
    assert "CombiTimeTable ev_001_driving" in runnable
    assert "Modelica.Blocks.Math.Gain pv_001_gain(k=5.7)" in runnable
    assert "connect(HSol_azi0_til35.y, pv_001_gain.u);" in runnable
    assert "connect(pv_001_gain.y, rc.pv_001_P);" in runnable
    interface = network_interface(network)
    export = to_casadi(source, interface.model)
    assert interface.state_names == list(export.state_names)
    assert interface.input_names == list(export.input_names)
    assert interface.parameter_names == list(export.parameter_names)
