"""Key performance indicators of the systems, read from the result file of a case.

The signals are looked up by the name of the component in the YAML (``GENERATOR:001`` is ``generator_001``), under
the containers the libraries nest them in, with one list of candidate variable names per signal so that the same
KPI reads the Buildings and IDEAS component (the Trano package) and the FastSmoothBuildings one. A KPI that a
library cannot provide is left out, never faked.
"""

from __future__ import annotations

import re
from collections.abc import Iterable
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import numpy.typing as npt
from buildingspy.io.outputfile import Reader  # type: ignore[import-untyped]

from trano.elements.space import Space
from trano.elements.system import AirHandlingUnit, Boiler, IdealHeatingCooling, Pump, Valve
from trano.topology import Network
from validation.systems.cases import Case, Family
from validation.systems.models import HEATING_SETPOINT, KELVIN, OCCUPIED_HOURS

FloatArray = npt.NDArray[np.float64]
BoolArray = npt.NDArray[np.bool_]
Kpis = dict[str, float]
HOUR = 3600.0
DAY = 86400.0
JOULES_PER_KWH = 3.6e6
CP_WATER = 4184.0
CP_AIR = 1006.0
RHO_AIR = 1.2
SETPOINT_TOLERANCE = 0.5  # [K] below the set point that counts as discomfort
RUNNING_FRACTION = 0.05  # a generator runs when its power exceeds this fraction of its maximum
VALVE_DEAD_BAND = 0.02  # [-] change of opening that counts as a movement of the valve
# Lower heating value of the default fuel record of Buildings (NaturalGasLowerHeatingValue): 50 MJ/kg x 0.84 kg/m3.
GAS_KWH_PER_M3 = 50e6 * 0.84 / JOULES_PER_KWH

UNITS: dict[str, str] = {
    "outdoor_mean_c": "degC",
    "zone_mean_occupied_c": "degC",
    "zone_min_c": "degC",
    "zone_max_c": "degC",
    "hours_below_setpoint": "h",
    "hp_electricity_kwh": "kWh",
    "hp_heat_kwh": "kWh",
    "hp_scop": "-",
    "hp_cop_running": "-",
    "hp_cop_table": "-",
    "hp_cop_ratio": "-",
    "hp_starts_per_day": "1/d",
    "hp_run_hours": "h",
    "hp_shortest_run_min": "min",
    "hp_shortest_stop_min": "min",
    "hp_speed_min_running": "-",
    "hp_speed_max": "-",
    "hp_condenser_out_running_c": "degC",
    "hp_source_running_c": "degC",
    "hp_supply_setpoint_mean_c": "degC",
    "hp_supply_setpoint_max_c": "degC",
    "hp_supply_mean_running_c": "degC",
    "hp_buffer_mean_c": "degC",
    "hp_buffer_below_band_hours": "h",
    "hp_pump_share": "-",
    "boiler_fuel_kwh": "kWh",
    "boiler_heat_kwh": "kWh",
    "boiler_efficiency": "-",
    "boiler_gas_m3": "m3",
    "boiler_kwh_per_m3": "kWh/m3",
    "boiler_starts_per_day": "1/d",
    "boiler_firing_hours": "h",
    "boiler_shortest_firing_min": "min",
    "boiler_water_max_c": "degC",
    "boiler_water_firing_c": "degC",
    "boiler_eta_firing": "-",
    "tank_top_max_c": "degC",
    "tank_top_min_c": "degC",
    "tank_bottom_max_c": "degC",
    "valve_mean_opening": "-",
    "valve_hours_closed": "h",
    "valve_hours_open": "h",
    "valve_movements_per_day": "1/d",
    "zone_tracking_rmse_k": "K",
    "radiator_heat_kwh": "kWh",
    "pump_electricity_kwh": "kWh",
    "pump_run_hours": "h",
    "pump_hours_without_demand": "h",
    "mixing_error_k": "K",
    "mixing_max_overshoot_k": "K",
    "heating_kwh": "kWh",
    "heating_peak_w": "W",
    "vent_flow_kg_s": "kg/s",
    "vent_supply_mean_c": "degC",
    "vent_supply_min_c": "degC",
    "vent_effectiveness_measured": "-",
    "vent_fan_power_w": "W",
    "vent_fan_electricity_kwh": "kWh",
    "vent_sfp_w_per_m3s": "W/(m3/s)",
    "vent_recovered_kwh": "kWh",
    "vent_heat_loss_kwh": "kWh",
    "vent_hours_supply_above_zone": "h",
    "vent_heater_electricity_kwh": "kWh",
    "vent_bypass_hours": "h",
}


@dataclass(frozen=True)
class Series:
    time: FloatArray
    values: FloatArray

    def mean(self) -> float:
        """Time-weighted mean (trapezoids)."""
        if len(self.time) < 2:
            return float(self.values.mean())
        return float(np.trapezoid(self.values, self.time) / (self.time[-1] - self.time[0]))

    def integral_kwh(self) -> float:
        return float(np.trapezoid(self.values, self.time)) / JOULES_PER_KWH

    def hours_where(self, mask: BoolArray) -> float:
        """Hours during which ``mask`` holds, each sample holding until the next one."""
        durations = np.diff(self.time)
        return float(np.sum(durations[mask[:-1]])) / HOUR

    def where(self, mask: BoolArray) -> FloatArray:
        return self.values[mask]

    def hourly(self) -> list[float]:
        """Means over the hours of the period, for the figures."""
        edges = np.arange(self.time[0], self.time[-1] + 1e-6, HOUR)
        if len(edges) < 2:
            return [float(self.values.mean())]
        grid = np.union1d(self.time, edges)
        on_grid = np.interp(grid, self.time, self.values)
        cumulative = np.concatenate([[0.0], np.cumsum(np.diff(grid) * (on_grid[1:] + on_grid[:-1]) / 2)])
        return [float(value) for value in np.diff(np.interp(edges, grid, cumulative)) / HOUR]


class Result:
    """A result file with its variables looked up by candidate suffixes."""

    def __init__(self, path: Path) -> None:
        self.reader = Reader(str(path), "dymola")
        self.names: list[str] = self.reader.varNames()

    def find(self, suffixes: Iterable[str]) -> str | None:
        """The first variable ending with one of the suffixes (as ``<container>.<suffix>`` or the suffix itself)."""
        for suffix in suffixes:
            pattern = re.compile(rf"(^|\.){re.escape(suffix)}$")
            matches = [name for name in self.names if pattern.search(name)]
            if matches:
                return sorted(matches, key=len)[0]
        return None

    def series(self, suffixes: Iterable[str]) -> Series | None:
        name = self.find(suffixes)
        if name is None:
            return None
        time, values = self.reader.values(name)
        return Series(np.asarray(time, dtype=float), np.asarray(values, dtype=float))

    def require(self, suffixes: Iterable[str]) -> Series:
        series = self.series(suffixes)
        if series is None:
            raise ValueError(f"None of {list(suffixes)} is in the result file; it holds {len(self.names)} variables")
        return series


def _name(element: Any) -> str:  # noqa: ANN401
    return str(element.name)


def _elements(network: Network, kind: type) -> list[Any]:
    return sorted((node for node in network.graph.nodes if isinstance(node, kind)), key=_name)


def on_grid(reference: Series, other: Series) -> FloatArray:
    return np.asarray(np.interp(reference.time, other.time, other.values), dtype=float)


def occupied(time: FloatArray) -> BoolArray:
    start, end = (hour * HOUR for hour in OCCUPIED_HOURS)
    hour_of_day = time % DAY
    return np.asarray((hour_of_day >= start) & (hour_of_day < end), dtype=bool)


def runs(time: FloatArray, running: BoolArray) -> tuple[int, list[float], list[float]]:
    """Starts, durations of the runs and of the stops [s] of a boolean signal (edges inside the period)."""
    edges = np.diff(running.astype(int))
    starts = np.flatnonzero(edges == 1) + 1
    stops = np.flatnonzero(edges == -1) + 1
    run_durations = [float(time[stop] - time[start]) for start in starts for stop in stops[stops > start][:1]]
    stop_durations = [float(time[start] - time[stop]) for stop in stops for start in starts[starts > stop][:1]]
    return len(starts), run_durations, stop_durations


def movements(values: FloatArray, dead_band: float = VALVE_DEAD_BAND) -> int:
    """Reversals of the direction of a signal, ignoring changes smaller than the dead band."""
    direction = 0
    last = values[0]
    count = 0
    for value in values[1:]:
        if abs(value - last) < dead_band:
            continue
        new_direction = 1 if value > last else -1
        if direction and new_direction != direction:
            count += 1
        direction = new_direction
        last = value
    return count


# --- EN 14511 data of the Vitocal 251-A08, the record of trano's air-water heat pump (Buildings, FastSmoothBuildings)
VITOCAL_SOURCE = np.array([253.15, 258.15, 266.15, 275.15, 280.15, 283.15, 293.15, 303.15, 308.15])
VITOCAL_CONDENSER = np.array([308.15, 318.15, 328.15, 338.15, 343.15])
VITOCAL_HEAT = np.array(
    [
        [4490, 5170, 6470, 6790, 8000, 10210, 12330, 12310, 13090],
        [4230, 4900, 6260, 6780, 8370, 9970, 11520, 13040, 12640],
        [3780, 4710, 6030, 6830, 8380, 9940, 11500, 13070, 13110],
        [0, 3170, 4610, 6320, 8140, 9550, 11290, 12100, 12180],
        [0, 0, 3830, 5560, 7600, 8700, 11290, 12500, 12590],
    ],
    dtype=float,
)
VITOCAL_POWER = np.array(
    [
        [2128, 2238, 2396, 1835, 1633, 2006, 1720, 1465, 1558],
        [2299, 2402, 2641, 2173, 2244, 2462, 2110, 1734, 1584],
        [2305, 2631, 2899, 2493, 2669, 2915, 2556, 2164, 2081],
        [0, 2032, 2478, 2760, 3095, 3375, 3085, 2521, 2511],
        [0, 0, 2201, 2699, 3193, 3398, 3330, 2822, 2817],
    ],
    dtype=float,
)


def _interpolate_table(table: FloatArray, condenser: float, source: float) -> float:
    """Bilinear interpolation of an EN 14511 table (rows: condenser outlet, columns: source), the zeros of the
    table (outside the operating envelope) left out of the interpolation."""
    condenser = float(np.clip(condenser, VITOCAL_CONDENSER[0], VITOCAL_CONDENSER[-1]))
    source = float(np.clip(source, VITOCAL_SOURCE[0], VITOCAL_SOURCE[-1]))
    rows = []
    for row in table:
        valid = row > 0
        rows.append(float(np.interp(source, VITOCAL_SOURCE[valid], row[valid])))
    return float(np.interp(condenser, VITOCAL_CONDENSER, np.array(rows)))


def table_cop(condenser_outlet: float, source: float) -> float:
    """COP of the Vitocal 251-A08 data at a condenser outlet and a source temperature [K]."""
    return _interpolate_table(VITOCAL_HEAT, condenser_outlet, source) / _interpolate_table(
        VITOCAL_POWER, condenser_outlet, source
    )


# --- KPIs per family ---------------------------------------------------------------------------------------------


def common_kpis(result: Result, network: Network, traces: dict[str, list[float]]) -> Kpis:
    space = _elements(network, Space)[0]
    zone = result.require([f"dataBus.TZon{_name(space).capitalize()}"])
    outdoor = result.series(["dataBus.TDryBul", "weaBus.TDryBul", "TDryBul"])
    mask = occupied(zone.time)
    below = zone.values < HEATING_SETPOINT - SETPOINT_TOLERANCE
    kpis: Kpis = {
        "zone_mean_occupied_c": float(zone.values[mask].mean()) - KELVIN,
        "zone_min_c": float(zone.values.min()) - KELVIN,
        "zone_max_c": float(zone.values.max()) - KELVIN,
        "hours_below_setpoint": zone.hours_where(below & mask),
    }
    traces["zone_c"] = [value - KELVIN for value in zone.hourly()]
    if outdoor is not None:
        kpis["outdoor_mean_c"] = outdoor.mean() - KELVIN
        traces["outdoor_c"] = [value - KELVIN for value in outdoor.hourly()]
    return kpis


def heat_pump_kpis(result: Result, network: Network, traces: dict[str, list[float]]) -> Kpis:
    generator = _name(_elements(network, Boiler)[0])
    power = result.require([f"{generator}.PEle.y", f"{generator}.PEle"])
    heat = result.require([f"{generator}.QHea.y", f"{generator}.QCon"])
    electricity = result.require([f"{generator}.EEle.y", f"{generator}.EEle"])
    heat_energy = result.series([f"{generator}.EHea.y"])
    speed = result.require([f"{generator}.ySet.y", f"{generator}.y"])
    buffer = result.require([f"{generator}.TBuf.T", f"{generator}.T"])
    condenser_out = result.require([f"{generator}.conOutTem.T", f"{generator}.TConOut"])
    set_point = result.require([f"{generator}.TSupSet.y", f"{generator}_control.TSupSet"])
    source = result.series([f"{generator}.souPum.T_in", f"{generator}.weaBus.TDryBul"])
    supply = result.series([f"{generator}.supTemp.T"])
    pump = result.series([f"{generator}.pumCon.P"])
    band = result.series([f"{generator}.dTHys", f"{generator}_control.dTHys"])

    time = power.time
    running = np.asarray(power.values > RUNNING_FRACTION * max(float(power.values.max()), 0.0), dtype=bool)
    starts, run_durations, stop_durations = runs(time, running)
    days = (time[-1] - time[0]) / DAY
    electricity_kwh = float(electricity.values[-1] - electricity.values[0])
    heat_kwh = float(heat_energy.values[-1] - heat_energy.values[0]) if heat_energy is not None else heat.integral_kwh()
    kpis: Kpis = {
        "hp_electricity_kwh": electricity_kwh,
        "hp_heat_kwh": heat_kwh,
        "hp_starts_per_day": starts / days,
        "hp_run_hours": power.hours_where(running),
        "hp_speed_max": float(speed.values.max()),
        "hp_supply_setpoint_mean_c": set_point.mean() - KELVIN,
        "hp_supply_setpoint_max_c": float(set_point.values.max()) - KELVIN,
        "hp_buffer_mean_c": buffer.mean() - KELVIN,
    }
    if electricity_kwh > 0:
        kpis["hp_scop"] = heat_kwh / electricity_kwh
    if running.any():
        on_power = np.trapezoid(np.where(running, power.values, 0.0), time)
        on_heat = np.trapezoid(np.where(running, on_grid(power, heat), 0.0), time)
        kpis["hp_cop_running"] = float(on_heat / on_power)
        kpis["hp_speed_min_running"] = (
            float(speed.values[speed.values > 0.01].min()) if (speed.values > 0.01).any() else 0.0
        )
        condenser = float(on_grid(power, condenser_out)[running].mean())
        kpis["hp_condenser_out_running_c"] = condenser - KELVIN
        if source is not None:
            source_running = float(on_grid(power, source)[running].mean())
            kpis["hp_source_running_c"] = source_running - KELVIN
            kpis["hp_cop_table"] = table_cop(condenser, source_running)
            kpis["hp_cop_ratio"] = kpis["hp_cop_running"] / kpis["hp_cop_table"]
        if supply is not None:
            kpis["hp_supply_mean_running_c"] = float(on_grid(power, supply)[running].mean()) - KELVIN
        if run_durations:
            kpis["hp_shortest_run_min"] = min(run_durations) / 60
        if stop_durations:
            kpis["hp_shortest_stop_min"] = min(stop_durations) / 60
        if pump is not None and on_power > 0:
            kpis["hp_pump_share"] = float(np.trapezoid(on_grid(power, pump), time) / np.trapezoid(power.values, time))
    if band is not None:
        half_band = float(band.values[0]) / 2
        low = on_grid(buffer, set_point) - half_band - 1.0
        kpis["hp_buffer_below_band_hours"] = buffer.hours_where((buffer.values < low) & (on_grid(buffer, power) > 0))
    traces["hp_power_w"] = power.hourly()
    traces["hp_heat_w"] = heat.hourly()
    traces["hp_buffer_c"] = [value - KELVIN for value in buffer.hourly()]
    traces["hp_supply_setpoint_c"] = [value - KELVIN for value in set_point.hourly()]
    return kpis


def boiler_kpis(result: Result, network: Network, traces: dict[str, list[float]]) -> Kpis:
    generator = _name(_elements(network, Boiler)[0])
    fuel = result.require([f"{generator}.QFue.y", f"{generator}.QFue"])
    heat = result.require([f"{generator}.QHea.y", f"{generator}.QWat"])
    fuel_energy = result.require([f"{generator}.EFue.y", f"{generator}.EFue"])
    heat_energy = result.series([f"{generator}.EHea.y"])
    gas = result.series([f"{generator}.GasUsage.y"])
    firing = result.require([f"{generator}.Boiy.y", f"{generator}.yBoi.y", f"{generator}.y"])
    water = result.require([f"{generator}.boi.T", f"{generator}.T"])
    eta = result.series([f"{generator}.boi.eta", f"{generator}.eta"])
    tank_top = result.series([f"{generator}.tanTemTop.T"])
    tank_bottom = result.series([f"{generator}.tanTemBot.T"])

    time = fuel.time
    running = np.asarray(fuel.values > RUNNING_FRACTION * max(float(fuel.values.max()), 0.0), dtype=bool)
    starts, run_durations, _ = runs(time, running)
    days = (time[-1] - time[0]) / DAY
    fuel_kwh = float(fuel_energy.values[-1] - fuel_energy.values[0])
    heat_kwh = float(heat_energy.values[-1] - heat_energy.values[0]) if heat_energy is not None else heat.integral_kwh()
    kpis: Kpis = {
        "boiler_fuel_kwh": fuel_kwh,
        "boiler_heat_kwh": heat_kwh,
        "boiler_starts_per_day": starts / days,
        "boiler_firing_hours": fuel.hours_where(running),
        "boiler_water_max_c": float(water.values.max()) - KELVIN,
    }
    if fuel_kwh > 0:
        kpis["boiler_efficiency"] = heat_kwh / fuel_kwh
    if gas is not None:
        kpis["boiler_gas_m3"] = float(gas.values[-1] - gas.values[0])
        if kpis["boiler_gas_m3"] > 0:
            kpis["boiler_kwh_per_m3"] = fuel_kwh / kpis["boiler_gas_m3"]
    if running.any():
        kpis["boiler_water_firing_c"] = float(on_grid(fuel, water)[running].mean()) - KELVIN
        if eta is not None:
            kpis["boiler_eta_firing"] = float(on_grid(fuel, eta)[running].mean())
        if run_durations:
            kpis["boiler_shortest_firing_min"] = min(run_durations) / 60
    if tank_top is not None:
        kpis["tank_top_max_c"] = float(tank_top.values.max()) - KELVIN
        kpis["tank_top_min_c"] = float(tank_top.values[len(tank_top.values) // 4 :].min()) - KELVIN
        traces["tank_top_c"] = [value - KELVIN for value in tank_top.hourly()]
    if tank_bottom is not None:
        kpis["tank_bottom_max_c"] = float(tank_bottom.values.max()) - KELVIN
    traces["boiler_fuel_w"] = fuel.hourly()
    traces["boiler_heat_w"] = heat.hourly()
    traces["boiler_water_c"] = [value - KELVIN for value in water.hourly()]
    _ = firing
    return kpis


def hydronic_kpis(result: Result, network: Network, traces: dict[str, list[float]]) -> Kpis:
    """Valve, pump, three-way valve and radiator of a hydronic case."""
    valves, pumps = _elements(network, Valve), _elements(network, Pump)
    if not valves:
        return {}
    valve = _name(valves[0])
    opening = result.require([f"{valve}.y_actual", f"{valve}.y"])
    space = _elements(network, Space)[0]
    zone = result.require([f"dataBus.TZon{_name(space).capitalize()}"])
    radiator = result.series(["radiator_001.Q_flow", "radiator_001.QTot_flow", "radiator_001.Q"])
    days = (opening.time[-1] - opening.time[0]) / DAY
    mask = occupied(zone.time)
    error = zone.values[mask] - HEATING_SETPOINT
    kpis: Kpis = {
        "valve_mean_opening": opening.mean(),
        "valve_hours_closed": opening.hours_where(opening.values < 0.01),
        "valve_hours_open": opening.hours_where(opening.values > 0.99),
        "valve_movements_per_day": movements(opening.values) / days,
        "zone_tracking_rmse_k": float(np.sqrt(np.mean(error**2))),
    }
    if radiator is not None:
        kpis["radiator_heat_kwh"] = abs(radiator.integral_kwh())  # Buildings counts the heat leaving the water
    if pumps:
        pump = _name(pumps[0])
        pump_power = result.series([f"{pump}.pumRad.P", f"{pump}.P"])
        if pump_power is not None:
            threshold = RUNNING_FRACTION * max(float(pump_power.values.max()), 0.0)
            running = np.asarray(pump_power.values > threshold, dtype=bool)
            kpis["pump_electricity_kwh"] = pump_power.integral_kwh()
            kpis["pump_run_hours"] = pump_power.hours_where(running)
            kpis["pump_hours_without_demand"] = pump_power.hours_where(running & (on_grid(pump_power, opening) < 0.01))
            traces["pump_power_w"] = pump_power.hourly()
    sensor = result.series(["temperature_sensor_001.T"])
    set_point = result.series(["three_way_valve_control_001.conVal.u_s", "three_way_valve_control_001.TColSet"])
    pump_power = result.series([f"{_name(pumps[0])}.pumRad.P", f"{_name(pumps[0])}.P"]) if pumps else None
    if sensor is not None and set_point is not None and pump_power is not None:
        running = np.asarray(
            on_grid(sensor, pump_power) > RUNNING_FRACTION * max(float(pump_power.values.max()), 0.0), dtype=bool
        )
        if running.any():
            deviation = sensor.values - on_grid(sensor, set_point)
            kpis["mixing_error_k"] = float(np.abs(deviation[running]).mean())
            kpis["mixing_max_overshoot_k"] = float(deviation[running].max())
        traces["mixing_supply_c"] = [value - KELVIN for value in sensor.hourly()]
    traces["valve_opening"] = opening.hourly()
    return kpis


def ventilation_kpis(result: Result, network: Network, traces: dict[str, list[float]]) -> Kpis:  # noqa: C901
    kpis: Kpis = {}
    heatings = _elements(network, IdealHeatingCooling)
    if heatings:
        heating = result.require([f"{_name(heatings[0])}.QHea_flow"])
        kpis["heating_kwh"] = heating.integral_kwh()
        kpis["heating_peak_w"] = float(heating.values.max())
        traces["heating_w"] = heating.hourly()
    units = _elements(network, AirHandlingUnit)
    if not units:
        return kpis
    unit = _name(units[0])
    space = _elements(network, Space)[0]
    zone = result.require([f"dataBus.TZon{_name(space).capitalize()}"])
    outdoor = result.require(["dataBus.TDryBul", f"{unit}.weaBus.TDryBul", "weaBus.TDryBul"])
    supply = result.require([f"{unit}.TSup.T", f"{unit}.TSup"])
    extract = result.series([f"{unit}.TExt"]) or zone
    fan_power = result.series([f"{unit}.PFan.y", f"{unit}.PFan", f"dataBus.fan_power{unit.capitalize()}"])
    if fan_power is None:  # the Guideline 36 unit: its supply and return fans
        supply_fan, return_fan = result.require([f"{unit}.fanSup.P"]), result.require([f"{unit}.fanSup1.P"])
        fan_power = Series(supply_fan.time, supply_fan.values + on_grid(supply_fan, return_fan))
    heater = result.series([f"{unit}.PHea.y"])
    bypass = result.series([f"{unit}.yByp.y"])
    flow = result.series([f"{unit}.fanSup.m_flow", f"{unit}.fanExt.m_flow"])  # the flow of the unit, every duct
    if flow is None:  # FastSmoothBuildings: the unit sets the fraction y of its design flow
        speed, nominal = result.series([f"{unit}.y"]), result.series([f"{unit}.m_flow_nominal"])
        if speed is not None and nominal is not None:
            flow = Series(speed.time, speed.values * float(nominal.values[0]))
    recovered = result.series([f"{unit}.QRec.y"])
    time = supply.time
    zone_on, outdoor_on, extract_on = on_grid(supply, zone), on_grid(supply, outdoor), on_grid(supply, extract)
    flow_values = on_grid(supply, flow) if flow is not None else np.full_like(time, np.nan)
    spread = extract_on - outdoor_on
    meaningful = spread > 2.0
    kpis.update(
        {
            "vent_supply_mean_c": supply.mean() - KELVIN,
            "vent_supply_min_c": float(supply.values.min()) - KELVIN,
            "vent_fan_power_w": fan_power.mean(),
            "vent_fan_electricity_kwh": fan_power.integral_kwh(),
            "vent_hours_supply_above_zone": supply.hours_where(supply.values > zone_on + 0.1),
        }
    )
    if meaningful.any():
        kpis["vent_effectiveness_measured"] = float(
            np.mean((supply.values[meaningful] - outdoor_on[meaningful]) / spread[meaningful])
        )
    if flow is not None and np.nanmean(flow_values) > 0:
        kpis["vent_flow_kg_s"] = float(np.nanmean(flow_values))
        kpis["vent_sfp_w_per_m3s"] = kpis["vent_fan_power_w"] / (kpis["vent_flow_kg_s"] / RHO_AIR)
        loss = Series(time, flow_values * CP_AIR * (zone_on - supply.values))
        kpis["vent_heat_loss_kwh"] = loss.integral_kwh()
        if recovered is None:
            recovered = Series(time, flow_values * CP_AIR * (supply.values - outdoor_on))
    if recovered is not None:
        kpis["vent_recovered_kwh"] = recovered.integral_kwh()
    if heater is not None:
        kpis["vent_heater_electricity_kwh"] = heater.integral_kwh()
    if bypass is not None:
        kpis["vent_bypass_hours"] = bypass.hours_where(bypass.values > 0.5)
    traces["vent_supply_c"] = [value - KELVIN for value in supply.hourly()]
    return kpis


FAMILY_KPIS = {
    Family.heat_pump: (heat_pump_kpis, hydronic_kpis),
    Family.boiler: (boiler_kpis, hydronic_kpis),
    Family.hydronics: (hydronic_kpis, boiler_kpis),
    Family.ventilation: (ventilation_kpis,),
    Family.house: (heat_pump_kpis, boiler_kpis, hydronic_kpis, ventilation_kpis),
}


def extract_kpis(case: Case, library: str, network: Network, result_file: Path) -> tuple[Kpis, dict[str, list[float]]]:
    """The KPIs of the case and the hourly traces of its main signals."""
    result = Result(result_file)
    traces: dict[str, list[float]] = {}
    kpis = common_kpis(result, network, traces)
    for function in FAMILY_KPIS[case.family]:
        try:
            kpis.update(function(result, network, traces))
        except (ValueError, IndexError):
            if case.family is Family.house:
                continue  # a house has not every system
            raise
    _ = library
    return kpis, traces
