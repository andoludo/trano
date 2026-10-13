"""Air-to-water heat pump of the ``FastModel`` library, after trano's Buildings model of the ``air_water_heat_pump``
production (``PartialAirWaterHeatPump``: ``Buildings.Fluid.HeatPumps.ModularReversible.AirToWaterTableData2D`` with
the EN 14511 data of the Viessmann Vitocal 251-A08, charging a mixed buffer).

Buildings interpolates the heating capacity and the electrical power linearly in tables of the condenser outlet
temperature (rows) and of the source temperature (columns), scaled to the nominal capacity of the YAML. The
``Hydronics.HeatPump`` component keeps that interpolation in the source temperature, smoothed at the knots (the
CasADi model must be differentiable), and is exact in the condenser temperature over the rows up to 55 degC with
a quadratic.
"""

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

from trano.fast_model.components.model import Instance

if TYPE_CHECKING:
    from trano.elements.system import Boiler

KELVIN = 273.15
WATER_HEAT_CAPACITY = 4184.0
WATER_DENSITY = 995.586
# Buildings.Fluid.HeatPumps.ModularReversible.Data.TableData2D.EN14511.Vitocal251A08
VITOCAL_SOURCE = np.array([253.15, 258.15, 266.15, 275.15, 280.15, 283.15, 293.15, 303.15, 308.15])
VITOCAL_CONDENSER = np.array([308.15, 318.15, 328.15, 338.15, 343.15])
VITOCAL_HEATING = np.array(
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
VITOCAL_CONDENSER_FLOW = 5600 / 5 / 4184  # [kg/s] mCon_flow_nominal of the record
CONDENSER_ROWS = 3  # rows of the quadratic in the condenser temperature: 35, 45 and 55 degC
CONDENSER_TIME_CONSTANT = 30.0  # [s] tauCon of ModularReversible
# defaults of trano's PartialAirWaterHeatPump
DESIGN_OUTDOOR_TEMPERATURE = 265.15  # [K] TOutDes, also the nominal source temperature
SEPARATOR_VOLUME = 1.5e-5  # [m3/W] VSep without a buffer tank
INSULATION_CONDUCTIVITY = 0.04  # [W/(m.K)]
AMBIENT_TEMPERATURE = 288.15  # [K]


def bilinear(table: np.ndarray, condenser: float, source: float) -> float:
    """Linear interpolation of a Vitocal table, as ``CombiTable2Ds`` (rows: condenser, columns: source)."""
    by_row = [np.interp(source, VITOCAL_SOURCE, row) for row in table]
    return float(np.interp(condenser, VITOCAL_CONDENSER, by_row))


@dataclass(frozen=True)
class Hinges:
    """``a + b*(t - t0) + sum(c[i]*hinge(t - knots[i]))``: the piecewise linear interpolant through values at knots."""

    a: float
    b: float
    c: list[float]

    @classmethod
    def through(cls, knots: np.ndarray, values: np.ndarray) -> "Hinges":
        slopes = np.diff(values) / np.diff(knots)
        return cls(a=float(values[0]), b=float(slopes[0]), c=[float(value) for value in np.diff(slopes)])


def table_parameters(table: np.ndarray, name: str) -> dict[str, float | list[float]]:
    """Hinge parameters of ``q0 + q1*TCon + q2*TCon^2`` (TCon in degC), each coefficient linear in the source."""
    condenser = VITOCAL_CONDENSER[:CONDENSER_ROWS] - KELVIN
    columns = [np.polyfit(condenser, table[:CONDENSER_ROWS, j], 2)[::-1] for j in range(table.shape[1])]
    coefficients = np.array(columns)  # (source knots, 3): q0, q1, q2 at each knot
    parameters: dict[str, float | list[float]] = {}
    for k in range(3):
        hinges = Hinges.through(VITOCAL_SOURCE - KELVIN, coefficients[:, k])
        parameters |= {f"a{name}{k}": hinges.a, f"b{name}{k}": hinges.b, f"c{name}{k}": hinges.c}
    return parameters


def heat_pump_instance(heat_pump: "Boiler", name: str, control: str) -> Instance:
    """``Hydronics.HeatPump`` with the parameters of trano's Buildings heat pump."""
    parameters = heat_pump.parameters
    power = float(parameters.nominal_heating_power)  # type: ignore[union-attr]
    supply = float(parameters.temperature_supply_setpoint_heat_pump)  # type: ignore[union-attr]
    scale = power / bilinear(VITOCAL_HEATING, supply, DESIGN_OUTDOOR_TEMPERATURE)
    condenser_flow = VITOCAL_CONDENSER_FLOW * scale
    tank = str(parameters.use_storage_tank).lower() == "true" or bool(parameters.tank_volume)  # type: ignore[union-attr]
    volume = float(parameters.tank_volume) if tank else SEPARATOR_VOLUME * power  # type: ignore[union-attr]
    loss = 0.0
    if tank:
        height = float(parameters.height_of_tank_without_insulation)  # type: ignore[union-attr]
        insulation = float(parameters.thickness_of_insulation)  # type: ignore[union-attr]
        radius = math.sqrt(volume / (math.pi * height))
        loss = INSULATION_CONDUCTIVITY / insulation * (2 * math.pi * radius * height + 2 * math.pi * radius**2)
    capacity = (WATER_DENSITY * volume + condenser_flow * CONDENSER_TIME_CONSTANT) * WATER_HEAT_CAPACITY
    return Instance(
        name=name,
        component="Hydronics.HeatPump",
        description=f"Air-to-water heat pump {heat_pump.name} ({power:g} W at {supply - KELVIN:g} degC, -8 degC)",
        system=name,
        parameters={
            "m": len(VITOCAL_SOURCE) - 2,
            "tEva0": float(VITOCAL_SOURCE[0] - KELVIN),
            "tEva": [float(knot - KELVIN) for knot in VITOCAL_SOURCE[1:-1]],
            **table_parameters(VITOCAL_HEATING, "Q"),
            **table_parameters(VITOCAL_POWER, "P"),
            "scaFac": scale,
            "dTCon": power / WATER_HEAT_CAPACITY / condenser_flow,
            "UA": loss,
            "TAmb": AMBIENT_TEMPERATURE,
            "C": capacity,
            "T0": supply,
        },
        inputs={"y": control, "TEva": "TOut"},
    )
