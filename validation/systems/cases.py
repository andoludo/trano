"""The cases of the systems validation: one YAML each in ``cases/``, written by ``write_cases``.

A case puts one system on the study zone of ``models`` for one week of the Uccle weather (the coldest week in
January, a week in March, a week in July) and asks one question: does the heat pump's COP follow its source
temperature, does the boiler burn what it delivers divided by its efficiency, does the ventilation unit blow cold
air... The KPIs that answer it are read by ``kpi`` and judged by ``expected.yaml``.
"""

from __future__ import annotations

from enum import Enum
from pathlib import Path
from typing import Any

from pydantic import BaseModel, Field

from validation.systems import models
from validation.systems.models import KELVIN

CASES_DIR = Path(__file__).parent.joinpath("cases")
DAY = 24 * 3600
LIBRARIES = ("Buildings", "IDEAS", "FastSmoothBuildings")
HYDRONIC_LIBRARIES = LIBRARIES

# Weeks of the Uccle TMYx file (day of the year, 0 based): the coldest week (daily means -2 to 0 degC), a mild
# week of March (5 to 9 degC) and a warm week of July (14 to 24 degC).
WINTER, SHOULDER, SUMMER = 17, 74, 195
WEEK = 7


class Family(str, Enum):
    heat_pump = "heat_pump"
    boiler = "boiler"
    hydronics = "hydronics"
    ventilation = "ventilation"
    house = "house"


class Case(BaseModel):
    id: str
    family: Family
    description: str
    start_day: int = WINTER
    days: int = WEEK
    libraries: tuple[str, ...] = LIBRARIES
    output_interval: int = Field(default=300, description="[s] between two output points")
    companion: str | None = Field(default=None, description="Case whose KPIs this one is compared with.")
    building: dict[str, Any] = Field(repr=False)

    @property
    def start_time(self) -> int:
        return self.start_day * DAY

    @property
    def end_time(self) -> int:
        return (self.start_day + self.days) * DAY

    @property
    def path(self) -> Path:
        return case_file(self.id)

    def supports(self, library: str) -> bool:
        return library in self.libraries


def case_file(case_id: str, directory: Path = CASES_DIR) -> Path:
    return directory.joinpath(f"{case_id}.yaml")


# --- heat pumps --------------------------------------------------------------------------------------------------

RADIATOR_45 = models.radiator_and_valve(45.0 + KELVIN, 35.0 + KELVIN)
RADIATOR_70 = models.radiator_and_valve(70.0 + KELVIN, 50.0 + KELVIN)
# The radiator of the heat pump cases: 4 kW at 45/35 degC, a third above the design heat loss, as radiators are
# oversized when a heat pump replaces a boiler, so that the heating curve (45 degC at -8 degC) covers the load and
# the morning recovery after the night setback.
RADIATOR_45_HEAT_PUMP = models.radiator_and_valve(45.0 + KELVIN, 35.0 + KELVIN, power=4000.0)
TABLE_LIBRARIES = ("Buildings", "IDEAS")
"""The libraries whose heat pump takes a constant source temperature: FastSmoothBuildings always takes the
outdoor air (the YAML parameters of the source are not among those its template passes)."""
VITOCAL_TABLE = {
    "heat_pump_data": "table",
    "heat_pump_table_source_temperatures": [253.15, 258.15, 266.15, 275.15, 280.15, 283.15, 293.15, 303.15, 308.15],
    "heat_pump_table_condenser_temperatures": [308.15, 318.15, 328.15, 338.15, 343.15],
    "heat_pump_table_heating_capacity": (
        "4490, 5170, 6470, 6790, 8000, 10210, 12330, 12310, 13090; "
        "4230, 4900, 6260, 6780, 8370, 9970, 11520, 13040, 12640; "
        "3780, 4710, 6030, 6830, 8380, 9940, 11500, 13070, 13110; "
        "0, 3170, 4610, 6320, 8140, 9550, 11290, 12100, 12180; "
        "0, 0, 3830, 5560, 7600, 8700, 11290, 12500, 12590"
    ),
    "heat_pump_table_electrical_power": (
        "2128, 2238, 2396, 1835, 1633, 2006, 1720, 1465, 1558; "
        "2299, 2402, 2641, 2173, 2244, 2462, 2110, 1734, 1584; "
        "2305, 2631, 2899, 2493, 2669, 2915, 2556, 2164, 2081; "
        "0, 2032, 2478, 2760, 3095, 3375, 3085, 2521, 2511; "
        "0, 0, 2201, 2699, 3193, 3398, 3330, 2822, 2817"
    ),
}
"""The EN 14511 data of the Vitocal 251-A08 as the YAML gives a table: the record of Buildings written out."""
FLAT_CURVE = {"minimum_supply_temperature": 45.0 + KELVIN, "heating_limit_temperature": 35.0 + KELVIN}
"""A heating curve flat at 45 degC and a heating limit the summer never reaches: the heat pump always holds the
same supply temperature, whatever the weather, so that its COP depends on the source temperature only."""


def heat_pump_case(  # noqa: PLR0913
    case_id: str,
    description: str,
    start_day: int = WINTER,
    companion: str | None = None,
    heat_pump: dict[str, Any] | None = None,
    libraries: tuple[str, ...] = LIBRARIES,
) -> Case:
    """A case of the heat pump family: ``heat_pump`` holds the arguments of ``models.air_water_heat_pump``."""
    heat_pump = heat_pump or {}
    supply = heat_pump.get("supply", 45.0 + KELVIN)
    return Case(
        id=case_id,
        family=Family.heat_pump,
        description=description,
        start_day=start_day,
        companion=companion,
        libraries=libraries,
        building=models.building(
            models.zone(),
            emissions=RADIATOR_45_HEAT_PUMP,
            systems=models.mixing_circuit(models.air_water_heat_pump(**heat_pump), supply=supply),
        ),
    )


def fixed_source(temperature_celsius: float, supply_celsius: float = 45.0) -> dict[str, Any]:
    """The evaporator at a constant source temperature and a flat heating curve at ``supply_celsius``."""
    return {
        "supply": supply_celsius + KELVIN,
        "parameters": {
            "heat_pump_source_outdoor_air": False,
            "temperature_heatpump_source": temperature_celsius + KELVIN,
            "minimum_supply_temperature": supply_celsius + KELVIN,
            "heating_limit_temperature": 35.0 + KELVIN,
        },
    }


HEAT_PUMP_CASES = [
    heat_pump_case(
        "hp_winter",
        "Air-water heat pump, 4 kW at -8/45 degC, 100 l buffer, heating curve 45 to 30 degC, the coldest week",
    ),
    heat_pump_case("hp_shoulder", "The same heat pump in a mild March week (part load)", start_day=SHOULDER),
    heat_pump_case(
        "hp_summer", "The same heat pump in July: above the heating limit it must not run", start_day=SUMMER
    ),
    heat_pump_case(
        "hp_no_buffer",
        "The same heat pump on a hydraulic separator instead of the 100 l buffer: more cycles",
        companion="hp_winter",
        heat_pump={"buffer_volume": None},
    ),
    *[
        heat_pump_case(
            f"hp_source_{'m' if t < 0 else ''}{abs(t)}",
            f"COP at a constant {t} degC source and 45 degC supply (EN 14511 point A{t}/W45)",
            heat_pump=fixed_source(t),
            libraries=TABLE_LIBRARIES,
        )
        for t in (-10, -7, 2, 7, 12)
    ],
    heat_pump_case(
        "hp_table_winter",
        "The air-water heat pump given as a table (heat_pump_data: table) holding the Vitocal 251-A08 data: it "
        "must reproduce hp_winter",
        companion="hp_winter",
        heat_pump={"parameters": VITOCAL_TABLE},
        libraries=TABLE_LIBRARIES,
    ),
    heat_pump_case(
        "hp_carnot_winter",
        "The air-water heat pump as a Carnot model (effectiveness 0.45, frosting of the evaporator) in the "
        "coldest week",
        companion="hp_winter",
        heat_pump={"parameters": {"heat_pump_data": "carnot"}},
        libraries=TABLE_LIBRARIES,
    ),
    heat_pump_case(
        "gshp_winter",
        "Ground-source heat pump (Carnot, effectiveness 0.45) on a ground loop at 10 degC (3 K annual swing) in "
        "the coldest week",
        companion="hp_winter",
        heat_pump={"variant": "ground_source_heat_pump"},
        libraries=TABLE_LIBRARIES,
    ),
    heat_pump_case(
        "wwhp_winter",
        "Water-water heat pump (Carnot, effectiveness 0.45) on a water loop at 12 degC in the coldest week",
        companion="hp_winter",
        heat_pump={"variant": "water_water_heat_pump", "parameters": {"temperature_heatpump_source": 285.15}},
        libraries=TABLE_LIBRARIES,
    ),
    *[
        heat_pump_case(
            f"hp_supply_{s}",
            f"COP at a constant 7 degC source and {s} degC supply (EN 14511 point A7/W{s})",
            companion="hp_source_7",
            heat_pump=fixed_source(7, s),
            libraries=TABLE_LIBRARIES,
        )
        for s in (35, 55)
    ],
]

# --- boilers -----------------------------------------------------------------------------------------------------

CONDENSING_CURVE = {
    "effcur": "Buildings.Fluid.Types.EfficiencyCurves.QuadraticLinear",
    # eta = a1 + a4 T: 0.97 at 40 degC, 0.90 at 70 degC (the rating temperature): a condensing boiler whose
    # efficiency rises as its water cools, against the lower heating value
    "coefficients_for_efficiency_curve": "{1.699, 0, 0, -0.002333, 0, 0}",
    "temperature_used_to_compute_nominal_efficiency": 70.0 + KELVIN,
}


def boiler_case(  # noqa: PLR0913
    case_id: str,
    description: str,
    start_day: int = WINTER,
    companion: str | None = None,
    emissions: list[dict[str, Any]] = RADIATOR_70,
    boiler: dict[str, Any] | None = None,
) -> Case:
    """A case of the boiler family: ``boiler`` holds the arguments of ``models.gas_boiler``."""
    boiler = boiler or {}
    supply = boiler.get("supply", 70.0 + KELVIN)
    return Case(
        id=case_id,
        family=Family.boiler,
        description=description,
        start_day=start_day,
        companion=companion,
        building=models.building(
            models.zone(),
            emissions=emissions,
            systems=models.mixing_circuit(models.gas_boiler(**boiler), supply=supply),
        ),
    )


BOILER_CASES = [
    boiler_case("boiler_storage_winter", "Gas boiler, 6 kW, 200 l storage tank, 70 degC set point, the coldest week"),
    boiler_case(
        "boiler_no_storage_winter",
        "Gas boiler without storage, 6 kW, 70 degC supply set point, the coldest week",
        companion="boiler_storage_winter",
        boiler={"storage": False},
    ),
    boiler_case(
        "boiler_condensing_winter",
        "Condensing boiler without storage, efficiency 0.97 at 40 degC to 0.90 at 70 degC, radiators 45/35",
        companion="boiler_no_storage_winter",
        emissions=RADIATOR_45,
        boiler={"storage": False, "supply": 45.0 + KELVIN, "parameters": CONDENSING_CURVE},
    ),
    boiler_case(
        "boiler_storage_shoulder", "The boiler with storage in a mild March week (part load)", start_day=SHOULDER
    ),
    boiler_case(
        "boiler_storage_summer",
        "The boiler with storage in July: no heat demand, only the losses of the tank",
        start_day=SUMMER,
    ),
]

# --- valves and pumps --------------------------------------------------------------------------------------------


def valve_case(
    case_id: str, description: str, companion: str | None = None, emission: dict[str, Any] | None = None
) -> Case:
    """A case of the valve family: ``emission`` holds the arguments of ``models.radiator_and_valve``."""
    emission = emission or {}
    return Case(
        id=case_id,
        family=Family.hydronics,
        description=description,
        companion=companion,
        building=models.building(
            models.zone(),
            emissions=models.radiator_and_valve(70.0 + KELVIN, 50.0 + KELVIN, **emission),
            systems=models.mixing_circuit(models.gas_boiler(), supply=70.0 + KELVIN),
        ),
    )


HYDRONIC_CASES = [
    valve_case("valve_p", "Radiator valve under a proportional controller on the zone temperature (PController)"),
    valve_case(
        "valve_pi",
        "Radiator valve under the Guideline 36 heating loop (the default emission control)",
        companion="valve_p",
        emission={"control_variant": "default"},
    ),
    valve_case(
        "valve_onoff",
        "Radiator valve under an on-off controller",
        companion="valve_p",
        emission={"control_variant": "onoff"},
    ),
    valve_case(
        "valve_nonlinear",
        "The proportional case with the valve's flow law not linearised (trano linearises it by default)",
        companion="valve_p",
        emission={"valve_parameters": {"use_linear_relation_between_m_flow_and_dp_for_any_flow_rate": "false"}},
    ),
]

# --- ventilation -------------------------------------------------------------------------------------------------


def ventilation_case(  # noqa: PLR0913
    case_id: str,
    description: str,
    start_day: int = WINTER,
    companion: str | None = None,
    libraries: tuple[str, ...] = LIBRARIES,
    unit: tuple[dict[str, Any], list[dict[str, Any]]] | None = None,
    natural_ventilation: float | None = None,
) -> Case:
    ducts, systems = unit if unit is not None else ({}, [])
    return Case(
        id=case_id,
        family=Family.ventilation,
        description=description,
        start_day=start_day,
        companion=companion,
        libraries=libraries,
        building=models.building(
            models.zone(natural_ventilation=natural_ventilation),
            emissions=models.ideal_heating(),
            systems=systems,
            ventilation=ducts,
        ),
    )


NIGHT_SETBACK = "[0, 0.5; 25200, 1; 79200, 0.5]"
"""Half the design flow from 22 to 7 h."""


VENTILATION_CASES = [
    ventilation_case("no_ventilation_winter", "The zone heated by an ideal heater, infiltration only (n50 = 3)"),
    ventilation_case(
        "hrv_winter",
        "Balanced ventilation with 80 % heat recovery (system_d), 150 m3/h, the coldest week",
        companion="no_ventilation_winter",
        unit=models.ventilation_unit(),
    ),
    ventilation_case(
        "balanced_no_recovery_winter",
        "The same balanced ventilation without heat recovery (effectiveness 0)",
        companion="no_ventilation_winter",
        unit=models.ventilation_unit(effectiveness=0.0),
    ),
    ventilation_case(
        "no_ventilation_summer", "The zone in July without ventilation (free floating above the set point)", SUMMER
    ),
    ventilation_case(
        "hrv_summer",
        "Balanced ventilation with heat recovery in July: without a bypass it recovers heat it should not",
        SUMMER,
        companion="no_ventilation_summer",
        unit=models.ventilation_unit(),
    ),
    ventilation_case(
        "hrv_bypass_summer",
        "The same unit with its summer bypass (open above 15 degC outdoor when the extract air is warmer)",
        SUMMER,
        companion="hrv_summer",
        libraries=TABLE_LIBRARIES,
        unit=models.ventilation_unit(parameters={"summer_bypass": "true"}),
    ),
    ventilation_case(
        "hrv_night_setback_winter",
        "The unit at half its flow from 22 to 7 h (fan schedule)",
        companion="hrv_winter",
        unit=models.ventilation_unit(parameters={"fan_schedule": NIGHT_SETBACK}),
    ),
    ventilation_case(
        "hrv_postheater_winter",
        "The unit with an electric post-heater holding the supply air at 18 degC",
        companion="hrv_winter",
        libraries=TABLE_LIBRARIES,
        unit=models.ventilation_unit(parameters={"supply_air_minimum_temperature": 18.0 + KELVIN}),
    ),
    ventilation_case(
        "extract_only_winter",
        "Mechanical extract ventilation (system_c), 150 m3/h: outdoor air through the trickle vents, one fan",
        companion="no_ventilation_winter",
        libraries=TABLE_LIBRARIES,
        unit=models.ventilation_unit(variant="system_c"),
    ),
    ventilation_case(
        "natural_winter",
        "Natural ventilation (system A): 150 m3/h of outdoor air through the vents and windows all day, no fan",
        companion="no_ventilation_winter",
        natural_ventilation=models.VENTILATION_FLOW,
    ),
    ventilation_case(
        "ahu_g36_winter",
        "The Guideline 36 multizone air handling unit of trano (default variant) with a VAV box: it has no coils",
        companion="no_ventilation_winter",
        libraries=TABLE_LIBRARIES,
        unit=models.guideline36_unit(),
    ),
]

# --- houses: the six-zone house with each family of systems, two weeks from the coldest day ------------------------

FORTNIGHT = 14


def house_case(
    case_id: str,
    description: str,
    generator: dict[str, Any],
    supply: float,
    libraries: tuple[str, ...] = LIBRARIES,
    **house: Any,  # noqa: ANN401
) -> Case:
    return Case(
        id=case_id,
        family=Family.house,
        description=description,
        days=FORTNIGHT,
        output_interval=900,
        libraries=libraries,
        building=models.house(generator, supply, **house),
    )


HOUSE_CASES = [
    house_case(
        "house_air_water_heat_pump",
        "Six-zone house: air-water heat pump (4 kW at -8/45 degC, 150 l buffer, heating curve), one mixing circuit "
        "on radiators rated 45/35 under proportional valves, balanced ventilation with heat recovery at half flow "
        "at night",
        models.air_water_heat_pump(power=4000.0, buffer_volume=0.15),
        45.0 + KELVIN,
        ventilation=models.ventilation_unit(flow=0.1, parameters={"fan_schedule": NIGHT_SETBACK}),
    ),
    house_case(
        "house_ground_source_heat_pump",
        "Six-zone house: ground-source heat pump (Carnot, 10 degC ground loop), the same circuit and radiators, "
        "balanced ventilation with heat recovery",
        models.air_water_heat_pump(power=4000.0, buffer_volume=0.15, variant="ground_source_heat_pump"),
        45.0 + KELVIN,
        libraries=TABLE_LIBRARIES,
        ventilation=models.ventilation_unit(flow=0.1),
    ),
    house_case(
        "house_condensing_boiler",
        "Six-zone house: condensing boiler without storage at 45 degC on the 45/35 radiators, mechanical extract "
        "ventilation (system C)",
        models.gas_boiler(power=8000.0, supply=45.0 + KELVIN, storage=False, parameters=CONDENSING_CURVE),
        45.0 + KELVIN,
        libraries=TABLE_LIBRARIES,
        ventilation=models.ventilation_unit(variant="system_c", flow=0.1),
    ),
    house_case(
        "house_boiler_storage",
        "Six-zone house: gas boiler with a 200 l storage tank at 70 degC, radiators re-rated 70/50 under on-off "
        "valves (1 % leakage so that the loop keeps a flow path when they all close), natural ventilation through "
        "the windows (system A)",
        models.gas_boiler(power=8000.0, supply=70.0 + KELVIN),
        70.0 + KELVIN,
        libraries=TABLE_LIBRARIES,
        radiator_rating=(70.0 + KELVIN, 50.0 + KELVIN),
        valve_control={"variant": "onoff"},
        valve_parameters={"valve_leakage": 0.01},
        natural_ventilation=0.1,
    ),
    house_case(
        "house_heat_pump_pv",
        "Six-zone house: the air-water heat pump of house_air_water_heat_pump with 20 m2 of photovoltaics on the "
        "roof and balanced ventilation with heat recovery",
        models.air_water_heat_pump(power=4000.0, buffer_volume=0.15),
        45.0 + KELVIN,
        ventilation=models.ventilation_unit(flow=0.1),
        photovoltaic_area=20.0,
    ),
]

CASES: dict[str, Case] = {
    case.id: case for case in [*HEAT_PUMP_CASES, *BOILER_CASES, *HYDRONIC_CASES, *VENTILATION_CASES, *HOUSE_CASES]
}


def render_case(case: Case) -> str:
    header = (
        f"Systems validation case {case.id}: {case.description}.\nGenerated by validation.systems.cases, do not edit."
    )
    return models.render(case.building, header)


def write_cases(directory: Path = CASES_DIR) -> list[Path]:
    """Write the YAML of every case; returns the files written."""
    directory.mkdir(parents=True, exist_ok=True)
    written = []
    for case in CASES.values():
        path = case_file(case.id, directory)
        path.write_text(render_case(case))
        written.append(path)
    return written
