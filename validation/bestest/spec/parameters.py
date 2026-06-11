"""Typed BESTEST case parameters, KPI definitions, and the CASES registry.

This module is the single source of truth for which cases the suite covers
and what HVAC / envelope / schedule deltas distinguish them. ``CASES`` is
imported by the harness, the pytest entrypoint, and the CLI.
"""
from datetime import datetime
from pathlib import Path
from typing import Literal

from pydantic import BaseModel, Field

KPIName = Literal[
    "annual_heating_kwh_m2",
    "annual_cooling_kwh_m2",
    "peak_heating_w",
    "peak_cooling_w",
    "t_zone_min_c",
    "t_zone_max_c",
]

CaseId = Literal[
    "600", "600FF", "620", "640", "650",
    "900", "900FF", "920", "940", "950",
]

REFERENCE_CSV: Path = Path(__file__).parent / "reference.csv"


class CaseParameters(BaseModel):
    """Per-case definition consumed by the YAML merge harness."""

    case_id: CaseId
    base: Literal["lightweight", "heavyweight"]
    free_float: bool = False
    has_heating: bool = True
    has_cooling: bool = True
    heating_setpoint_c: float | None = 20.0
    cooling_setpoint_c: float | None = 27.0
    heating_setback_c: float | None = None
    setback_schedule: tuple[int, int] | None = None
    night_ventilation_ach: float | None = None
    night_ventilation_schedule: tuple[int, int] | None = None
    window_orientation: Literal["south", "east_west"] = "south"
    report_days_doy: tuple[int, int] = (4, 218)


class KPIBands(BaseModel):
    """Reference acceptance band for a single KPI of a single case."""

    case_id: str
    kpi: KPIName
    ref_min: float
    ref_max: float
    ref_mean: float
    units: str
    source: str = Field(default="ASHRAE 140-2017 Annex B")


class HourlyTrace(BaseModel):
    """Hourly zone-air-temperature trace for a single reporting day."""

    day_of_year: int
    hours: list[float]
    values_c: list[float]


class KPIResults(BaseModel):
    """Computed KPIs for a single simulated case."""

    case_id: str
    annual_heating_kwh_m2: float
    annual_cooling_kwh_m2: float
    peak_heating_w: float
    peak_heating_at: datetime
    peak_cooling_w: float
    peak_cooling_at: datetime
    t_zone_min_c: float | None = None
    t_zone_min_at: datetime | None = None
    t_zone_max_c: float | None = None
    t_zone_max_at: datetime | None = None
    hourly_traces: list[HourlyTrace] = Field(default_factory=list)
    sim_wall_time_s: float
    cache_key: str


CASES: dict[str, CaseParameters] = {
    "600": CaseParameters(case_id="600", base="lightweight"),
    "600FF": CaseParameters(
        case_id="600FF",
        base="lightweight",
        free_float=True,
        has_heating=False,
        has_cooling=False,
        heating_setpoint_c=None,
        cooling_setpoint_c=None,
    ),
    "620": CaseParameters(
        case_id="620", base="lightweight", window_orientation="east_west"
    ),
    "640": CaseParameters(
        case_id="640",
        base="lightweight",
        heating_setback_c=10.0,
        setback_schedule=(23, 7),
    ),
    "650": CaseParameters(
        case_id="650",
        base="lightweight",
        has_heating=False,
        heating_setpoint_c=None,
        night_ventilation_ach=13.14,  # ASHRAE 140 §5.2.1.6: 1703.16 m^3/h vent fan / 129.6 m^3 zone vol
        night_ventilation_schedule=(18, 7),
    ),
    "900": CaseParameters(case_id="900", base="heavyweight"),
    "900FF": CaseParameters(
        case_id="900FF",
        base="heavyweight",
        free_float=True,
        has_heating=False,
        has_cooling=False,
        heating_setpoint_c=None,
        cooling_setpoint_c=None,
    ),
    "920": CaseParameters(
        case_id="920", base="heavyweight", window_orientation="east_west"
    ),
    "940": CaseParameters(
        case_id="940",
        base="heavyweight",
        heating_setback_c=10.0,
        setback_schedule=(23, 7),
    ),
    "950": CaseParameters(
        case_id="950",
        base="heavyweight",
        has_heating=False,
        heating_setpoint_c=None,
        night_ventilation_ach=13.14,  # ASHRAE 140 §5.2.1.6: 1703.16 m^3/h vent fan / 129.6 m^3 zone vol
        night_ventilation_schedule=(18, 7),
    ),
}
