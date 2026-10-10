"""Key performance indicators of ASHRAE 140 computed from a simulation result.

The standard reports hourly integrated quantities: loads are hourly averages of the heating or
cooling power (a kWh per hour), extremes and means of the zone temperature are taken over hourly
averages. Every signal is therefore first averaged over the 8760 hours of the year.
"""

from __future__ import annotations

import datetime
from pathlib import Path

import numpy as np
import numpy.typing as npt
from buildingspy.io.outputfile import Reader  # type: ignore
from pydantic import BaseModel, Field

from validation.bestest.reference import Kpi

FloatArray = npt.NDArray[np.float64]
HOURS_PER_YEAR = 8760
SECONDS_PER_HOUR = 3600.0
KELVIN = 273.15
BIN_TEMPERATURES = list(range(-50, 99))  # [degC] bins of the standard's temperature histogram
_YEAR = 2001  # a non-leap year, only used to name the days


class Signals(BaseModel):
    """Result variables of a zone: full names in the result file."""

    temperature: str
    heating_power: str | None = None  # [W], positive when heating
    cooling_power: str | None = None  # [W], positive when cooling


class HourlyTrace(BaseModel):
    month: int
    day: int
    values: list[float]  # 24 hourly values, hour 1 first


class KpiResults(BaseModel):
    case: str
    library: str
    annual_heating: float = Field(description="MWh")
    annual_cooling: float = Field(description="MWh")
    peak_heating: float = Field(description="kW")
    peak_heating_at: str
    peak_cooling: float = Field(description="kW")
    peak_cooling_at: str
    maximum_temperature: float = Field(description="degC")
    maximum_temperature_at: str
    minimum_temperature: float = Field(description="degC")
    minimum_temperature_at: str
    mean_temperature: float = Field(description="degC")
    temperature_traces: list[HourlyTrace] = Field(default_factory=list)
    load_traces: list[HourlyTrace] = Field(default_factory=list, description="kWh, heating positive")
    temperature_bins: list[int] = Field(default_factory=list, description="hours per bin of BIN_TEMPERATURES")

    def value(self, kpi: Kpi) -> float:
        return float(getattr(self, kpi))


def hourly_means(time: FloatArray, values: FloatArray) -> FloatArray:
    """Average of a piecewise linear signal over each of the 8760 hours of the year."""
    if time[0] > 0 or time[-1] < HOURS_PER_YEAR * SECONDS_PER_HOUR - 1e-6:
        raise ValueError("The simulation must cover the whole year.")
    edges = np.arange(HOURS_PER_YEAR + 1) * SECONDS_PER_HOUR
    # Trapezoids on the samples and the hour edges together: exact for a piecewise linear signal.
    grid = np.union1d(time, edges)
    on_grid = np.interp(grid, time, values)
    cumulative = np.concatenate([[0.0], np.cumsum(np.diff(grid) * (on_grid[1:] + on_grid[:-1]) / 2)])
    return np.diff(np.interp(edges, grid, cumulative)) / SECONDS_PER_HOUR  # type: ignore[no-any-return]


def timestamp(hour: int) -> str:
    """Day and hour of the year in the notation of the standard: ``26-Nov:8`` is hour 8 of November 26."""
    day = datetime.date(_YEAR, 1, 1) + datetime.timedelta(days=hour // 24)
    return f"{day.day:02d}-{day.strftime('%b')}:{hour % 24 + 1}"


def day_trace(hourly: FloatArray, month: int, day: int) -> HourlyTrace:
    start = (datetime.date(_YEAR, month, day) - datetime.date(_YEAR, 1, 1)).days * 24
    return HourlyTrace(month=month, day=day, values=[float(value) for value in hourly[start : start + 24]])


def temperature_bins(hourly_celsius: FloatArray) -> list[int]:
    """Hours spent in each 1 K bin, a bin holding the temperatures rounded down to its value."""
    edges = np.array([*BIN_TEMPERATURES, BIN_TEMPERATURES[-1] + 1], dtype=float)
    counts, _ = np.histogram(hourly_celsius, bins=edges)
    return [int(count) for count in counts]


def _signal(reader: Reader, name: str) -> tuple[FloatArray, FloatArray]:
    time, values = reader.values(name)
    return np.asarray(time, dtype=float), np.asarray(values, dtype=float)


def _hourly(reader: Reader, name: str | None) -> FloatArray:
    if name is None:
        return np.zeros(HOURS_PER_YEAR)
    return hourly_means(*_signal(reader, name))


def extract_kpis(
    result_file: Path,
    case: str,
    library: str,
    signals: Signals,
    trace_days: list[tuple[int, int]],
) -> KpiResults:
    reader = Reader(str(result_file), "dymola")
    temperature = _hourly(reader, signals.temperature) - KELVIN
    heating = np.maximum(_hourly(reader, signals.heating_power), 0.0)
    cooling = np.maximum(_hourly(reader, signals.cooling_power), 0.0)
    load = (heating - cooling) / 1000  # [kWh] per hour, heating positive as in the standard's tables
    peak_heating, peak_cooling = int(np.argmax(heating)), int(np.argmax(cooling))
    hottest, coldest = int(np.argmax(temperature)), int(np.argmin(temperature))
    return KpiResults(
        case=case,
        library=library,
        annual_heating=float(heating.sum()) / 1e6,
        annual_cooling=float(cooling.sum()) / 1e6,
        peak_heating=float(heating[peak_heating]) / 1000,
        peak_heating_at=timestamp(peak_heating),
        peak_cooling=float(cooling[peak_cooling]) / 1000,
        peak_cooling_at=timestamp(peak_cooling),
        maximum_temperature=float(temperature[hottest]),
        maximum_temperature_at=timestamp(hottest),
        minimum_temperature=float(temperature[coldest]),
        minimum_temperature_at=timestamp(coldest),
        mean_temperature=float(temperature.mean()),
        temperature_traces=[day_trace(temperature, month, day) for month, day in trace_days],
        load_traces=[day_trace(load, month, day) for month, day in trace_days],
        temperature_bins=temperature_bins(temperature),
    )


def find_variable(reader: Reader, suffix: str) -> str:
    """The single result variable ending with ``suffix`` (the model nests components in containers)."""
    matches = [name for name in reader.varNames() if name == suffix or name.endswith("." + suffix)]
    if len(matches) != 1:
        raise ValueError(f"Expected one variable ending with {suffix!r}, found {matches}")
    return str(matches[0])
