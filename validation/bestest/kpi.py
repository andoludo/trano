"""Extract BESTEST KPIs from an OpenModelica ``.mat`` result file.

Reads zone air temperature and (optionally) ideal-heater/cooler power signals,
integrates to annual energy, finds peak power, and resamples zone temperature
to hourly values for the two reporting days specified per case.

The Reader from buildingspy.io.outputfile is already a Trano dependency (used in
trano/plot/plot.py and trano/reporting/utils.py); no new deps required.
"""
from __future__ import annotations

from datetime import datetime, timedelta
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np
from buildingspy.io.outputfile import Reader  # type: ignore[import-untyped]

from validation.bestest.spec.parameters import HourlyTrace, KPIResults

if TYPE_CHECKING:
    from numpy.typing import NDArray

KELVIN_OFFSET: float = 273.15
JOULE_TO_KWH: float = 1.0 / 3_600_000.0
SECONDS_PER_DAY: int = 86_400
SECONDS_PER_HOUR: int = 3_600
EPOCH: datetime = datetime(2024, 1, 1)


def open_mat(mat_path: Path) -> Reader:
    return Reader(str(mat_path), "openmodelica")


def _get_signal(reader: Reader, name: str) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    time, values = reader.values(name)
    return np.asarray(time, dtype=np.float64), np.asarray(values, dtype=np.float64)


def _get_zone_temperature(
    reader: Reader, space_name: str
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Read the zone air temperature, tolerant to wrapper class naming.

    Trano wraps every space inside a ``building.envelope1.<space_name>``
    container, so the OMC result file holds the zone signals at
    ``building.envelope1.<space_name>.air.{vol,heaPorAir}.T``. The exact
    prefix depends on the generated wrapper class; search the result file
    by suffix to stay tolerant. If nothing matches, raise with the
    available ``space_name``-prefixed variables so the right path is
    obvious from CI logs.
    """
    suffixes = [
        f"{space_name}.air.vol.T",
        f"{space_name}.air.heaPorAir.T",
        f"{space_name}.heaPorAir.T",
        f"{space_name}.vol.T",
    ]
    try:
        all_names = list(reader.varNames())
    except AttributeError:
        all_names = []
    for suffix in suffixes:
        match = next((n for n in all_names if n.endswith(suffix)), None)
        if match is not None:
            return _get_signal(reader, match)
    matches = [n for n in all_names if space_name in n and n.endswith(".T")]
    raise KeyError(
        f"zone temperature not found for {space_name!r}; tried suffixes {suffixes}. "
        f"Result file contains {len(matches)} matching .T signals: {matches[:30]}"
    )


def integrate_signal(time_s: NDArray[np.float64], values: NDArray[np.float64]) -> float:
    """Trapezoidal integral over the full time series, in the units of values·seconds."""
    return float(np.trapezoid(values, time_s))


def find_peak(time_s: NDArray[np.float64], values: NDArray[np.float64]) -> tuple[float, datetime]:
    if values.size == 0:
        return 0.0, EPOCH
    idx = int(np.argmax(values))
    return float(values[idx]), EPOCH + timedelta(seconds=float(time_s[idx]))


def find_min(time_s: NDArray[np.float64], values: NDArray[np.float64]) -> tuple[float, datetime]:
    if values.size == 0:
        return 0.0, EPOCH
    idx = int(np.argmin(values))
    return float(values[idx]), EPOCH + timedelta(seconds=float(time_s[idx]))


def hourly_resample(
    time_s: NDArray[np.float64],
    values: NDArray[np.float64],
    day_of_year: int,
) -> HourlyTrace:
    """Linear-interpolate to 24 hourly samples for the given day-of-year."""
    start = (day_of_year - 1) * SECONDS_PER_DAY
    sample_times = np.array([start + h * SECONDS_PER_HOUR for h in range(24)], dtype=np.float64)
    sampled = np.interp(sample_times, time_s, values)
    return HourlyTrace(
        day_of_year=day_of_year,
        hours=[float(h) for h in range(24)],
        values_c=[float(v - KELVIN_OFFSET) for v in sampled],
    )


def _sum_signals(reader: Reader, names: list[str]) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    if not names:
        return np.array([0.0, 1.0]), np.array([0.0, 0.0])
    time, total = _get_signal(reader, names[0])
    for name in names[1:]:
        _, signal = _get_signal(reader, name)
        total = total + signal
    return time, total


def extract_kpis(  # noqa: PLR0913
    mat_path: Path,
    *,
    case_id: str,
    space_name: str,
    floor_area_m2: float,
    heater_names: list[str],
    cooler_names: list[str],
    report_days: tuple[int, int],
    sim_wall_time_s: float,
    cache_key: str,
) -> KPIResults:
    """Compute the full KPIResults bundle from a Modelica ``_res.mat`` file.

    Heater/cooler signals are read from ``<name>.HeatingPower.y``. Cooler signals
    are expected to be negative (i.e. ``power = -|P|`` on an ideal radiator) but
    iteration 1 leaves the cooling side disabled in the YAMLs.
    """
    reader = open_mat(mat_path)

    time_zone, t_zone_k = _get_zone_temperature(reader, space_name)
    t_zone_c = t_zone_k - KELVIN_OFFSET

    heater_time, heater_signal = _sum_signals(reader, [f"{n}.HeatingPower.y" for n in heater_names])
    heater_only = np.where(heater_signal > 0, heater_signal, 0.0)
    annual_heating_j = integrate_signal(heater_time, heater_only) if heater_names else 0.0
    peak_heating_w, peak_heating_at = find_peak(heater_time, heater_only) if heater_names else (0.0, EPOCH)

    cooler_time, cooler_signal = _sum_signals(reader, [f"{n}.HeatingPower.y" for n in cooler_names])
    cooler_only = np.where(cooler_signal < 0, -cooler_signal, 0.0)
    annual_cooling_j = integrate_signal(cooler_time, cooler_only) if cooler_names else 0.0
    peak_cooling_w, peak_cooling_at = find_peak(cooler_time, cooler_only) if cooler_names else (0.0, EPOCH)

    t_min_c, t_min_at = find_min(time_zone, t_zone_c)
    t_max_c, t_max_at = find_peak(time_zone, t_zone_c)

    return KPIResults(
        case_id=case_id,
        annual_heating_kwh_m2=annual_heating_j * JOULE_TO_KWH / floor_area_m2,
        annual_cooling_kwh_m2=annual_cooling_j * JOULE_TO_KWH / floor_area_m2,
        peak_heating_w=peak_heating_w,
        peak_heating_at=peak_heating_at,
        peak_cooling_w=peak_cooling_w,
        peak_cooling_at=peak_cooling_at,
        t_zone_min_c=t_min_c,
        t_zone_min_at=t_min_at,
        t_zone_max_c=t_max_c,
        t_zone_max_at=t_max_at,
        hourly_traces=[
            hourly_resample(time_zone, t_zone_k, day) for day in report_days
        ],
        sim_wall_time_s=sim_wall_time_s,
        cache_key=cache_key,
    )
