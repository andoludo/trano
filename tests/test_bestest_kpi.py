"""Fast unit tests for validation.bestest.kpi helpers (no simulation required)."""
from datetime import timedelta

import numpy as np

from validation.bestest.kpi import (
    EPOCH,
    JOULE_TO_KWH,
    KELVIN_OFFSET,
    SECONDS_PER_DAY,
    SECONDS_PER_HOUR,
    find_min,
    find_peak,
    hourly_resample,
    integrate_signal,
)


def test_integrate_signal_constant_one_year() -> None:
    one_year_s = 365 * SECONDS_PER_DAY
    time = np.array([0.0, one_year_s], dtype=np.float64)
    values = np.array([1000.0, 1000.0], dtype=np.float64)
    integral_j = integrate_signal(time, values)
    expected_kwh = 1000.0 * one_year_s * JOULE_TO_KWH  # 1 kW * 8760 h = 8760 kWh
    assert abs(integral_j * JOULE_TO_KWH - expected_kwh) < 1e-6


def test_integrate_signal_triangle() -> None:
    time = np.array([0.0, 1.0, 2.0], dtype=np.float64)
    values = np.array([0.0, 10.0, 0.0], dtype=np.float64)
    assert integrate_signal(time, values) == 10.0  # area of triangle = 0.5*base*height; trap = 10


def test_find_peak_returns_value_and_timestamp() -> None:
    time = np.array([0.0, 60.0, 120.0, 180.0], dtype=np.float64)
    values = np.array([5.0, 7.0, 2.0, 9.0], dtype=np.float64)
    peak, ts = find_peak(time, values)
    assert peak == 9.0
    assert ts == EPOCH + timedelta(seconds=180.0)


def test_find_min_returns_value_and_timestamp() -> None:
    time = np.array([0.0, 60.0, 120.0], dtype=np.float64)
    values = np.array([5.0, 1.0, 7.0], dtype=np.float64)
    minimum, ts = find_min(time, values)
    assert minimum == 1.0
    assert ts == EPOCH + timedelta(seconds=60.0)


def test_find_peak_empty_returns_zero_at_epoch() -> None:
    empty = np.array([], dtype=np.float64)
    peak, ts = find_peak(empty, empty)
    assert peak == 0.0
    assert ts == EPOCH


def test_hourly_resample_returns_24_samples_for_target_day() -> None:
    # Build a 1-year time series with constant 293.15 K (20 C).
    time = np.linspace(0.0, 365 * SECONDS_PER_DAY, num=365 * 24 + 1, dtype=np.float64)
    values_k = np.full_like(time, fill_value=293.15)
    trace = hourly_resample(time, values_k, day_of_year=4)
    assert trace.day_of_year == 4
    assert len(trace.hours) == 24
    assert len(trace.values_c) == 24
    for v in trace.values_c:
        assert abs(v - 20.0) < 1e-6


def test_hourly_resample_picks_correct_day() -> None:
    # Monotonic linear ramp over a year: T(t) = t/SECONDS_PER_DAY (degrees, fake).
    seconds = np.arange(0, 365 * SECONDS_PER_DAY + 1, SECONDS_PER_HOUR, dtype=np.float64)
    values_k = seconds / SECONDS_PER_DAY + KELVIN_OFFSET
    trace = hourly_resample(seconds, values_k, day_of_year=4)
    # First sample at start of day 4 = 3 days into year.
    assert abs(trace.values_c[0] - 3.0) < 1e-3
    # Last sample at hour 23 of day 4 = 3 + 23/24 days.
    assert abs(trace.values_c[23] - (3.0 + 23.0 / 24.0)) < 1e-3
