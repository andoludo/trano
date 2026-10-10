"""KPIs follow the hourly-integrated definitions of ASHRAE 140."""

import numpy as np
import pytest

from validation.bestest.kpi import (
    HOURS_PER_YEAR,
    SECONDS_PER_HOUR,
    day_trace,
    hourly_means,
    temperature_bins,
    timestamp,
)

YEAR = HOURS_PER_YEAR * SECONDS_PER_HOUR


def test_hourly_means_of_a_constant_signal_sampled_irregularly() -> None:
    time = np.array([0.0, 100.0, 5000.0, YEAR])

    assert hourly_means(time, np.full(4, 3.0)) == pytest.approx(np.full(HOURS_PER_YEAR, 3.0))


def test_hourly_means_integrate_a_ramp_exactly() -> None:
    time = np.array([0.0, YEAR])
    hourly = hourly_means(time, np.array([0.0, YEAR]))  # value equals time: the mean of hour i is (i + 0.5) h

    assert hourly[0] == pytest.approx(0.5 * SECONDS_PER_HOUR)
    assert hourly[-1] == pytest.approx((HOURS_PER_YEAR - 0.5) * SECONDS_PER_HOUR)


def test_hourly_means_need_the_whole_year() -> None:
    with pytest.raises(ValueError, match="whole year"):
        hourly_means(np.array([0.0, YEAR / 2]), np.array([1.0, 1.0]))


def test_timestamps_use_the_notation_of_the_standard() -> None:
    assert timestamp(0) == "01-Jan:1"
    assert timestamp(HOURS_PER_YEAR - 1) == "31-Dec:24"
    assert timestamp((31 + 8) * 24 + 6) == "09-Feb:7"


def test_day_trace_picks_the_24_hours_of_the_day() -> None:
    hourly = np.arange(HOURS_PER_YEAR, dtype=float)
    trace = day_trace(hourly, 2, 1)

    assert (trace.month, trace.day) == (2, 1)
    assert trace.values == [float(31 * 24 + hour) for hour in range(24)]


def test_temperature_bins_count_hours_per_degree() -> None:
    bins = temperature_bins(np.array([-50.0, 20.4, 20.9, 98.5] + [21.0] * (HOURS_PER_YEAR - 4)))

    assert sum(bins) == HOURS_PER_YEAR
    assert bins[0] == 1 and bins[20 + 50] == 2 and bins[21 + 50] == HOURS_PER_YEAR - 4 and bins[-1] == 1
