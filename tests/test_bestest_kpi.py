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


def test_hourly_increments_of_a_cumulative_signal_are_the_mean_rates() -> None:
    from validation.bestest.kpi import HOURS_PER_YEAR, SECONDS_PER_HOUR, hourly_increments

    time = np.array([0.0, 1800.0, 3600.0, HOURS_PER_YEAR * SECONDS_PER_HOUR])
    energy = np.array([0.0, 3600.0 * 1000, 3600.0 * 1000, 3600.0 * 1000 + 2 * 3600.0 * 500 * (HOURS_PER_YEAR - 1)])

    rates = hourly_increments(time, energy)

    assert rates[0] == pytest.approx(1000)  # 1 kW on average over the first hour
    assert rates[1] == pytest.approx(1000) and rates[-1] == pytest.approx(1000)
    assert len(rates) == HOURS_PER_YEAR


def test_known_deviations_are_reported_but_not_failures() -> None:
    from validation.bestest.report import KNOWN_DEVIATIONS, Comparison

    (library, case, kpi), reason = next(iter(KNOWN_DEVIATIONS.items()))
    outside = Comparison(case=case, library=library, kpi=kpi, value=0.0, lower=1.0, upper=2.0, mean=1.5, unit="MWh")
    unknown = outside.model_copy(update={"case": "no-such-case"})

    assert outside.status == "known" and outside.known_deviation == reason
    assert unknown.status == "fail" and unknown.known_deviation is None
    assert outside.model_copy(update={"value": 1.5}).status == "pass"


def test_the_model_hash_ignores_layout_and_the_order_of_records() -> None:
    from trano.simulate.simulate import SimulationOptions
    from validation.bestest.harness import model_hash

    options = SimulationOptions(end_time=3600)
    records = ["record a = X(k=1);", "record b = X(k=2);"]
    one = "package p\n" + "\n".join(records) + "\nmodel m\n  A a annotation (Placement(x=1));\nend m;\nend p;"
    other = "package p\n" + "\n".join(records[::-1]) + "\nmodel m\n  A a annotation (Placement(x=99));\nend m;\nend p;"
    changed = other.replace("k=2", "k=3")

    assert model_hash(one, "Buildings", options) == model_hash(other, "Buildings", options)
    assert model_hash(one, "Buildings", options) != model_hash(changed, "Buildings", options)
    assert model_hash(one, "Buildings", options) != model_hash(one, "IDEAS", options)
