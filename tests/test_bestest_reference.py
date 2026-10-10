"""The ASHRAE 140-2020 reference data is read completely and faithfully."""

import pytest

from validation.bestest.reference import KPIS, ReferenceData, load_reference
from validation.bestest.report import acceptance_band

PROGRAMS = {"BSIMAC", "CSE", "DeST", "EnergyPlus", "ESP-r", "TRNSYS"}


@pytest.fixture(scope="module")
def reference() -> ReferenceData:
    return load_reference()


def test_every_case_of_the_standard_is_read(reference: ReferenceData) -> None:
    loads = {"600", "610", "620", "630", "640", "650", "660", "670", "680", "685", "695"}
    loads |= {"900", "910", "920", "930", "940", "950", "960", "980", "985", "995"}
    free_float = {"600FF", "650FF", "680FF", "900FF", "950FF", "980FF", "960"}

    assert {kpi.case for kpi in reference.kpis if kpi.kpi == "annual_heating"} == loads
    assert {kpi.case for kpi in reference.kpis if kpi.kpi == "maximum_temperature"} == free_float
    assert len(reference.kpis) == 4 * len(loads) + 3 * len(free_float)


def test_annual_loads_carry_the_acceptance_limits(reference: ReferenceData) -> None:
    heating = reference.kpi("600", "annual_heating")

    assert heating is not None
    assert heating.programs["CSE"].value == 3.993
    assert set(heating.programs) == PROGRAMS
    assert (heating.lower_limit, heating.upper_limit) == (3.75, 4.98)
    assert acceptance_band(heating) == (3.75, 4.98)
    # Case 910 lists its cooling limits the other way round; the band is still ordered.
    assert acceptance_band(reference.kpi("910", "annual_cooling")) == (0.86, 2.0)  # type: ignore[arg-type]


def test_peaks_and_extremes_carry_their_time_of_occurrence(reference: ReferenceData) -> None:
    peak = reference.kpi("600", "peak_heating")
    coldest = reference.kpi("600FF", "minimum_temperature")

    assert peak is not None and peak.programs["BSIMAC"].model_dump() == {"value": 3.255, "at": "26-Nov:8"}
    assert peak.lower_limit is None
    assert coldest is not None and coldest.minimum == -13.8 and coldest.maximum == -9.9
    assert acceptance_band(coldest) == (-14.8, -8.9)


def test_missing_results_are_left_out(reference: ReferenceData) -> None:
    sunspace = reference.kpi("960", "annual_heating")

    assert sunspace is not None and "BSIMAC" not in sunspace.programs
    assert sunspace.mean == pytest.approx((2.522 + 2.771 + 2.689 + 2.624 + 2.860) / 5)


def test_hourly_data_and_bins(reference: ReferenceData) -> None:
    assert [(h.case, h.quantity, h.month, h.day) for h in reference.hourly] == [
        ("600FF", "temperature", 2, 1),
        ("900FF", "temperature", 2, 1),
        ("650FF", "temperature", 7, 14),
        ("950FF", "temperature", 7, 14),
        ("600", "load", 2, 1),
        ("900", "load", 2, 1),
    ]
    feb_1 = reference.hourly_of("600FF")[0]
    assert len(feb_1.programs["EnergyPlus"]) == 24 and feb_1.programs["EnergyPlus"][15] == 47.6
    (bins,) = reference.bins
    assert bins.case == "900FF" and bins.temperatures[0] == -50 and bins.temperatures[-1] == 98
    assert all(sum(hours) == 8760 for hours in bins.programs.values())


def test_every_kpi_has_a_unit(reference: ReferenceData) -> None:
    assert {kpi.unit for kpi in reference.kpis if kpi.kpi in KPIS[:2]} == {"MWh"}
    assert {kpi.unit for kpi in reference.kpis if kpi.kpi in KPIS[2:4]} == {"kW"}
    assert {kpi.unit for kpi in reference.kpis if kpi.kpi.endswith("temperature")} == {"degC"}
