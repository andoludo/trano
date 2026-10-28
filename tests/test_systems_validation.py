"""The systems validation (validation/systems): its cases, its KPI arithmetic, and the gate on the simulated KPIs.

The gate (``pytest -m systems``) simulates a week per case and library, in Docker or with the local ``omc``
(``TRANO_SIMULATION_RUNTIME=local``), caching the results in ``.cache/systems`` by the generated model.
"""

from __future__ import annotations

import warnings

import numpy as np
import pytest

from trano.data_models.conversion import convert_network
from trano.elements.library.library import Library
from validation.bestest.harness import reset_element_names
from validation.systems.cases import CASES, case_file, render_case
from validation.systems.kpi import KELVIN, UNITS, Series, movements, occupied, runs, table_cop
from validation.systems.report import Band, load_expected


def test_every_case_file_is_up_to_date() -> None:
    """The YAML files are generated: `python -m validation.systems generate` after a change of the cases."""
    for case in CASES.values():
        assert case_file(case.id).read_text() == render_case(case), (
            f"{case.id}: run `python -m validation.systems generate`"
        )


@pytest.mark.parametrize("case_id", list(CASES))
def test_every_case_converts_with_its_libraries_without_hydronic_warnings(case_id: str) -> None:
    case = CASES[case_id]
    for library in case.libraries:
        reset_element_names()
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            network = convert_network(case.id, case.path, library=Library.from_configuration(library))
            assert network.model()
        layout = [str(w.message) for w in caught if "WRONG HYDRONIC LAYOUT" in str(w.message)]
        assert not layout, layout


def test_the_expected_bands_name_known_cases_and_kpis() -> None:
    expected = load_expected()
    assert expected, "expected.yaml holds no band"
    for case_id, bands in expected.items():
        assert case_id in CASES, case_id
        for band in bands:
            assert isinstance(band, Band)
            assert band.kpi in UNITS, f"{case_id}: unknown KPI {band.kpi}"
            assert band.min is not None or band.max is not None, f"{case_id}: {band.kpi} has no limit"
            assert band.why, f"{case_id}: {band.kpi} has no reasoning"


def test_runs_counts_starts_and_durations() -> None:
    time = np.arange(0, 3600, 60, dtype=float)
    running = np.zeros_like(time, dtype=bool)
    running[10:20] = True  # a 10 minute run
    running[40:45] = True  # a 5 minute run after a 20 minute stop
    starts, run_durations, stop_durations = runs(time, running)
    assert starts == 2
    assert run_durations == [600.0, 300.0]
    assert stop_durations == [1200.0]


def test_movements_ignores_changes_inside_the_dead_band() -> None:
    assert movements(np.array([0.0, 0.5, 1.0, 0.5, 0.0])) == 1
    assert movements(np.array([0.0, 0.5, 0.505, 0.5, 1.0])) == 0  # the wiggle is below the dead band
    assert movements(np.array([0.2, 0.3, 0.2, 0.3, 0.2])) == 3


def test_series_statistics_are_time_weighted() -> None:
    series = Series(np.array([0.0, 3600.0, 7200.0]), np.array([1000.0, 1000.0, 0.0]))
    assert series.mean() == pytest.approx(750.0)
    assert series.integral_kwh() == pytest.approx(1.5)
    assert series.hours_where(series.values > 500) == pytest.approx(2.0)
    assert series.hourly() == pytest.approx([1000.0, 500.0])


def test_occupied_hours_follow_the_schedule_of_the_cases() -> None:
    time = np.array([6.5, 7.0, 12.0, 21.9, 22.0, 23.0]) * 3600
    assert occupied(time).tolist() == [False, True, True, True, False, False]


def test_table_cop_reproduces_the_en_14511_points() -> None:
    """A7/W35 of the Vitocal 251-A08 data: 8000 W for 1633 W (COP 4.9); A-7/W35: 6470 W for 2396 W (COP 2.7)."""
    assert table_cop(35.0 + KELVIN, 7.0 + KELVIN) == pytest.approx(8000 / 1633)
    assert table_cop(35.0 + KELVIN, -7.0 + KELVIN) == pytest.approx(6470 / 2396)
    # the COP falls with the condenser temperature and rises with the source temperature
    assert (
        table_cop(55.0 + KELVIN, 7.0 + KELVIN)
        < table_cop(45.0 + KELVIN, 7.0 + KELVIN)
        < table_cop(35.0 + KELVIN, 7.0 + KELVIN)
    )
    assert (
        table_cop(45.0 + KELVIN, -10.0 + KELVIN)
        < table_cop(45.0 + KELVIN, 2.0 + KELVIN)
        < table_cop(45.0 + KELVIN, 12.0 + KELVIN)
    )


def gate_parameters() -> list[pytest.param]:  # type: ignore[valid-type]
    return [
        pytest.param(case.id, library, id=f"{case.id}-{library}")
        for case in CASES.values()
        for library in case.libraries
    ]


@pytest.mark.systems
@pytest.mark.parametrize(("case_id", "library"), gate_parameters())
def test_case_kpis_are_inside_their_bands(case_id: str, library: str) -> None:
    from validation.systems.harness import run_case
    from validation.systems.report import failures

    result = run_case(case_id, library)
    assert not (failed := failures(result)), "\n".join(failed)
