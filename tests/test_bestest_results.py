"""The frozen BESTEST results of the gating libraries are consistent with the standard and with physics.

``validation/bestest/expected`` holds the KPIs each gating library gave on its last validated run; the
``bestest`` simulations are held to them. These checks need no simulation: they make sure that what was
frozen passes the acceptance bands (or is a listed deviation), behaves as a conditioned or free-floating
zone must, and reacts to each change of the standard's cases the way every reference program does.
"""

from pathlib import Path

import pytest

from validation.bestest.cases import CASES, UnsupportedCaseError, check_support
from validation.bestest.reference import KPIS, Kpi, ReferenceData, load_reference
from validation.bestest.report import KNOWN_DEVIATIONS, Comparison, acceptance_band, frozen_values
from validation.bestest.kpi import KpiResults

GATING_LIBRARIES = ("Buildings", "IDEAS", "FastModel")
SETPOINT_TOLERANCE = 0.25  # [K] the ideal system lets the hourly mean leave its set point by this much at most

# (case, base case): what the standard changes from one case to the other.
SENSITIVITIES = [
    ("610", "600"),  # south overhang
    ("620", "600"),  # east and west windows
    ("630", "620"),  # east and west overhangs and fins
    ("640", "600"),  # heating set-back
    ("650", "600"),  # night ventilation, no heating
    ("660", "600"),  # low-e argon glazing
    ("670", "600"),  # single pane glazing
    ("680", "600"),  # more insulation
    ("685", "600"),  # single set point 20 degC
    ("695", "685"),  # more insulation with the single set point
    ("900", "600"),  # heavy mass
    ("910", "900"),
    ("920", "900"),
    ("930", "920"),
    ("940", "900"),
    ("950", "900"),
    ("980", "900"),
    ("985", "900"),
    ("995", "985"),
    ("900FF", "600FF"),
    ("680FF", "600FF"),
    ("980FF", "900FF"),
    ("950FF", "900FF"),
]
# Smallest change all reference programs must show for its direction to be checked: below it the direction
# of the change is within the noise of the programs.
SIGNIFICANT_CHANGE = {"MWh": 0.05, "kW": 0.05, "degC": 0.5}


@pytest.fixture(scope="module")
def reference() -> ReferenceData:
    return load_reference()


def supported(library: str) -> set[str]:
    cases = set()
    for case in CASES.values():
        try:
            check_support(case, library)
        except UnsupportedCaseError:
            continue
        cases.add(case.id)
    return cases


def frozen_kpis(library: str, case_id: str) -> KpiResults:
    values = frozen_values(library)[case_id]
    times = {f"{kpi}_at": "" for kpi in KPIS if kpi != "mean_temperature"}
    return KpiResults(case=case_id, library=library, **values, **times)


def comparisons(library: str, reference: ReferenceData) -> list[Comparison]:
    results = []
    for case_id in frozen_values(library):
        kpis = frozen_kpis(library, case_id)
        for kpi in KPIS:
            if (kpi_reference := reference.kpi(case_id, kpi)) is None:
                continue
            lower, upper = acceptance_band(kpi_reference)
            results.append(
                Comparison(
                    case=case_id,
                    library=library,
                    kpi=kpi,
                    value=kpis.value(kpi),
                    lower=lower,
                    upper=upper,
                    mean=kpi_reference.mean,
                    unit=kpi_reference.unit,
                )
            )
    return results


@pytest.mark.parametrize("library", GATING_LIBRARIES)
def test_every_supported_case_has_frozen_results(library: str) -> None:
    assert set(frozen_values(library)) == supported(library)


@pytest.mark.parametrize("library", GATING_LIBRARIES)
def test_frozen_results_lie_in_their_acceptance_bands(library: str, reference: ReferenceData) -> None:
    failures = [
        f"{c.case} {c.kpi}: {c.value:.3f} {c.unit} outside [{c.lower:.3f}, {c.upper:.3f}]"
        for c in comparisons(library, reference)
        if c.status == "fail"
    ]

    assert not failures, "\n".join(failures)


def test_every_known_deviation_is_still_outside_its_band(reference: ReferenceData) -> None:
    by_key = {(c.library, c.case, c.kpi): c for library in GATING_LIBRARIES for c in comparisons(library, reference)}

    for key in KNOWN_DEVIATIONS:
        assert key in by_key, f"{key} has no frozen result"
        assert by_key[key].status == "known", f"{key} is back in its band: remove it from KNOWN_DEVIATIONS"


@pytest.mark.parametrize("library", GATING_LIBRARIES)
def test_free_floating_zones_have_no_load(library: str) -> None:
    for case_id, values in frozen_values(library).items():
        if CASES[case_id].free_float:
            loads = {kpi: values[kpi] for kpi in KPIS[:4]}
            assert loads == dict.fromkeys(KPIS[:4], 0.0), case_id


@pytest.mark.parametrize("library", GATING_LIBRARIES)
def test_conditioned_zones_stay_between_their_set_points(library: str) -> None:
    for case_id, values in frozen_values(library).items():
        case = CASES[case_id]
        if case.hvac is None or case.sunspace:  # case 960 reports the free-floating sun-space
            continue
        lowest = case.hvac.heating_setpoint if case.hvac.heating_setback is None else case.hvac.heating_setback
        if lowest is not None:
            assert values["minimum_temperature"] >= lowest - SETPOINT_TOLERANCE, case_id
        else:
            assert values["annual_heating"] == values["peak_heating"] == 0.0, case_id
        if case.hvac.cooling_hours is None and case.hvac.cooling_setpoint is not None:
            assert values["maximum_temperature"] <= case.hvac.cooling_setpoint + SETPOINT_TOLERANCE, case_id
        assert values["annual_cooling"] > 0 and values["peak_cooling"] > 0, case_id


@pytest.mark.parametrize("library", GATING_LIBRARIES)
@pytest.mark.parametrize(("case_id", "free_floating"), [("650", "650FF"), ("950", "950FF")])
def test_night_ventilated_zones_without_heating_are_as_cold_as_their_free_floating_twin(
    library: str, case_id: str, free_floating: str
) -> None:
    """Without heating, the coldest hour of a winter night is not touched by the day-time cooling."""
    coldest = {name: frozen_values(library)[name]["minimum_temperature"] for name in (case_id, free_floating)}

    assert coldest[case_id] == pytest.approx(coldest[free_floating], abs=0.1)


@pytest.mark.parametrize("library", GATING_LIBRARIES)
def test_thermal_mass_damps_the_free_floating_swing(library: str) -> None:
    values = frozen_values(library)
    for light, heavy in (("600FF", "900FF"), ("650FF", "950FF"), ("680FF", "980FF")):
        swing = {case_id: values[case_id]["maximum_temperature"] - values[case_id]["minimum_temperature"]
                 for case_id in (light, heavy)}  # fmt: skip
        assert swing[heavy] < swing[light] / 1.5, (light, heavy, swing)


def reference_changes(reference: ReferenceData, case_id: str, base: str, kpi: Kpi) -> list[float]:
    case, base_case = reference.kpi(case_id, kpi), reference.kpi(base, kpi)
    if case is None or base_case is None:
        return []
    programs = set(case.programs) & set(base_case.programs)
    return [case.programs[program].value - base_case.programs[program].value for program in sorted(programs)]


@pytest.mark.parametrize("library", GATING_LIBRARIES)
def test_each_change_of_the_cases_acts_in_the_direction_of_every_reference_program(
    library: str, reference: ReferenceData
) -> None:
    values = frozen_values(library)
    checked, wrong = 0, []
    for case_id, base in SENSITIVITIES:
        if case_id not in values or base not in values:
            continue
        for kpi in KPIS:
            changes = reference_changes(reference, case_id, base, kpi)
            threshold = SIGNIFICANT_CHANGE[reference.kpi(case_id, kpi).unit] if changes else 0.0  # type: ignore[union-attr]
            if not changes or not (all(c > threshold for c in changes) or all(c < -threshold for c in changes)):
                continue
            checked += 1
            change = values[case_id][kpi] - values[base][kpi]
            if change * changes[0] <= 0:
                programs = f"programs {min(changes):+.3f} to {max(changes):+.3f}"
                wrong.append(f"{case_id} - {base} {kpi}: {change:+.3f}, {programs}")

    assert checked > 50
    assert not wrong, "\n".join(wrong)


def test_the_section_of_a_library_replaces_its_previous_version(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """The FastModel section is written into the page without the results of the other libraries at hand."""
    from validation.bestest import report

    page = tmp_path / "bestest.md"
    page.write_text("# BESTEST\n\n## Buildings\n\nBuildings table\n\n## IDEAS\n\nIDEAS table\n")
    monkeypatch.setattr(report, "DOCS_PAGE", page)
    monkeypatch.setattr(
        report, "render_library_section", lambda library, results, reference=None: f"## {library}\n\n{results}\n"
    )
    report.update_docs_section("FastModel", {"first": 1})  # type: ignore[dict-item]
    report.update_docs_section("FastModel", {"second": 2})  # type: ignore[dict-item]
    text = page.read_text()
    assert text.count("## FastModel") == 1
    assert "second" in text and "first" not in text
    assert "Buildings table" in text and "IDEAS table" in text
    assert text.index("## IDEAS") < text.index("## FastModel")
