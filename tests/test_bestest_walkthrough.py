"""The walk-through page is rendered from the case files, the reference data and the results."""

import pytest

from validation.bestest.cases import CASES
from validation.bestest.harness import CaseResult
from validation.bestest.kpi import HourlyTrace, KpiResults
from validation.bestest.reference import load_reference
from validation.bestest.walkthrough import (
    FEATURED_CASES,
    PROGRAM_COLOR,
    Series,
    case_yaml_sections,
    day_chart,
    render_walkthrough,
    results_summary,
    results_table,
    svg_day_chart,
    yaml_fragment,
)

DOCUMENT = {
    "weather": {"parameters": {"path": "denver.mos"}},
    "constructions": [{"id": "LIGHT_WALL:001", "layers": []}, {"id": "ROOF:001", "layers": [{"thickness": 0.1}]}],
    "spaces": [{"id": "ZONE:001", "parameters": {"ach": 0.414}}],
}


def _result(case_id: str, library: str, **kpis: float) -> CaseResult:
    values = {
        "annual_heating": 4.4,
        "annual_cooling": 5.9,
        "peak_heating": 3.2,
        "peak_cooling": 6.1,
        "maximum_temperature": 27.0,
        "minimum_temperature": 20.0,
        "mean_temperature": 23.0,
    } | kpis
    return CaseResult(
        kpis=KpiResults(
            case=case_id,
            library=library,
            peak_heating_at="01-Jan:1",
            peak_cooling_at="01-Jan:1",
            maximum_temperature_at="01-Jan:1",
            minimum_temperature_at="01-Jan:1",
            load_traces=[HourlyTrace(month=2, day=1, values=[2.0] * 24)],
            **values,
        ),
        model_hash="",
        wall_time=0.0,
    )


def test_yaml_fragment_follows_indexes_and_ids() -> None:
    assert yaml_fragment(DOCUMENT, "weather.parameters") == "path: denver.mos"
    assert yaml_fragment(DOCUMENT, "constructions[id=ROOF:001].layers") == "- thickness: 0.1"
    assert yaml_fragment(DOCUMENT, "spaces[0].parameters") == "ach: 0.414"


def test_yaml_fragment_rejects_a_malformed_path() -> None:
    with pytest.raises(ValueError, match="malformed"):
        yaml_fragment(DOCUMENT, "spaces[zero]")


@pytest.mark.parametrize("featured", FEATURED_CASES, ids=lambda featured: featured.id)
def test_every_featured_path_exists_in_the_generated_case_file(featured) -> None:  # noqa: ANN001
    sections = case_yaml_sections(CASES[featured.id], featured.yaml_paths)

    assert [path for path, _ in sections] == featured.yaml_paths
    assert all(fragment for _, fragment in sections)


def test_the_results_table_marks_each_library() -> None:
    reference = load_reference()
    results = {
        "Buildings": {"600": _result("600", "Buildings")},
        "IDEAS": {"600": _result("600", "IDEAS", peak_cooling=9.0)},
    }

    table = results_table(CASES["600"], results, reference)
    summary = results_summary(CASES["600"], results, reference)

    assert "| peak cooling [kW] | 5.098 to 6.805 | 6.100 ✓ | 9.000 ✗ |" in table
    assert "- Buildings: 4 of 4 KPIs inside their band." in summary
    assert "- IDEAS: 3 of 4 KPIs inside their band, outside: peak cooling 9.000 kW against 5.098 to 6.805." in summary
    assert "- ISO 13790: not simulated yet." in summary


def test_the_day_chart_overlays_the_libraries_on_the_programs() -> None:
    reference = load_reference()
    hourly = next(item for item in reference.hourly_of("600") if item.quantity == "load")

    chart = day_chart(CASES["600"], hourly, {"Buildings": {"600": _result("600", "Buildings")}})

    assert chart is not None
    assert chart.count("<polyline") == len(hourly.programs) + 1
    assert "reference programs" in chart and ">Buildings<" in chart


def test_the_day_chart_is_skipped_without_any_simulated_trace() -> None:
    reference = load_reference()
    hourly = next(item for item in reference.hourly_of("600") if item.quantity == "load")

    assert day_chart(CASES["600"], hourly, {}) is None


def test_the_svg_chart_scales_the_values_to_the_plot() -> None:
    chart = svg_day_chart("title", "unit", [Series(name="p", values=[0.0] * 23 + [10.0], color=PROGRAM_COLOR)])

    assert chart.startswith("<svg") and chart.endswith("</svg>")
    assert ">0<" in chart and ">10<" in chart


def test_the_page_has_a_section_per_featured_case() -> None:
    page = render_walkthrough({"Buildings": {"600": _result("600", "Buildings")}}, load_reference())

    for featured in FEATURED_CASES:
        assert f"## Case {featured.id}: {CASES[featured.id].description}" in page
        assert featured.what in page and featured.yaml_note in page
    assert page.count("### How trano fares") == len(FEATURED_CASES)
    assert "<svg" in page
