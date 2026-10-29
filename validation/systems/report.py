"""Judge the KPIs against ``expected.yaml`` and render the report and the documentation page."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Literal

import yaml
from pydantic import BaseModel, Field

from validation.systems.cases import CASES, LIBRARIES, Family
from validation.systems.harness import CaseResult
from validation.systems.kpi import UNITS

EXPECTED_FILE = Path(__file__).parent.joinpath("expected.yaml")
DOCS_PAGE = Path(__file__).parents[2].joinpath("docs", "validation", "systems.md")
Status = Literal["pass", "fail", "missing"]


class Band(BaseModel):
    """What a KPI of a case must be, and why."""

    kpi: str
    min: float | None = None
    max: float | None = None
    why: str = ""
    libraries: tuple[str, ...] = Field(default=LIBRARIES, description="Libraries the band applies to")

    def holds(self, value: float) -> bool:
        return (self.min is None or value >= self.min) and (self.max is None or value <= self.max)

    @property
    def text(self) -> str:
        if self.min is not None and self.max is not None:
            return f"{self.min:g} to {self.max:g}"
        if self.min is not None:
            return f"at least {self.min:g}"
        return f"at most {self.max:g}" if self.max is not None else "any"


class Comparison(BaseModel):
    case: str
    library: str
    band: Band
    value: float | None

    @property
    def status(self) -> Status:
        if self.value is None:
            return "missing"
        return "pass" if self.band.holds(self.value) else "fail"


def load_expected(path: Path = EXPECTED_FILE) -> dict[str, list[Band]]:
    """The bands of every case: ``{case: [{kpi, min, max, why, libraries}]}``."""
    if not path.exists():
        return {}
    data = yaml.safe_load(path.read_text()) or {}
    expected: dict[str, list[Band]] = {}
    for case_id, bands in data.items():
        expected[case_id] = [Band(**band) for band in bands]
    return expected


def compare(result: CaseResult, expected: dict[str, list[Band]] | None = None) -> list[Comparison]:
    expected = load_expected() if expected is None else expected
    return [
        Comparison(case=result.case, library=result.library, band=band, value=result.kpis.get(band.kpi))
        for band in expected.get(result.case, [])
        if result.library in band.libraries
    ]


def failures(result: CaseResult, expected: dict[str, list[Band]] | None = None) -> list[str]:
    """The KPIs of a result outside their band, or missing, one line each with the reasoning of the band."""
    messages = []
    for comparison in compare(result, expected):
        if comparison.status == "pass":
            continue
        band = comparison.band
        if comparison.value is None:
            messages.append(f"{band.kpi}: missing from the KPIs ({band.why})")
        else:
            unit = UNITS.get(band.kpi, "")
            messages.append(f"{band.kpi}: {comparison.value:.3f} {unit} outside {band.text} ({band.why})")
    return messages


def _format(value: float | None) -> str:
    if value is None:
        return "-"
    return f"{value:.3g}" if abs(value) < 1000 else f"{value:.0f}"


def render_family(family: Family, results: dict[str, dict[str, CaseResult]], figures: bool = False) -> list[str]:
    """One table per case of the family: KPI, value per library, band, status; the figure of the case after it."""
    expected = load_expected()
    lines: list[str] = []
    for case in CASES.values():
        if case.family is not family:
            continue
        libraries = [library for library in LIBRARIES if case.id in results.get(library, {})]
        if not libraries:
            continue
        bands = {band.kpi: band for band in expected.get(case.id, [])}
        kpis = sorted({kpi for library in libraries for kpi in results[library][case.id].kpis})
        lines += [
            f"### `{case.id}`",
            "",
            f"{case.description}. Week from day {case.start_day + 1} of the year.",
            "",
            "| KPI | " + " | ".join(libraries) + " | Band | Status |",
            "|---|" + "---|" * len(libraries) + "---|---|",
        ]
        for kpi in kpis:
            values = [results[library][case.id].kpis.get(kpi) for library in libraries]
            band = bands.get(kpi)
            unit = UNITS.get(kpi, "")
            statuses = [
                "pass" if value is not None and band.holds(value) else "fail"
                for library, value in zip(libraries, values, strict=True)
                if band is not None and library in band.libraries
            ]
            status = ("fail" if "fail" in statuses else "pass") if statuses else ""
            lines.append(
                f"| {kpi} [{unit}] | "
                + " | ".join(_format(value) for value in values)
                + f" | {band.text if band else ''} | {status} |"
            )
        lines.append("")
        if figures and any(results[library][case.id].traces for library in libraries):
            from validation.systems.figures import figure_name

            lines += [FIGURE.format(name=figure_name(case.id)), ""]
    return lines


def render_markdown(results: dict[str, dict[str, CaseResult]], figures: bool = False) -> str:
    lines = ["# Systems validation results", ""]
    for family in Family:
        family_lines = render_family(family, results, figures)
        if family_lines:
            lines += [f"## {family.value.replace('_', ' ').capitalize()}", "", *family_lines]
    return "\n".join(lines)


def write_report(results: dict[str, dict[str, CaseResult]], directory: Path) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    directory.joinpath("report.md").write_text(render_markdown(results))
    directory.joinpath("report.json").write_text(
        json.dumps(
            {
                library: {case_id: result.model_dump(mode="json") for case_id, result in cases.items()}
                for library, cases in results.items()
            },
            indent=2,
        )
    )


INTRO = """# Systems validation

Do the heating and ventilation systems trano generates behave as the real ones do? Each case below puts one
system on the same 100 m2 zone (filled cavity walls, insulated roof and slab, 10 m2 of low-e double glazing
facing south, n50 = 3, occupied from 7 to 19 h, heated to 21 degC from 7 to 22 h and 16 degC at night) for one
week of the Uccle weather, and is simulated with Buildings, IDEAS and FastSmoothBuildings. The KPIs are read
from the result files by `validation/systems/kpi.py`; the band of every KPI and the physical reasoning behind
it are in `validation/systems/expected.yaml`; `pytest -m systems` fails when a KPI leaves its band. The tables
are rendered from the last results with `python -m validation.systems report --docs`.

"""


FIGURE = "{{!validation/figures/{name}.html!}}"  # markdown_include, as the tutorial pages do


def render_docs(results: dict[str, dict[str, CaseResult]]) -> str:
    """The documentation page: the introduction, then the tables of every family, each case followed by the figure
    of its week (written by ``figures.write_figure``)."""
    body = render_markdown(results, figures=True)
    return INTRO + body[body.index("\n") + 1 :].lstrip("\n")


def summary(results: dict[str, dict[str, CaseResult]]) -> dict[str, Any]:
    """Counts of passing, failing and missing KPI bands per library."""
    expected = load_expected()
    counts: dict[str, Any] = {}
    for library, cases in results.items():
        statuses: list[str] = [c.status for result in cases.values() for c in compare(result, expected)]
        counts[library] = {status: statuses.count(status) for status in ("pass", "fail", "missing")}
    return counts
