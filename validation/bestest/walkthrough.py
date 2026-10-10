"""A walk through a few BESTEST cases for the documentation.

For each featured case the page tells what the case is, what the reference programs of the
standard expect, how the trano YAML describes it and how the simulated results compare with
that expectation. The prose lives here; the numbers, the YAML fragments and the hourly charts
are rendered from the generated case files, the reference data and the last results, so that
the page never drifts away from what is actually simulated.
"""

from __future__ import annotations

import re
from pathlib import Path
from typing import Any

import yaml
from pydantic import BaseModel, Field

from validation.bestest.cases import CASES, Case, case_file, check_support
from validation.bestest.harness import LIBRARIES, CaseResult
from validation.bestest.kpi import HourlyTrace
from validation.bestest.reference import KPIS, UNITS, HourlyReference, Kpi, ReferenceData, load_reference
from validation.bestest.report import Comparison, Status, acceptance_band, compare

WALKTHROUGH_PAGE = Path(__file__).parents[2].joinpath("docs", "validation", "bestest_cases.md")
GATING_LIBRARIES = ("Buildings", "IDEAS")
STATUS_MARKS: dict[Status, str] = {"pass": "✓", "known": "~", "fail": "✗"}
LIBRARY_LABELS = {
    "Buildings": "Buildings",
    "IDEAS": "IDEAS",
    "reduced_order": "reduced order",
    "iso_13790": "ISO 13790",
}
HOURS_PER_DAY = 24
_PATH_SEGMENT = re.compile(r"^(?P<name>[A-Za-z_]+)(?:\[(?P<index>\d+|id=[^\]]+)\])?$")


class FeaturedCase(BaseModel):
    """A case of the walk-through: its prose and the YAML fragments worth showing."""

    id: str
    what: str = Field(description="What the case is, and what differs from the case it builds on")
    expectation: str = Field(description="What the reference programs expect and the physics behind it")
    yaml_paths: list[str] = Field(description="Paths into the generated YAML, e.g. spaces[0].emissions")
    yaml_note: str = Field(description="How the YAML expresses the case")


FEATURED_CASES: list[FeaturedCase] = [
    FeaturedCase(
        id="600",
        what=(
            "The base case: a single rectangular zone of 8 m by 6 m by 2.7 m in Denver (cold, sunny, "
            "1609 m above sea level) with light-weight walls (wood siding, fibreglass, plasterboard), a "
            "light roof and a timber floor over 1 m of insulation whose underside sees the outdoor air. "
            "The south wall carries 12 m2 of clear double glazing without any frame. Infiltration is "
            "0.414 air changes per hour (0.5 at sea level, corrected for the altitude), the internal gains "
            "are 200 W around the clock (60 % radiant, 40 % convective) and an ideal system keeps the air "
            "between 20 degC (heating) and 27 degC (cooling) with unlimited capacity."
        ),
        expectation=(
            "Winter nights drive the heating, so the heating peak falls on the coldest hours of January "
            "or December. The cooling peak falls on a sunny winter day too: the low sun pours through the "
            "south glazing and the light envelope has nothing to store it in, so the zone hits 27 degC "
            "within a few hours. The annual loads must fall inside the acceptance limits of the standard; "
            "the peaks must fall inside the spread of the reference programs widened by 5 % of the "
            "largest peak. The hourly loads of 1 February are compared as well: a cold clear day on "
            "which the zone switches from heating at night to cooling around noon."
        ),
        yaml_paths=[
            "weather",
            "constructions[id=LIGHT_WALL:001]",
            "spaces[0].parameters",
            "spaces[0].external_boundaries.windows",
            "spaces[0].occupancy",
            "spaces[0].emissions",
        ],
        yaml_note=(
            "The zone is a `space` with the `infiltration` variant, whose `ach` parameter is the air "
            "change rate; `linearize_emissive_power: false` keeps the long-wave exchange non-linear as "
            "the standard asks. The floor is a `floor_on_grounds` boundary with the `outdoor_air` variant "
            "(its outer surface follows the outdoor dry-bulb temperature), the roof is an external wall "
            "with the `ceiling` tilt. The 200 W of internal gains are an `occupancy` element occupied all "
            "day, with the radiant, convective and latent gains per square metre of floor. The ideal "
            "system is the `ideal_heating_cooling` emission: its set points are day schedules in kelvin "
            "(time since midnight in seconds, value) repeated every day, and its capacities are 1 MW so "
            "that the set points are always met. The weather is the Denver TMY3 file shipped with "
            "Buildings; the atmospheric pressure is read from the file, so that the air density, and with "
            "it the infiltration mass flow, is that of the site. Every material has 18 states per 0.2 m "
            "of layer (`number_of_states`), the discretisation of Buildings' own BESTEST models."
        ),
    ),
    FeaturedCase(
        id="600FF",
        what=(
            "Case 600 without any heating or cooling: the temperature of the zone floats freely under the "
            "weather and the internal gains."
        ),
        expectation=(
            "The light envelope stores next to nothing, so the zone follows the weather closely: it drops "
            "well below freezing on winter nights and climbs above 60 degC on sunny days, with the glazing "
            "turning the zone into a greenhouse. The standard compares the maximum, minimum and annual "
            "mean temperature with the spread of the reference programs (widened by 1 K here), and the "
            "hourly temperatures of 1 February."
        ),
        yaml_paths=["spaces[0].parameters", "spaces[0].occupancy"],
        yaml_note=(
            "The YAML is that of case 600 without the `emissions` list: a space without an emission is "
            "free-floating. The occupancy stays, since the 200 W of internal gains are part of the case."
        ),
    ),
    FeaturedCase(
        id="900",
        what=(
            "Case 600 with heavy constructions: the walls are concrete blocks behind foam insulation and "
            "wood siding, the floor is a concrete slab over the insulation. The U-values are the same as in "
            "case 600; only the thermal mass changes."
        ),
        expectation=(
            "The mass stores the solar gains of the day and gives them back at night, so both the heating "
            "and the cooling loads drop a lot against case 600, the cooling by more than half. The peaks "
            "drop too, and the heating peak moves to the end of long cold spells rather than to the first "
            "cold night. The hourly loads of 1 February show a much smoother profile than case 600."
        ),
        yaml_paths=["constructions[id=HEAVY_WALL:001]", "constructions[id=HEAVY_FLOOR:001]"],
        yaml_note=(
            "Only the constructions change: the walls and the floor refer to the heavy constructions, "
            "whose concrete layers carry the mass. For the libraries that aggregate the envelope (the "
            "reduced-order and ISO 13790 zones), trano derives the mass class of the zone from the "
            "heat capacity of these layers."
        ),
    ),
    FeaturedCase(
        id="640",
        what=(
            "Case 600 with a night set-back of the heating: the heating set point is 10 degC from 23:00 to "
            "07:00 and 20 degC otherwise. The cooling set point stays at 27 degC."
        ),
        expectation=(
            "The zone cools down at night, so the annual heating drops against case 600. The heating peak "
            "rises though: at 07:00 the system has to bring a cold zone back to 20 degC at once, and this "
            "morning pick-up is the largest load of the year, well above the peak of case 600."
        ),
        yaml_paths=["spaces[0].emissions"],
        yaml_note=(
            "The heating set point becomes a day schedule with the set-back: pairs of time since midnight "
            "and set point in kelvin, held constant between two rows, with the step written as two rows "
            "at the same time. The schedule repeats every day."
        ),
    ),
    FeaturedCase(
        id="650",
        what=(
            "Case 600 without heating: the cooling set point is 27 degC from 07:00 to 18:00 and off "
            "otherwise, and a fan blows 1700 m3/h of outdoor air through the zone from 18:00 to 07:00 "
            "without adding any heat."
        ),
        expectation=(
            "The annual heating is zero by construction. The night ventilation flushes the zone with cold "
            "outdoor air, so the cooling drops against case 600 but by less than the free cooling would "
            "suggest: the zone is light and heats up again as soon as the sun rises. The cooling peak "
            "stays close to that of case 600, since it occurs in the afternoon, long after the fan "
            "stopped."
        ),
        yaml_paths=["spaces[0].parameters", "spaces[0].emissions"],
        yaml_note=(
            "The fan is the `ventilation_schedule` parameter of the space: a day schedule of outdoor air "
            "mass flow in kg/s brought in on top of the infiltration, at the outdoor temperature and "
            "humidity, 1409 kg/h being 1700 m3/h at the air density of the site. The heating is switched "
            "off with a set point of 0 degC and a capacity of zero; the cooling set point is 100 degC "
            "outside the cooling hours."
        ),
    ),
    FeaturedCase(
        id="610",
        what=(
            "Case 600 with a 1 m deep horizontal overhang above the south window, placed 0.5 m above the "
            "top of the glazing and running the full width of the wall."
        ),
        expectation=(
            "The overhang cuts the high summer sun and leaves the low winter sun alone, so the annual "
            "cooling drops noticeably against case 600 while the heating hardly moves. The cooling peak "
            "drops a little: it occurs in winter, when the overhang shades only part of the window."
        ),
        yaml_paths=["spaces[0].external_boundaries.windows"],
        yaml_note=(
            "The overhang is an `overhang` object of the window: its depth, the gap between the top of "
            "the window and the overhang, and how far it extends past the window on each side. Side fins "
            "(cases 630 and 930) are a `side_fins` object in the same spirit."
        ),
    ),
    FeaturedCase(
        id="960",
        what=(
            "A two-zone case: the back zone of case 600 keeps its light envelope but loses its windows, "
            "and a 2 m deep unconditioned sun-space with heavy walls, a concrete slab and the 12 m2 of "
            "south glazing is attached to its south wall. The two zones share a 0.2 m concrete common "
            "wall. The back zone is conditioned between 20 and 27 degC; the sun-space floats freely."
        ),
        expectation=(
            "The sun-space collects the solar gains, stores them in its mass and conducts part of them "
            "through the common wall into the back zone, so the back zone needs less heating than case "
            "600 and very little cooling. The standard checks the loads of the back zone and the "
            "maximum, minimum and mean temperature of the sun-space."
        ),
        yaml_paths=["spaces[1]", "internal_walls"],
        yaml_note=(
            "The sun-space is a second space with `occupancy: {variant: none}` (no internal gains at "
            "all) and no emission. The common wall is an `internal_walls` entry between the two spaces; "
            "trano infers the rest of the topology from the boundaries of each space."
        ),
    ),
]


def featured_case(case_id: str) -> FeaturedCase:
    return next(featured for featured in FEATURED_CASES if featured.id == case_id)


# --------------------------------------------------------------------------- #
# YAML fragments
# --------------------------------------------------------------------------- #


def yaml_fragment(document: dict[str, Any], path: str) -> str:
    """Dump the part of the document at `path`.

    A path is a dotted sequence of keys; a key can be indexed by position (`spaces[0]`) or by the
    `id` of a list entry (`constructions[id=LIGHT_WALL:001]`).
    """
    node: Any = document
    for segment in path.split("."):
        match = _PATH_SEGMENT.match(segment)
        if match is None:
            raise ValueError(f"malformed path segment {segment!r} in {path!r}")
        node = node[match.group("name")]
        index = match.group("index")
        if index is None:
            continue
        if index.startswith("id="):
            wanted = index.removeprefix("id=")
            node = next(entry for entry in node if entry.get("id") == wanted)
        else:
            node = node[int(index)]
    return yaml.safe_dump(node, sort_keys=False, width=100).rstrip()


def case_yaml_sections(case: Case, paths: list[str]) -> list[tuple[str, str]]:
    document = yaml.safe_load(case_file(case.id).read_text())
    return [(path, yaml_fragment(document, path)) for path in paths]


# --------------------------------------------------------------------------- #
# Charts: inline SVG, so that the page renders anywhere without a script
# --------------------------------------------------------------------------- #

PROGRAM_COLOR = "#9aa5b1"
LIBRARY_COLORS = {"Buildings": "#1f77b4", "IDEAS": "#d62728", "reduced_order": "#2ca02c", "iso_13790": "#9467bd"}
_WIDTH, _HEIGHT = 720, 320
_LEFT, _RIGHT, _TOP, _BOTTOM = 56, 16, 20, 40


class Series(BaseModel):
    name: str
    values: list[float]
    color: str
    width: float = 1.0
    dashed: bool = False


def _ticks(low: float, high: float, count: int = 5) -> list[float]:
    span = high - low or 1.0
    raw = span / count
    magnitude = 10 ** int(f"{raw:e}".split("e")[1])
    step = next(candidate * magnitude for candidate in (1, 2, 2.5, 5, 10) if candidate * magnitude >= raw)
    first = step * int(low // step)
    ticks = []
    tick = first
    while tick <= high + step / 2:
        if tick >= low - step / 2:
            ticks.append(round(tick, 6))
        tick += step
    return ticks


def svg_day_chart(title: str, y_label: str, series: list[Series]) -> str:
    """A line chart of 24 hourly values per series: hour of the day on the x-axis."""
    values = [value for item in series for value in item.values]
    ticks = _ticks(min(values), max(values))
    low, high = min(ticks[0], *values), max(ticks[-1], *values)
    plot_width, plot_height = _WIDTH - _LEFT - _RIGHT, _HEIGHT - _TOP - _BOTTOM

    def x(hour: float) -> float:
        return _LEFT + (hour - 1) / 23 * plot_width

    def y(value: float) -> float:
        return _TOP + (high - value) / (high - low) * plot_height

    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {_WIDTH} {_HEIGHT}" width="100%" '
        f'role="img" aria-label="{title}" style="font-family: sans-serif; font-size: 12px; max-width: {_WIDTH}px">',
        f"<title>{title}</title>",
        f'<text x="{_LEFT}" y="{_TOP - 6}" font-weight="bold">{title}</text>',
    ]
    for tick in ticks:
        parts.append(
            f'<line x1="{_LEFT}" x2="{_WIDTH - _RIGHT}" y1="{y(tick):.1f}" y2="{y(tick):.1f}" '
            'stroke="#e0e0e0" stroke-width="1"/>'
        )
        parts.append(f'<text x="{_LEFT - 6}" y="{y(tick) + 4:.1f}" text-anchor="end">{tick:g}</text>')
    parts.extend(
        f'<text x="{x(hour):.1f}" y="{_HEIGHT - _BOTTOM + 16}" text-anchor="middle">{hour}</text>'
        for hour in range(1, 25, 3)
    )
    parts.append(
        f'<text x="{_LEFT + plot_width / 2:.1f}" y="{_HEIGHT - 6}" text-anchor="middle">hour of the day</text>'
    )
    parts.append(
        f'<text transform="translate(14 {_TOP + plot_height / 2:.1f}) rotate(-90)" text-anchor="middle">'
        f"{y_label}</text>"
    )
    for item in series:
        points = " ".join(f"{x(hour + 1):.1f},{y(value):.1f}" for hour, value in enumerate(item.values))
        dash = ' stroke-dasharray="6 3"' if item.dashed else ""
        parts.append(
            f'<polyline fill="none" stroke="{item.color}" stroke-width="{item.width}"{dash} points="{points}">'
            f"<title>{item.name}</title></polyline>"
        )
    entries = _legend_entries(series)
    legend_x = _WIDTH - _RIGHT - sum(_legend_width(entry) for entry in entries)
    for entry in entries:
        dash = ' stroke-dasharray="6 3"' if entry.dashed else ""
        parts.append(
            f'<line x1="{legend_x}" x2="{legend_x + 24}" y1="{_TOP - 10}" y2="{_TOP - 10}" '
            f'stroke="{entry.color}" stroke-width="{entry.width}"{dash}/>'
        )
        parts.append(f'<text x="{legend_x + 30}" y="{_TOP - 6}">{entry.name}</text>')
        legend_x += _legend_width(entry)
    parts.append("</svg>")
    return "\n".join(parts)


def _legend_width(entry: Series) -> float:
    """Room for a legend entry on the title line: the line sample, the gap and about 6.5 px per character."""
    return 24 + 6 + 6.5 * len(entry.name) + 16


def _legend_entries(series: list[Series]) -> list[Series]:
    """One legend entry per colour: the reference programs share one."""
    entries: list[Series] = []
    for item in series:
        if all(entry.color != item.color for entry in entries):
            entries.append(
                item if item.color != PROGRAM_COLOR else item.model_copy(update={"name": "reference programs"})
            )
    return entries


def day_chart(
    case: Case,
    reference: HourlyReference,
    results: dict[str, dict[str, CaseResult]],
) -> str | None:
    """The hourly chart of the trace day: reference programs in grey, the gating libraries in colour."""
    series = [
        Series(name=program, values=values, color=PROGRAM_COLOR)
        for program, values in reference.programs.items()
        if len(values) == HOURS_PER_DAY
    ]
    for library in GATING_LIBRARIES:
        result = results.get(library, {}).get(case.id)
        if result is None:
            continue
        traces = result.kpis.temperature_traces if reference.quantity == "temperature" else result.kpis.load_traces
        trace = _trace_of(traces, reference.month, reference.day)
        if trace is not None:
            series.append(Series(name=library, values=trace.values, color=LIBRARY_COLORS[library], width=2.5))
    if len(series) == len(reference.programs):
        return None
    day = f"{reference.day} {_MONTHS[reference.month]}"
    if reference.quantity == "temperature":
        return svg_day_chart(f"Case {case.id}: zone temperature on {day}", "temperature [degC]", series)
    return svg_day_chart(f"Case {case.id}: hourly load on {day}", "load [kWh], heating positive", series)


def _trace_of(traces: list[HourlyTrace], month: int, day: int) -> HourlyTrace | None:
    return next((trace for trace in traces if trace.month == month and trace.day == day), None)


_MONTHS = {1: "January", 2: "February", 3: "March", 4: "April", 5: "May", 6: "June", 7: "July", 8: "August",
           9: "September", 10: "October", 11: "November", 12: "December"}  # fmt: skip


# --------------------------------------------------------------------------- #
# Tables
# --------------------------------------------------------------------------- #


def expectation_table(case: Case, reference: ReferenceData) -> str:
    """The value of each reference program per KPI, and the band a result must fall in."""
    references = reference.kpis_of(case.id)
    programs = sorted({program for item in references for program in item.programs}, key=str.lower)
    lines = [
        "| KPI | " + " | ".join(programs) + " | Band |",
        "|---|" + "---|" * len(programs) + "---|",
    ]
    for item in references:
        lower, upper = acceptance_band(item)
        cells = [f"{item.programs[program].value:.3f}" if program in item.programs else "" for program in programs]
        kind = "limits of the standard" if item.lower_limit is not None else "programs ± tolerance"
        lines.append(
            f"| {_kpi_label(item.kpi)} [{item.unit}] | "
            + " | ".join(cells)
            + f" | {lower:.3f} to {upper:.3f} ({kind}) |"
        )
    return "\n".join(lines)


def results_table(case: Case, results: dict[str, dict[str, CaseResult]], reference: ReferenceData) -> str:
    """One column per library: the trano value and whether it falls in the band."""
    libraries = [library for library in LIBRARIES if _supported(case, library)]
    comparisons = {
        library: {item.kpi: item for item in compare(results[library][case.id].kpis, reference)}
        for library in libraries
        if case.id in results.get(library, {})
    }
    lines = [
        "| KPI | Band | " + " | ".join(LIBRARY_LABELS[library] for library in libraries) + " |",
        "|---|---|" + "---|" * len(libraries),
    ]
    for kpi in KPIS:
        item = reference.kpi(case.id, kpi)
        if item is None:
            continue
        lower, upper = acceptance_band(item)
        cells = []
        for library in libraries:
            comparison = comparisons.get(library, {}).get(kpi)
            cells.append(_result_cell(comparison) if comparison else "not simulated")
        lines.append(f"| {_kpi_label(kpi)} [{UNITS[kpi]}] | {lower:.3f} to {upper:.3f} | " + " | ".join(cells) + " |")
    return "\n".join(lines)


def _result_cell(comparison: Comparison) -> str:
    return f"{comparison.value:.3f} {STATUS_MARKS[comparison.status]}"


def results_summary(case: Case, results: dict[str, dict[str, CaseResult]], reference: ReferenceData) -> str:
    """One sentence per library: how many KPIs sit in their band, and which do not."""
    sentences = []
    for library in LIBRARIES:
        if not _supported(case, library):
            sentences.append(f"{LIBRARY_LABELS[library]}: the case is not supported.")
            continue
        result = results.get(library, {}).get(case.id)
        if result is None:
            sentences.append(f"{LIBRARY_LABELS[library]}: not simulated yet.")
            continue
        comparisons = compare(result.kpis, reference)
        passed = [item for item in comparisons if item.status == "pass"]
        sentence = f"{LIBRARY_LABELS[library]}: {len(passed)} of {len(comparisons)} KPIs inside their band"
        outside = [item for item in comparisons if item.status != "pass"]
        if outside:
            details = "; ".join(
                f"{_kpi_label(item.kpi)} {item.value:.3f} {item.unit} against {item.lower:.3f} to {item.upper:.3f}"
                + (" (known deviation)" if item.status == "known" else "")
                for item in outside
            )
            sentence += f", outside: {details}"
        sentences.append(sentence + ".")
    return "\n".join(f"- {sentence}" for sentence in sentences)


def _supported(case: Case, library: str) -> bool:
    try:
        check_support(case, library)
    except NotImplementedError:
        return False
    return True


def _kpi_label(kpi: Kpi) -> str:
    return kpi.replace("_", " ")


# --------------------------------------------------------------------------- #
# The page
# --------------------------------------------------------------------------- #

INTRO = """# A walk through the BESTEST cases

The [validation page](bestest.md) lists every KPI of every case. This page takes a few cases and
tells, for each of them, what the case is, what the six reference programs of ASHRAE Standard
140-2020 (BSIMAC, CSE, DeST, EnergyPlus, ESP-r and TRNSYS) expect, how the trano YAML describes
the case and how the results of each library compare with that expectation. The YAML fragments
are cut out of the generated case files in `validation/bestest/cases/`, the tables and the charts
are rendered from the reference data and the last results, with
`python -m validation.bestest report --docs`.

A band is the range a result must fall in: the acceptance limits of the standard for the annual
loads, and the spread of the reference programs widened by 5 % of the largest peak (loads) or by
1 K (temperatures) otherwise. In the result tables, ✓ marks a value inside its band, ~ a known
deviation documented on the validation page and ✗ a value outside its band. Buildings and IDEAS
gate the test suite; the reduced-order (AixLib) and ISO 13790 zones are shown for information.
"""


def render_walkthrough(results: dict[str, dict[str, CaseResult]], reference: ReferenceData | None = None) -> str:
    reference = reference or load_reference()
    sections = [INTRO, *(render_case(CASES[featured.id], featured, results, reference) for featured in FEATURED_CASES)]
    return "\n".join(sections)


def render_case(
    case: Case,
    featured: FeaturedCase,
    results: dict[str, dict[str, CaseResult]],
    reference: ReferenceData,
) -> str:
    lines = [f"## Case {case.id}: {case.description}", "", featured.what, ""]
    lines += [
        "### What the reference programs expect",
        "",
        expectation_table(case, reference),
        "",
        featured.expectation,
        "",
    ]
    lines += ["### How the YAML describes it", "", featured.yaml_note, ""]
    for path, fragment in case_yaml_sections(case, featured.yaml_paths):
        lines += [f"`{path}`:", "", "```yaml", fragment, "```", ""]
    lines += ["### How trano fares", "", results_table(case, results, reference), ""]
    lines += [results_summary(case, results, reference), ""]
    for hourly in reference.hourly_of(case.id):
        chart = day_chart(case, hourly, results)
        if chart is not None:
            lines += [chart, ""]
    return "\n".join(lines)
