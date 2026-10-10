"""Compare KPIs with the reference results and render the comparison."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Literal

from pydantic import BaseModel

from validation.bestest.harness import CaseResult
from validation.bestest.kpi import KpiResults
from validation.bestest.reference import KPIS, UNITS, Kpi, KpiReference, ReferenceData, load_reference

TEMPERATURE_TOLERANCE = 1.0  # [K] added around the spread of the reference programs
PEAK_TOLERANCE = 0.05  # share of the largest reference peak, added around the spread of the programs
Status = Literal["pass", "fail", "known"]

# Results outside their band that are understood and accepted, with the reason: (library, case, KPI).
KNOWN_DEVIATIONS: dict[tuple[str, str, Kpi], str] = {
    ("IDEAS", "950", "peak_cooling"): (
        "2.517 kW against a band ending at 2.507 kW (the spread of the reference programs plus 5 % of the "
        "largest peak): 0.4 % above a tolerance that is a convention, not a limit of the standard."
    ),
    ("Buildings", "980", "annual_cooling"): (
        "Buildings' own Case980 gives 3.418 MWh, 3 % below the lower limit of the standard (3.52 MWh); "
        "trano reproduces the library's model."
    ),
}


def acceptance_band(reference: KpiReference) -> tuple[float, float]:
    """Range a result must fall in.

    Annual loads: the acceptance limits of the standard. Peaks and temperatures (no limits in the
    standard): the spread of the reference programs widened by a tolerance.
    """
    if reference.lower_limit is not None and reference.upper_limit is not None:
        return min(reference.lower_limit, reference.upper_limit), max(reference.lower_limit, reference.upper_limit)
    margin = (
        TEMPERATURE_TOLERANCE
        if reference.kpi.endswith("_temperature")
        else PEAK_TOLERANCE * max(abs(value) for value in reference.values)
    )
    return reference.minimum - margin, reference.maximum + margin


class Comparison(BaseModel):
    case: str
    library: str
    kpi: Kpi
    value: float
    lower: float
    upper: float
    mean: float
    unit: str

    @property
    def status(self) -> Status:
        if self.lower <= self.value <= self.upper:
            return "pass"
        return "known" if self.known_deviation else "fail"

    @property
    def known_deviation(self) -> str | None:
        return KNOWN_DEVIATIONS.get((self.library, self.case, self.kpi))


def compare(kpis: KpiResults, reference: ReferenceData) -> list[Comparison]:
    """One comparison per KPI the reference data has for the case."""
    comparisons = []
    for kpi in KPIS:
        kpi_reference = reference.kpi(kpis.case, kpi)
        if kpi_reference is None:
            continue
        lower, upper = acceptance_band(kpi_reference)
        comparisons.append(
            Comparison(
                case=kpis.case,
                library=kpis.library,
                kpi=kpi,
                value=kpis.value(kpi),
                lower=lower,
                upper=upper,
                mean=kpi_reference.mean,
                unit=kpi_reference.unit,
            )
        )
    return comparisons


def render_markdown(results: dict[str, dict[str, CaseResult]], reference: ReferenceData | None = None) -> str:
    """Table per library: case, KPI, value, acceptance band, reference mean, status."""
    reference = reference or load_reference()
    lines = ["# BESTEST results", ""]
    for library, cases in results.items():
        lines += [
            f"## {library}",
            "",
            "| Case | KPI | trano | Band | Reference mean | Status |",
            "|---|---|---|---|---|---|",
        ]
        for case_id in sorted(cases, key=lambda name: (len(name), name)):
            for comparison in compare(cases[case_id].kpis, reference):
                band = f"{comparison.lower:.3f} to {comparison.upper:.3f}"
                status = comparison.status
                if status == "known":
                    status = f"known deviation: {comparison.known_deviation}"
                lines.append(
                    f"| {comparison.case} | {comparison.kpi} | {comparison.value:.3f} {comparison.unit} "
                    f"| {band} | {comparison.mean:.3f} | {status} |"
                )
        lines.append("")
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


EXPECTED_DIR = Path(__file__).parent.joinpath("expected")
REGRESSION_TOLERANCE = 0.02  # relative change of a frozen value that counts as a regression
DOCS_PAGE = Path(__file__).parents[2].joinpath("docs", "validation", "bestest.md")


def freeze(library: str, results: dict[str, CaseResult]) -> Path:
    """Write the KPI values of a library as the frozen values its future results are held to."""
    EXPECTED_DIR.mkdir(parents=True, exist_ok=True)
    path = EXPECTED_DIR.joinpath(f"{library}.json")
    frozen = {
        case_id: {kpi: round(result.kpis.value(kpi), 4) for kpi in KPIS}
        for case_id, result in sorted(results.items(), key=lambda item: (len(item[0]), item[0]))
    }
    path.write_text(json.dumps(frozen, indent=2) + "\n")
    return path


def frozen_values(library: str) -> dict[str, dict[str, float]]:
    path = EXPECTED_DIR.joinpath(f"{library}.json")
    if not path.exists():
        return {}
    return json.loads(path.read_text())  # type: ignore[no-any-return]


def regressions(library: str, case_id: str, kpis: KpiResults) -> list[str]:
    """KPIs that moved by more than the tolerance from their frozen value (absolute 0.01 near zero)."""
    frozen = frozen_values(library).get(case_id, {})
    moved = []
    for kpi, expected in frozen.items():
        value = kpis.value(kpi)  # type: ignore[arg-type]
        if abs(value - expected) > max(REGRESSION_TOLERANCE * abs(expected), 0.01):
            moved.append(f"{kpi}: {value:.3f} was {expected:.3f} {UNITS[kpi]}")  # type: ignore[index]
    return moved


def render_docs(results: dict[str, dict[str, CaseResult]], reference: ReferenceData | None = None) -> str:
    """The documentation page: what is validated, how, and the tables of the report."""
    intro = """# ASHRAE 140 (BESTEST) validation

The building models trano generates are validated against the 27 cases of section 5.2 of ASHRAE
Standard 140-2020 (the BESTEST single-zone cases): a 8 m x 6 m x 2.7 m zone in Denver with light
or heavy constructions, 12 m2 of double glazing, 0.414 air changes per hour of infiltration and
200 W of internal gains, with variants for the insulation, the glazing, the window orientation,
overhangs and side fins, an unconditioned sun-space, free-floating temperatures, a dual set point
ideal system with set-backs and night ventilation.

Each case is a trano YAML file (`validation/bestest/cases/`), simulated for a year with each
library. The annual heating and cooling loads must fall inside the acceptance limits of the
standard; the peak loads and the free-floating temperatures must fall inside the spread of the
reference programs (BSIMAC, CSE, DeST, EnergyPlus, ESP-r, TRNSYS) widened by 5 % of the largest
peak or 1 K. Buildings and IDEAS are gating libraries: `pytest -m bestest` fails when one of
their cases leaves its band, except for the deviations listed below with their reason. The
reduced-order (AixLib) and ISO 13790 zones are shown for information: ISO 13790 is a monthly
method applied hour by hour and the reduced-order zone still deviates on several cases, so
neither gates. Cases a library cannot describe (shading with IDEAS, which OpenModelica cannot
compile for IDEAS 3.0.0; shading, sun-space and night ventilation with the two simplified zones)
are left out. The tables are rendered from the last results with
`python -m validation.bestest report --docs`.

"""
    body = render_markdown(results, reference)
    return intro + body[body.index("\n") + 1 :].lstrip("\n")
