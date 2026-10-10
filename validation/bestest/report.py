"""Compare KPIs with the reference results and render the comparison."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Literal

from pydantic import BaseModel

from validation.bestest.harness import CaseResult
from validation.bestest.kpi import KpiResults
from validation.bestest.reference import KPIS, Kpi, KpiReference, ReferenceData, load_reference

TEMPERATURE_TOLERANCE = 1.0  # [K] added around the spread of the reference programs
PEAK_TOLERANCE = 0.05  # share of the largest reference peak, added around the spread of the programs
Status = Literal["pass", "fail", "known"]

# Results outside their band that are understood and accepted, with the reason: (library, case, KPI).
KNOWN_DEVIATIONS: dict[tuple[str, str, Kpi], str] = {
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
