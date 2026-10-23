"""BESTEST of FastModel: every KPI against the band of ASHRAE 140 and against Buildings, now and along the history.

Reads committed files only (no simulation): the results of the last BESTEST run of FastModel
(``validation/bestest/results/FastModel``), the frozen values of Buildings (``validation/bestest/expected``) and, with
``--history``, every version of the frozen FastModel values in the git history of the branch, each set against the
Buildings values frozen at the same commit.

    python -m analysis.fast_model.scripts.bestest_comparison --history

Writes ``bestest_kpis.csv`` (one row per case and KPI), ``bestest_summary.json`` and ``bestest_history.csv``.
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
from collections.abc import Mapping
from dataclasses import dataclass
from statistics import fmean

from analysis.fast_model.scripts._common import ROOT, configure_logging, write_csv, write_json
from validation.bestest.harness import committed_results
from validation.bestest.reference import KPIS, ReferenceData, load_reference
from validation.bestest.report import LOAD_FLOOR, acceptance_band, frozen_values

FROZEN = "validation/bestest/expected/{library}.json"
Values = Mapping[str, Mapping[str, float]]


@dataclass(frozen=True)
class KpiRow:
    case: str
    kpi: str
    unit: str
    fast_model: float
    buildings: float | None
    deviation: float | None  # FastModel against Buildings: relative for a load above LOAD_FLOOR, [K] for a temperature
    lower: float
    upper: float
    reference_mean: float
    inside: bool


def kpi_rows(fast_model: Values, buildings: Values, reference: ReferenceData) -> list[KpiRow]:
    rows = []
    for case in sorted(fast_model, key=lambda name: (len(name), name)):
        for kpi in KPIS:
            kpi_reference = reference.kpi(case, kpi)
            value = fast_model[case].get(kpi)
            if kpi_reference is None or value is None:
                continue
            lower, upper = acceptance_band(kpi_reference)
            other = buildings.get(case, {}).get(kpi)
            deviation = None
            if other is not None and kpi_reference.unit == "degC":
                deviation = value - other
            elif other is not None and abs(other) > LOAD_FLOOR:
                deviation = value / other - 1
            rows.append(
                KpiRow(
                    case=case,
                    kpi=kpi,
                    unit=kpi_reference.unit,
                    fast_model=value,
                    buildings=other,
                    deviation=deviation,
                    lower=lower,
                    upper=upper,
                    reference_mean=kpi_reference.mean,
                    inside=lower <= value <= upper,
                )
            )
    return rows


def _statistics(name: str, values: list[float]) -> dict[str, float | None]:
    return {
        f"{name}_mean": fmean(values) if values else None,
        f"{name}_min": min(values, default=None),
        f"{name}_max": max(values, default=None),
    }


def summary(rows: list[KpiRow]) -> dict[str, float | int | None]:
    """The statistics the BESTEST page gives: KPIs inside their band, gaps of the loads and temperatures."""

    def deviations(kpi: str | None = None, temperature: bool = False) -> list[float]:
        return [
            row.deviation
            for row in rows
            if row.deviation is not None and (row.unit == "degC") == temperature and kpi in (None, row.kpi)
        ]

    loads = [abs(value) for value in deviations()]
    temperatures = [abs(value) for value in deviations(temperature=True)]
    return (
        {"cases": len({row.case for row in rows}), "kpis": len(rows), "inside": sum(row.inside for row in rows)}
        | _statistics("annual_heating", deviations(kpi="annual_heating"))
        | _statistics("annual_cooling", deviations(kpi="annual_cooling"))
        | {"loads_mean_abs": fmean(loads) if loads else None, "loads_max_abs": max(loads, default=None)}
        | {
            "temperatures_mean_abs_k": fmean(temperatures) if temperatures else None,
            "temperatures_max_abs_k": max(temperatures, default=None),
        }
    )


def _git(*arguments: str) -> str:
    git = shutil.which("git")
    if git is None:
        raise RuntimeError("--history reads the git history: git is not on the PATH")
    # the arguments are a fixed git command and paths of this module
    return subprocess.run([git, *arguments], cwd=ROOT, capture_output=True, text=True, check=True).stdout  # noqa: S603


def history(reference: ReferenceData) -> list[dict[str, object]]:
    """The statistics of every frozen version of the FastModel values, oldest first."""
    rows: list[dict[str, object]] = []
    log = _git("log", "--reverse", "--format=%h%x09%ad%x09%s", "--date=short", "--", FROZEN.format(library="FastModel"))
    for line in log.splitlines():
        commit, date, subject = line.split("\t", 2)
        fast_model = json.loads(_git("show", f"{commit}:{FROZEN.format(library='FastModel')}"))
        buildings = json.loads(_git("show", f"{commit}:{FROZEN.format(library='Buildings')}"))
        rows.append(
            {"commit": commit, "date": date, "subject": subject} | summary(kpi_rows(fast_model, buildings, reference))
        )
    return rows


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--history", action="store_true", help="also every frozen version in the git history")
    arguments = parser.parse_args(argv)
    configure_logging()
    reference = load_reference()
    fast_model = {
        case: {kpi: result.kpis.value(kpi) for kpi in KPIS} for case, result in committed_results("FastModel").items()
    }
    rows = kpi_rows(fast_model, frozen_values("Buildings"), reference)
    write_csv("bestest_kpis.csv", [vars(row) for row in rows])
    shading = [row for row in rows if row.case in {"610", "630", "910", "930"}]
    write_json(
        "bestest_summary.json",
        {
            "all_cases": summary(rows),
            "shading_cases": summary(shading)
            | {"largest_gap_to_buildings": max(abs(row.deviation or 0) for row in shading)},
            "outside_band": [f"{row.case} {row.kpi}" for row in rows if not row.inside],
        },
    )
    if arguments.history:
        write_csv("bestest_history.csv", history(reference))


if __name__ == "__main__":
    main()
