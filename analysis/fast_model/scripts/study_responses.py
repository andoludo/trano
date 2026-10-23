"""The FastModel-against-Buildings study as tables: KPIs, hour-by-hour fidelity, responses and verdicts.

Reads the committed results of the study (``validation/fast_model/results``, written by the workflow
``fast_model_study.yaml``); nothing is simulated. The verdicts are those of the study page
(``validation.fast_model.report.agreement``): *alike* when, on every main KPI, the ratio of the changes lies within
0.9 to 1.1 or the two changes are within 0.05 K or 0.5 % of the base case; *close* within 0.8 to 1.25, 0.1 K or 1 %.

    python -m analysis.fast_model.scripts.study_responses

Writes ``study_kpis.csv``, ``study_fidelity.csv``, ``study_responses.csv``, ``study_verdicts.csv``,
``study_summary.json`` and ``study_hour_of_day.csv`` (the base case of each house, by hour of the day).
"""

from __future__ import annotations

import argparse
from collections import Counter
from typing import Any

import pandas as pd

from analysis.fast_model.scripts._common import configure_logging, write_csv, write_json
from validation.fast_model.cases import BASE, CASES
from validation.fast_model.harness import BUILDINGS, FAST_MODEL, LIBRARIES, load_kpis, load_series
from validation.fast_model.report import SUMMARY_KPIS, VERDICTS, agreement, fidelity, sensitivity_rows

HOUSES = ("residential_after", "residential_before")
METERS = ("heating_kwh", "peak_heating_kw", "cooling_kwh", "electricity_kwh", "fuel_kwh", "fans_kwh", "pv_kwh")


def kpi_rows() -> list[dict[str, Any]]:
    rows = []
    for case in CASES.values():
        for variant in case.variants:
            for library in LIBRARIES:
                kpis = load_kpis(case.id, variant, library)
                if kpis is None:
                    continue
                rows.append(
                    {"case": case.id, "variant": variant, "library": library}
                    | {name: kpis.value(name) for name in ("mean_temperature", "min_temperature", "max_temperature")}
                    | {name: kpis.value(name) for name in METERS}
                    | {"wall_time_s": kpis.wall_time}
                )
    return rows


def fidelity_rows() -> list[dict[str, Any]]:
    return [
        {"case": case.id, "variant": variant} | row.model_dump()
        for case in CASES.values()
        for variant in case.variants
        for row in fidelity(case, variant)
    ]


def response_rows() -> list[dict[str, Any]]:
    return [
        {"case": case.id} | row | {"agreement": VERDICTS[agreement(row)]}
        for case in CASES.values()
        for row in sensitivity_rows(case)
    ]


def verdict_rows() -> list[dict[str, Any]]:
    """One row per perturbation: the ratio of the changes on each main KPI and the verdict of the study page."""
    rows = []
    for case in CASES.values():
        moved: dict[str, dict[str, dict[str, object]]] = {}
        for row in sensitivity_rows(case):
            moved.setdefault(str(row["perturbation"]), {})[str(row["kpi"])] = row
        for perturbation in case.perturbations:
            if None in (load_kpis(case.id, perturbation.id, library) for library in LIBRARIES):
                continue
            kpis = moved.get(perturbation.id, {})
            levels = [agreement(kpis[name]) for name, _ in SUMMARY_KPIS if name in kpis]
            rows.append(
                {"case": case.id, "perturbation": perturbation.id, "group": perturbation.group}
                | {f"ratio_{name}": kpis[name]["ratio"] if name in kpis else None for name, _ in SUMMARY_KPIS}
                | {
                    "verdict": VERDICTS[max(levels)] if levels else "neither moves",
                    "description": perturbation.description,
                }
            )
    return rows


def hour_of_day_rows() -> list[dict[str, Any]]:
    """Mean of every series of the base case of each house by hour of the day, both libraries side by side."""
    rows: list[dict[str, Any]] = []
    for case_id in HOUSES:
        hours: dict[str, pd.DataFrame] = {}
        for library in LIBRARIES:
            frame = load_series(case_id, BASE, library)
            if frame is not None:
                hours[library] = frame.groupby((frame.index.to_numpy() // 3600) % 24).mean()
        if len(hours) < len(LIBRARIES):
            continue
        columns = [column for column in ("TOut", "heating", "electricity", "fuel") if column in hours[BUILDINGS]]
        rows.extend(
            {"case": case_id, "hour": hour}
            | {
                f"{column}_{library}": float(hours[library][column].to_numpy()[hour])
                for column in columns
                for library in LIBRARIES
            }
            for hour in range(24)
        )
    return rows


def summary(verdicts: list[dict[str, Any]], fidelities: list[dict[str, Any]]) -> dict[str, Any]:
    base = pd.DataFrame([row for row in fidelities if row["variant"] == BASE])
    return {
        case.id: {
            "verdicts": dict(Counter(row["verdict"] for row in verdicts if row["case"] == case.id)),
            "base_rmse_k": [float(base[base.case == case.id].rmse.min()), float(base[base.case == case.id].rmse.max())],
            "base_heating_deviation": _deviation(case.id, "heating_kwh"),
            "base_fuel_deviation": _deviation(case.id, "fuel_kwh"),
            "base_electricity_deviation": _deviation(case.id, "electricity_kwh"),
        }
        for case in CASES.values()
    }


def _deviation(case_id: str, name: str) -> float | None:
    reference, model = (load_kpis(case_id, BASE, library) for library in (BUILDINGS, FAST_MODEL))
    if reference is None or model is None:
        return None
    expected, value = reference.value(name), model.value(name)
    return None if expected is None or value is None or abs(expected) < 1e-9 else value / expected - 1


def main(argv: list[str] | None = None) -> None:
    argparse.ArgumentParser(description=__doc__.splitlines()[0]).parse_args(argv)
    configure_logging()
    write_csv("study_kpis.csv", kpi_rows())
    fidelities = fidelity_rows()
    write_csv("study_fidelity.csv", fidelities)
    write_csv("study_responses.csv", response_rows())
    verdicts = verdict_rows()
    write_csv("study_verdicts.csv", verdicts)
    write_csv("study_hour_of_day.csv", hour_of_day_rows())
    write_json("study_summary.json", summary(verdicts, fidelities))


if __name__ == "__main__":
    main()
