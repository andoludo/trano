"""BESTEST comparison + reporting: load reference bands, classify pass/warn/fail,
render markdown + JSON + trumpet plots.

Pass/warn/fail policy:
  - pass: KPI value lies inside [ref_min, ref_max].
  - warn: KPI value lies within ±10% of the band edge but outside the band.
  - fail: KPI value is further than 10% outside.
  - no_band: no reference row exists for this (case, kpi). Treated as informational.

Iteration 1 is warn-only at the pytest layer (see tests/test_bestest.py).
"""
from __future__ import annotations

import csv
import json
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from validation.bestest.spec.parameters import (
    REFERENCE_CSV,
    HourlyTrace,
    KPIBands,
    KPIName,
    KPIResults,
)

WARN_TOLERANCE: float = 0.10  # 10% beyond a band edge → warn rather than fail

KPI_VALUE_ATTR: dict[str, str] = {
    "annual_heating_kwh_m2": "annual_heating_kwh_m2",
    "annual_cooling_kwh_m2": "annual_cooling_kwh_m2",
    "peak_heating_w": "peak_heating_w",
    "peak_cooling_w": "peak_cooling_w",
    "t_zone_min_c": "t_zone_min_c",
    "t_zone_max_c": "t_zone_max_c",
}


def load_reference_bands(path: Path = REFERENCE_CSV) -> dict[str, list[KPIBands]]:
    """Load reference bands from a CSV. Lines beginning with ``#`` are skipped."""
    bands: dict[str, list[KPIBands]] = defaultdict(list)
    with path.open("r") as fh:
        reader = csv.DictReader(line for line in fh if not line.lstrip().startswith("#"))
        for row in reader:
            bands[row["case_id"]].append(
                KPIBands(
                    case_id=row["case_id"],
                    kpi=row["kpi"],  # type: ignore[arg-type]
                    ref_min=float(row["ref_min"]),
                    ref_max=float(row["ref_max"]),
                    ref_mean=float(row["ref_mean"]),
                    units=row["units"],
                    source=row.get("source") or "ASHRAE 140-2017 Annex B",
                )
            )
    return dict(bands)


def _kpi_value(result: KPIResults, kpi: KPIName) -> float | None:
    value = getattr(result, KPI_VALUE_ATTR[kpi], None)
    return float(value) if value is not None else None


def _classify(value: float, band: KPIBands) -> str:
    if band.ref_min <= value <= band.ref_max:
        return "pass"
    span = band.ref_max - band.ref_min if band.ref_max > band.ref_min else abs(band.ref_mean) or 1.0
    if value < band.ref_min and (band.ref_min - value) <= WARN_TOLERANCE * span:
        return "warn"
    if value > band.ref_max and (value - band.ref_max) <= WARN_TOLERANCE * span:
        return "warn"
    return "fail"


def compare_to_reference(result: KPIResults, bands: list[KPIBands]) -> dict[str, str]:
    """Return a status (pass|warn|fail|no_band) for every KPI in the bands list."""
    statuses: dict[str, str] = {}
    for band in bands:
        value = _kpi_value(result, band.kpi)
        statuses[band.kpi] = "no_band" if value is None else _classify(value, band)
    return statuses


def _render_markdown(
    results: dict[str, KPIResults],
    bands_by_case: dict[str, list[KPIBands]],
) -> str:
    lines = ["# BESTEST report", ""]
    for case_id in sorted(results):
        result = results[case_id]
        bands = bands_by_case.get(case_id, [])
        lines.append(f"## Case {case_id}")
        lines.append("")
        lines.append(f"- Simulation wall time: {result.sim_wall_time_s:.1f} s")
        lines.append(f"- Cache key: `{result.cache_key[:12]}...`")
        lines.append("")
        lines.append("| KPI | Value | Ref min | Ref max | Status |")
        lines.append("|-----|-------|---------|---------|--------|")
        if not bands:
            lines.append("| _no reference bands seeded yet_ |  |  |  |  |")
        for band in bands:
            value = _kpi_value(result, band.kpi)
            value_str = f"{value:.3f}" if value is not None else "n/a"
            status = "no_band" if value is None else _classify(value, band)
            lines.append(
                f"| {band.kpi} | {value_str} | {band.ref_min:.3f} | {band.ref_max:.3f} | {status} |"
            )
        lines.append("")
        if result.hourly_traces:
            lines.append("### Hourly zone temperature [C]")
            lines.append("")
            lines.extend(
                f"- DOY {trace.day_of_year}: " + ", ".join(f"{v:.1f}" for v in trace.values_c)
                for trace in result.hourly_traces
            )
            lines.append("")
    return "\n".join(lines)


def _render_json(
    results: dict[str, KPIResults],
    bands_by_case: dict[str, list[KPIBands]],
) -> str:
    out: dict[str, dict[str, object]] = {}
    for case_id, result in results.items():
        bands = bands_by_case.get(case_id, [])
        statuses = compare_to_reference(result, bands)
        out[case_id] = {
            "kpis": result.model_dump(mode="json"),
            "statuses": statuses,
        }
    return json.dumps(out, indent=2, default=str)


def _trumpet_plot(
    out_path: Path,
    kpi: KPIName,
    results: dict[str, KPIResults],
    bands_by_case: dict[str, list[KPIBands]],
) -> None:
    fig, ax = plt.subplots(figsize=(10, 5))
    case_ids = sorted(results)
    x = list(range(len(case_ids)))

    band_min = []
    band_max = []
    for cid in case_ids:
        match = next((b for b in bands_by_case.get(cid, []) if b.kpi == kpi), None)
        band_min.append(match.ref_min if match else float("nan"))
        band_max.append(match.ref_max if match else float("nan"))
    ax.fill_between(x, band_min, band_max, alpha=0.25, label="Reference band")

    values = [_kpi_value(results[cid], kpi) for cid in case_ids]
    ax.plot(x, values, "o-", label="Trano (Buildings)")

    ax.set_xticks(x)
    ax.set_xticklabels(case_ids, rotation=45)
    ax.set_ylabel(kpi)
    ax.set_title(f"BESTEST trumpet plot: {kpi}")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)


def write_report(
    results: dict[str, KPIResults],
    bands_by_case: dict[str, list[KPIBands]],
    out_dir: Path,
) -> None:
    """Write report.md, report.json, and one trumpet PNG per KPI to ``out_dir``."""
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "report.md").write_text(_render_markdown(results, bands_by_case))
    (out_dir / "report.json").write_text(_render_json(results, bands_by_case))
    plot_dir = out_dir / "plots"
    plot_dir.mkdir(parents=True, exist_ok=True)
    for kpi in KPI_VALUE_ATTR:
        _trumpet_plot(plot_dir / f"trumpet_{kpi}.png", kpi, results, bands_by_case)  # type: ignore[arg-type]


def _hourly_trace_for_test() -> HourlyTrace:  # pragma: no cover - sanity helper
    return HourlyTrace(day_of_year=4, hours=list(range(24)), values_c=[20.0] * 24)
