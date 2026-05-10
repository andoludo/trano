"""Fast unit tests for validation.bestest.report (no simulation required)."""
from datetime import datetime
from pathlib import Path

from validation.bestest.report import (
    _classify,
    compare_to_reference,
    load_reference_bands,
    write_report,
)
from validation.bestest.spec.parameters import HourlyTrace, KPIBands, KPIResults


def _result(case_id: str, t_min: float = -10.0, t_max: float = 65.0) -> KPIResults:
    epoch = datetime(2024, 1, 1)
    return KPIResults(
        case_id=case_id,
        annual_heating_kwh_m2=0.0,
        annual_cooling_kwh_m2=0.0,
        peak_heating_w=0.0,
        peak_heating_at=epoch,
        peak_cooling_w=0.0,
        peak_cooling_at=epoch,
        t_zone_min_c=t_min,
        t_zone_min_at=datetime(2024, 1, 15),
        t_zone_max_c=t_max,
        t_zone_max_at=datetime(2024, 7, 1),
        hourly_traces=[HourlyTrace(day_of_year=4, hours=list(range(24)), values_c=[20.0] * 24)],
        sim_wall_time_s=10.0,
        cache_key="abc123",
    )


def test_classify_inside_band_passes() -> None:
    band = KPIBands(case_id="600FF", kpi="t_zone_min_c", ref_min=-18.0, ref_max=-15.0, ref_mean=-16.5, units="degC")
    assert _classify(-16.0, band) == "pass"


def test_classify_just_outside_band_warns() -> None:
    band = KPIBands(case_id="600FF", kpi="t_zone_min_c", ref_min=-18.0, ref_max=-15.0, ref_mean=-16.5, units="degC")
    # Span is 3.0 → 10% tolerance is 0.3. -14.8 is 0.2 above ref_max → warn.
    assert _classify(-14.8, band) == "warn"


def test_classify_far_outside_band_fails() -> None:
    band = KPIBands(case_id="600FF", kpi="t_zone_min_c", ref_min=-18.0, ref_max=-15.0, ref_mean=-16.5, units="degC")
    assert _classify(0.0, band) == "fail"


def test_compare_to_reference_per_kpi_status() -> None:
    bands = [
        KPIBands(case_id="600FF", kpi="t_zone_min_c", ref_min=-18.0, ref_max=-15.0, ref_mean=-16.5, units="degC"),
        KPIBands(case_id="600FF", kpi="t_zone_max_c", ref_min=64.0, ref_max=70.0, ref_mean=67.0, units="degC"),
    ]
    statuses = compare_to_reference(_result("600FF", t_min=-16.0, t_max=65.0), bands)
    assert statuses == {"t_zone_min_c": "pass", "t_zone_max_c": "pass"}


def test_load_reference_bands_skips_comment_lines() -> None:
    bands = load_reference_bands()
    assert "600FF" in bands
    assert "900FF" in bands
    assert any(b.kpi == "t_zone_min_c" for b in bands["600FF"])


def test_write_report_creates_artifacts(tmp_path: Path) -> None:
    results = {"600FF": _result("600FF"), "900FF": _result("900FF", t_min=-1.0, t_max=42.0)}
    bands_by_case = {
        "600FF": [
            KPIBands(case_id="600FF", kpi="t_zone_min_c", ref_min=-18.0, ref_max=-15.0, ref_mean=-16.5, units="degC"),
        ],
    }
    write_report(results, bands_by_case, tmp_path)
    md = (tmp_path / "report.md").read_text()
    assert "## Case 600FF" in md
    assert "t_zone_min_c" in md
    assert (tmp_path / "report.json").exists()
    assert (tmp_path / "plots" / "trumpet_t_zone_min_c.png").exists()
