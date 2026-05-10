"""Fast unit tests for validation.bestest.harness (no simulation required)."""
from datetime import datetime
from pathlib import Path

import pytest
import yaml

from validation.bestest.harness import (
    _deep_merge,
    _merge_id_lists,
    build_yaml,
    case_hash,
    run_case,
)
from validation.bestest.spec.parameters import CASES, KPIResults


def test_merge_id_lists_overrides_by_id() -> None:
    base = [{"id": "A", "x": 1}, {"id": "B", "x": 2}]
    over = [{"id": "B", "x": 99}, {"id": "C", "x": 3}]
    merged = _merge_id_lists(base, over)
    by_id = {item["id"]: item for item in merged}
    assert by_id["A"]["x"] == 1
    assert by_id["B"]["x"] == 99
    assert by_id["C"]["x"] == 3


def test_deep_merge_recurses_into_dicts() -> None:
    base = {"weather": {"path": "old", "extra": "keep"}}
    over = {"weather": {"path": "new"}}
    merged = _deep_merge(base, over)
    assert merged["weather"]["path"] == "new"
    assert merged["weather"]["extra"] == "keep"


def test_build_yaml_lightweight_uses_base_only(tmp_path: Path) -> None:
    out = build_yaml("600FF", output_dir=tmp_path)
    data = yaml.safe_load(out.read_text())
    mat1 = next(m for m in data["material"] if m["id"] == "MATERIAL:001")
    assert mat1["density"] == 530  # lightweight wood siding


def test_build_yaml_heavyweight_swaps_envelope(tmp_path: Path) -> None:
    out = build_yaml("900", output_dir=tmp_path)
    data = yaml.safe_load(out.read_text())
    mat1 = next(m for m in data["material"] if m["id"] == "MATERIAL:001")
    assert mat1["density"] == 1400  # concrete block
    ext_wall = next(c for c in data["constructions"] if c["id"] == "EXTERIOR_WALL:001")
    assert ext_wall["layers"][0]["thickness"] == pytest.approx(0.100)


def test_case_hash_changes_with_case_yaml(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    h1 = case_hash("600FF")
    h2 = case_hash("900FF")
    assert h1 != h2  # different case YAML bytes
    h3 = case_hash("600FF")
    assert h1 == h3  # deterministic


def test_run_case_returns_cached_result_when_hash_matches(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr("validation.bestest.harness.CACHE_ROOT", tmp_path)
    case_dir = tmp_path / "600FF"
    case_dir.mkdir()
    fixture = KPIResults(
        case_id="600FF",
        annual_heating_kwh_m2=0.0,
        annual_cooling_kwh_m2=0.0,
        peak_heating_w=0.0,
        peak_heating_at=datetime(2024, 1, 1),
        peak_cooling_w=0.0,
        peak_cooling_at=datetime(2024, 1, 1),
        t_zone_min_c=10.0,
        t_zone_min_at=datetime(2024, 1, 15),
        t_zone_max_c=35.0,
        t_zone_max_at=datetime(2024, 7, 1),
        sim_wall_time_s=12.5,
        cache_key=case_hash("600FF"),
    )
    (case_dir / "kpis.json").write_text(fixture.model_dump_json())
    (case_dir / "hash.txt").write_text(case_hash("600FF") + "\n")
    # Need build_yaml() to write the merged YAML there too (run_case does it).
    result = run_case("600FF")
    assert result.t_zone_min_c == 10.0
    assert result.cache_key == case_hash("600FF")


def test_cases_dict_covers_ten_cases() -> None:
    assert len(CASES) == 10
    assert set(CASES) == {
        "600", "600FF", "620", "640", "650",
        "900", "900FF", "920", "940", "950",
    }
