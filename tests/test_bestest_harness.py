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
    # Wood siding (MATERIAL:001) is preserved from _base.yaml in the heavyweight stack.
    mat1 = next(m for m in data["material"] if m["id"] == "MATERIAL:001")
    assert mat1["density"] == 530  # wood siding (outside layer of heavyweight wall too)
    # New dedicated heavyweight material IDs introduced by _heavyweight.yaml.
    concrete = next(m for m in data["material"] if m["id"] == "MATERIAL:HW_CONCRETE_BLOCK")
    assert concrete["density"] == 1400
    # EXTERIOR_WALL:001 layer order is outside→inside: wood siding, foam, concrete block.
    ext_wall = next(c for c in data["constructions"] if c["id"] == "EXTERIOR_WALL:001")
    materials = [layer["material"] for layer in ext_wall["layers"]]
    thicknesses = [layer["thickness"] for layer in ext_wall["layers"]]
    assert materials == ["MATERIAL:001", "MATERIAL:HW_FOAM", "MATERIAL:HW_CONCRETE_BLOCK"]
    assert thicknesses == pytest.approx([0.009, 0.0615, 0.100])


def test_build_yaml_applies_ashrae_140_internal_gain_split(tmp_path: Path) -> None:
    # ASHRAE 140 §5.2.1.7: 200 W continuous = 120 W radiative + 80 W convective.
    out = build_yaml("600", output_dir=tmp_path)
    data = yaml.safe_load(out.read_text())
    gain = data["spaces"][0]["occupancy"]["parameters"]["gain"]
    assert "120/48" in gain and "80/48" in gain
    assert gain.index("120/48") < gain.index("80/48")  # radiative first


def test_build_yaml_sets_infiltration(tmp_path: Path) -> None:
    out = build_yaml("600FF", output_dir=tmp_path)
    data = yaml.safe_load(out.read_text())
    assert data["spaces"][0]["parameters"]["ach"] == pytest.approx(0.41)


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
