"""BESTEST harness: YAML composition, cache hashing, Docker probe, run orchestration.

The harness merges ``cases/_base.yaml`` (lightweight envelope) plus optionally
``cases/_heavyweight.yaml`` (concrete-block override for 9xx cases) plus the
per-case ``cases/case_<id>.yaml`` into a single self-contained YAML, hands the
result to ``trano.data_models.conversion.convert_network``, runs the simulation
through ``trano.simulate.simulate.simulate``, and returns extracted KPIs.

Buildings library only — hard-coded.
"""
from __future__ import annotations

import hashlib
import shutil
import socket
import subprocess
import time
from pathlib import Path
from typing import Any

import yaml

from trano.data_models.conversion import convert_network
from trano.elements.library.library import Library
from trano.simulate.simulate import SimulationOptions, simulate
from trano.utils.utils import is_success
from validation.bestest.kpi import extract_kpis
from validation.bestest.spec.parameters import CASES, CaseParameters, KPIResults

CASES_DIR: Path = Path(__file__).parent / "cases"
BASE_YAML: Path = CASES_DIR / "_base.yaml"
HEAVYWEIGHT_YAML: Path = CASES_DIR / "_heavyweight.yaml"
CACHE_ROOT: Path = Path(__file__).resolve().parents[2] / ".cache" / "bestest"
DOCKER_SOCKET: Path = Path("/var/run/docker.sock")


def _case_yaml(case_id: str) -> Path:
    return CASES_DIR / f"case_{case_id}.yaml"


def docker_available() -> bool:
    """Return True iff a Docker client + reachable daemon are present.

    Used to skip the BESTEST suite cleanly on hosts without Modelica tooling.
    """
    if shutil.which("docker") is None:
        return False
    if DOCKER_SOCKET.exists():
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
                sock.settimeout(2.0)
                sock.connect(str(DOCKER_SOCKET))
            return True
        except (OSError, TimeoutError):
            return False
    docker_path = shutil.which("docker")
    if docker_path is None:
        return False
    try:
        result = subprocess.run(  # noqa: S603
            [docker_path, "info", "--format", "{{.ServerVersion}}"],
            capture_output=True,
            text=True,
            timeout=5,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return False
    return result.returncode == 0 and bool(result.stdout.strip())


def _merge_id_lists(base: list[Any], override: list[Any]) -> list[Any]:
    """Merge two lists of dicts by ``id`` field (override wins).

    Items without ``id`` cause a fallback to override-replaces-base semantics,
    matching the existing Trano YAML convention where lists keyed by ``id``
    behave like dicts.
    """
    keyed = all(isinstance(x, dict) and "id" in x for x in (*base, *override))
    if not keyed:
        return list(override)
    by_id: dict[Any, dict[str, Any]] = {item["id"]: item for item in base}
    for item in override:
        existing = by_id.get(item["id"])
        by_id[item["id"]] = _deep_merge(existing, item) if existing is not None else item
    return list(by_id.values())


def _deep_merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    """Recursively merge ``override`` into ``base``; override wins on conflict."""
    result: dict[str, Any] = dict(base)
    for key, value in override.items():
        existing = result.get(key)
        if isinstance(existing, dict) and isinstance(value, dict):
            result[key] = _deep_merge(existing, value)
        elif isinstance(existing, list) and isinstance(value, list):
            result[key] = _merge_id_lists(existing, value)
        else:
            result[key] = value
    return result


def _load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r") as fh:
        loaded = yaml.safe_load(fh)
    return loaded or {}


def build_yaml(case_id: str, *, output_dir: Path | None = None) -> Path:
    """Compose the merged YAML for ``case_id`` and write it to ``output_dir``.

    Default ``output_dir`` is ``.cache/bestest/<case_id>/``.
    """
    if case_id not in CASES:
        raise KeyError(f"Unknown BESTEST case: {case_id!r}")
    case = CASES[case_id]
    target_dir = output_dir if output_dir is not None else CACHE_ROOT / case_id
    target_dir.mkdir(parents=True, exist_ok=True)
    target = target_dir / "case.yaml"

    merged: dict[str, Any] = _load_yaml(BASE_YAML)
    if case.base == "heavyweight":
        merged = _deep_merge(merged, _load_yaml(HEAVYWEIGHT_YAML))
    merged = _deep_merge(merged, _load_yaml(_case_yaml(case_id)))

    with target.open("w") as fh:
        yaml.safe_dump(merged, fh, sort_keys=False)
    return target


def case_inputs(case_id: str) -> list[Path]:
    """Return the ordered list of files that contribute to ``case_id``'s output.

    The list is the SHA-256 input set for cache invalidation: any change to
    one of these files implies the cached simulation result is stale.
    """
    inputs = [BASE_YAML, _case_yaml(case_id)]
    if CASES[case_id].base == "heavyweight":
        inputs.insert(1, HEAVYWEIGHT_YAML)
    inputs.extend(
        [
            Path(__file__),
            Path(__file__).parent / "spec" / "parameters.py",
        ]
    )
    return inputs


def case_hash(case_id: str) -> str:
    """Return a hex SHA-256 over every file that influences ``case_id``'s KPIs."""
    digest = hashlib.sha256()
    for path in case_inputs(case_id):
        digest.update(path.name.encode())
        digest.update(b"\0")
        digest.update(path.read_bytes())
        digest.update(b"\0")
    return digest.hexdigest()


def case_parameters(case_id: str) -> CaseParameters:
    return CASES[case_id]


SECONDS_PER_YEAR: int = 31_536_000
RESULT_SUFFIX: str = ".building_res.mat"


def _result_mat(project_path: Path, network_name: str) -> Path:
    return project_path / "results" / f"{network_name}{RESULT_SUFFIX}"


def _network_for(case_id: str, yaml_path: Path) -> Any:  # noqa: ANN401  # trano Network
    return convert_network(
        f"case_{case_id}",
        yaml_path,
        library=Library.from_configuration("Buildings"),
    )


def _emission_names(network: Any, *, polarity: str) -> list[str]:  # noqa: ANN401
    """Return the normalized Modelica names of heater (positive) or cooler (negative) emissions.

    Iteration 1 convention: YAML IDs starting with ``HEATER`` are heaters, ``COOLER``
    are coolers. After convert_network normalization, names become lowercase with
    underscores.
    """
    prefix = "heater" if polarity == "heater" else "cooler"
    names = []
    for node in network.graph.nodes:
        if node.__class__.__name__ != "Radiator":
            continue
        if str(node.name).startswith(prefix):
            names.append(str(node.name))
    return names


def _read_cached(case_id: str, expected_hash: str) -> KPIResults | None:
    cache_dir = CACHE_ROOT / case_id
    hash_file = cache_dir / "hash.txt"
    kpis_file = cache_dir / "kpis.json"
    if not hash_file.exists() or not kpis_file.exists():
        return None
    if hash_file.read_text().strip() != expected_hash:
        return None
    return KPIResults.model_validate_json(kpis_file.read_text())


def _write_cache(case_id: str, results: KPIResults, hash_hex: str) -> None:
    cache_dir = CACHE_ROOT / case_id
    cache_dir.mkdir(parents=True, exist_ok=True)
    (cache_dir / "kpis.json").write_text(results.model_dump_json(indent=2))
    (cache_dir / "hash.txt").write_text(hash_hex + "\n")


def run_case(
    case_id: str,
    *,
    force: bool = False,
    end_time: int = SECONDS_PER_YEAR,
) -> KPIResults:
    """Execute the BESTEST case, returning KPIs (cached on case-input hash).

    Hard-fails when Docker isn't available — call ``docker_available()`` first
    in test setup to skip cleanly.
    """
    case = CASES[case_id]
    cache_dir = CACHE_ROOT / case_id
    cache_dir.mkdir(parents=True, exist_ok=True)

    yaml_path = build_yaml(case_id, output_dir=cache_dir)
    expected_hash = case_hash(case_id)

    if not force:
        cached = _read_cached(case_id, expected_hash)
        if cached is not None:
            return cached

    if not docker_available():
        raise RuntimeError(
            f"BESTEST case {case_id}: Docker daemon is not reachable; "
            "OpenModelica simulation cannot run."
        )

    network_name = f"case_{case_id}"
    network = _network_for(case_id, yaml_path)
    options = SimulationOptions(start_time=0, end_time=end_time, tolerance=1e-4)

    started = time.monotonic()
    sim_result = simulate(cache_dir, network, options=options)
    sim_wall = time.monotonic() - started
    omc_log_path = cache_dir / "omc_output.log"
    try:
        omc_output = sim_result.output.decode(errors="replace")
    except AttributeError:
        omc_output = ""
    omc_log_path.write_text(omc_output)
    if not is_success(sim_result):
        tail = "\n".join(omc_output.splitlines()[-40:]) or "<no output>"
        raise RuntimeError(
            f"BESTEST case {case_id}: simulation did not finish successfully.\n"
            f"Full OMC output saved to {omc_log_path}.\n"
            f"--- last 40 lines of OMC output ---\n{tail}\n--- end OMC output ---"
        )

    mat_path = _result_mat(cache_dir, network_name)
    if not mat_path.exists():
        raise RuntimeError(f"BESTEST case {case_id}: result file missing at {mat_path}")

    heater_names = _emission_names(network, polarity="heater")
    cooler_names = _emission_names(network, polarity="cooler")

    results = extract_kpis(
        mat_path,
        case_id=case_id,
        space_name="space_001",
        floor_area_m2=48.0,
        heater_names=heater_names,
        cooler_names=cooler_names,
        report_days=case.report_days_doy,
        sim_wall_time_s=sim_wall,
        cache_key=expected_hash,
    )
    _write_cache(case_id, results, expected_hash)
    return results


def run_all(
    case_ids: list[str] | None = None,
    *,
    force: bool = False,
    end_time: int = SECONDS_PER_YEAR,
) -> dict[str, KPIResults]:
    """Run every case sequentially (Docker holds a single OM container)."""
    targets = case_ids if case_ids is not None else list(CASES.keys())
    return {cid: run_case(cid, force=force, end_time=end_time) for cid in targets}
