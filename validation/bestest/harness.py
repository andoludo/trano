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
from pathlib import Path
from typing import Any

import yaml

from validation.bestest.spec.parameters import CASES, CaseParameters

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
        result = subprocess.run(
            [docker_path, "info", "--format", "{{.ServerVersion}}"],  # noqa: S603
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
