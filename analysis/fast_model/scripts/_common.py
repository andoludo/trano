"""Paths and writers shared by the scripts of the FastModel analysis."""

from __future__ import annotations

import csv
import json
import logging
from collections.abc import Iterable, Mapping
from dataclasses import asdict, is_dataclass
from pathlib import Path
from typing import Any, cast

ROOT = Path(__file__).resolve().parents[3]
ANALYSIS = ROOT / "analysis" / "fast_model"
RESULTS = ANALYSIS / "results"
CACHE = ROOT / ".cache" / "analysis"
EXAMPLES = ROOT / "examples"
BESTEST_CASES = ROOT / "validation" / "bestest" / "cases"
KELVIN = 273.15

logger = logging.getLogger("analysis.fast_model")


def _plain(value: object) -> object:
    """JSON-ready: dataclasses as dicts, floats rounded to what the analysis can tell apart."""
    if is_dataclass(value) and not isinstance(value, type):
        return _plain(asdict(value))
    if isinstance(value, Mapping):
        return {str(key): _plain(item) for key, item in value.items()}
    if isinstance(value, list | tuple):
        return [_plain(item) for item in value]
    if isinstance(value, float):
        return round(value, 6)
    return value


def write_json(name: str, content: object) -> Path:
    """Write ``results/<name>`` and return its path."""
    path = RESULTS / name
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(_plain(content), indent=1) + "\n")
    logger.info("wrote %s", path.relative_to(ROOT))
    return path


def write_csv(name: str, rows: Iterable[Mapping[str, Any]]) -> Path:
    """Write ``results/<name>``; the header is every key of the rows in the order they first appear."""
    plain = [cast("dict[str, Any]", _plain(row)) for row in rows]
    fieldnames = list(dict.fromkeys(key for row in plain for key in row))
    path = RESULTS / name
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, restval="")
        writer.writeheader()
        writer.writerows(plain)
    logger.info("wrote %s (%d rows)", path.relative_to(ROOT), len(plain))
    return path


def configure_logging() -> None:
    logging.basicConfig(level=logging.INFO, format="%(message)s", force=True)
