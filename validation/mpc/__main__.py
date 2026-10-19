"""Compare FastModel with Buildings: ``python -m validation.mpc compare 600FF``.

Results are written to ``.cache/mpc_validation/results.json`` and rendered with ``report``.
"""

import argparse
import json
import logging
from typing import Any

from validation.mpc.cases import CASES
from validation.mpc.harness import (
    CACHE_ROOT,
    CLOSED_LOOP,
    FAST_MODEL,
    REFERENCE,
    REFERENCES,
    CaseComparison,
    compare,
)
from validation.mpc.report import render_report

RESULTS = CACHE_ROOT / "results.json"


def _load() -> dict[str, dict[str, Any]]:
    return json.loads(RESULTS.read_text()) if RESULTS.exists() else {}


def _store(key: str, comparison: CaseComparison) -> None:
    """Merge into the stored results as they are now: several comparisons may run side by side."""
    results = _load()
    results[key] = comparison.model_dump()
    RESULTS.parent.mkdir(parents=True, exist_ok=True)
    temporary = RESULTS.with_suffix(".tmp")
    temporary.write_text(json.dumps(results, indent=1))
    temporary.replace(RESULTS)


def main() -> None:
    parser = argparse.ArgumentParser(prog="python -m validation.mpc")
    commands = parser.add_subparsers(dest="command", required=True)
    run = commands.add_parser("compare", help="replay Buildings through FastModel and compare")
    run.add_argument("cases", nargs="*", default=list(CASES))
    run.add_argument(
        "--models",
        nargs="*",
        default=[FAST_MODEL],
        choices=[FAST_MODEL, CLOSED_LOOP],
        help=f"FastModel (replaying the controls of the reference) or {CLOSED_LOOP} (its own controllers)",
    )
    run.add_argument(
        "--references",
        nargs="*",
        default=[REFERENCE],
        choices=REFERENCES,
        help="libraries whose simulation is replayed",
    )
    commands.add_parser("report", help="print the Markdown report of the stored results")
    commands.add_parser("list", help="list the cases")
    arguments = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s", force=True)

    if arguments.command == "list":
        for case in CASES.values():
            print(f"{case.id:20s} {case.days:4d} days  {case.description}")
        return
    if arguments.command == "report":
        print(render_report([CaseComparison.model_validate(item) for item in _load().values()]))
        return
    for case_id in arguments.cases:
        for reference in arguments.references:
            for model in arguments.models:
                try:
                    comparison = compare(case_id, model, reference)
                except Exception:
                    logging.exception("Case %s with %s against %s failed", case_id, model, reference)
                    continue
                key = f"{case_id}/{model}" if reference == REFERENCE else f"{case_id}/{model}/{reference}"
                _store(key, comparison)
                logging.info(
                    "%s %s against %s: worst zone RMSE %.2f K", case_id, model, reference, comparison.worst_rmse
                )


if __name__ == "__main__":
    main()
