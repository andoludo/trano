"""Compare the MPC libraries (flat ``mpc``, ``FastModel``) with Buildings: ``python -m validation.mpc compare 600FF``.

Results are written to ``.cache/mpc_validation/results.json`` and rendered with ``report``.
"""

import argparse
import json
import logging
from typing import Any

from trano.mpc import RCModelType
from validation.mpc.cases import CASES
from validation.mpc.harness import CACHE_ROOT, CaseComparison, compare
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
    run = commands.add_parser("compare", help="replay Buildings through mpc models and compare the zones")
    run.add_argument("cases", nargs="*", default=list(CASES))
    run.add_argument("--models", nargs="*", default=[model.value for model in RCModelType] + ["Multilayer"])
    run.add_argument("--flat", action="store_true", help="flat instead of connected building_mpc")
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
        for model in arguments.models:
            try:
                if model == "Multilayer":
                    comparison = compare(case_id, envelope="multilayer")
                else:
                    comparison = compare(case_id, RCModelType(model), connected=not arguments.flat)
            except Exception:
                logging.exception("Case %s with %s failed", case_id, model)
                continue
            _store(f"{case_id}/{comparison.model}", comparison)
            logging.info("%s %s: worst zone RMSE %.2f K", case_id, comparison.model, comparison.worst_rmse)


if __name__ == "__main__":
    main()
