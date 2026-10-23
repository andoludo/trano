"""Run the FastModel-against-Buildings study and render its report.

python -m validation.fast_model list
python -m validation.fast_model plan 950FF 940                 # the (case, variant) matrix as JSON, for CI
python -m validation.fast_model run 950FF --variants base insulation_x1.5 --libraries Buildings FastModel
python -m validation.fast_model report --docs                  # tables and figures into the documentation
"""

from __future__ import annotations

import argparse
import json
import logging
from typing import Any

from validation.fast_model.cases import BASE, CASES
from validation.fast_model.harness import LIBRARIES, run
from validation.fast_model.report import render_report, write_docs, write_figures


def plan(cases: list[str] | None = None, variants: list[str] | None = None) -> list[dict[str, str]]:
    """Every (case, variant) pair of the study, restricted to the cases and variants given."""
    rows: list[dict[str, str]] = []
    for case_id in cases or list(CASES):
        case = CASES[case_id]
        rows.extend(
            {"case": case.id, "variant": variant}
            for variant in case.variants
            if not variants or variant in variants or (variant == BASE and BASE in variants)
        )
    return rows


def _list() -> None:
    for case in CASES.values():
        print(f"{case.id:20s} {case.days:4d} days  {case.description}")
        for perturbation in case.perturbations:
            print(f"    {perturbation.id:22s} {perturbation.group:9s} {perturbation.description}")


def _run(case_id: str, variants: list[str] | None, libraries: list[str], force: bool) -> None:
    case = CASES[case_id]
    failures: list[str] = []
    for variant in variants or case.variants:
        for library in libraries:
            try:
                run(case.id, variant, library, force=force)
            except Exception:
                logging.exception("%s %s with %s failed", case.id, variant, library)
                failures.append(f"{variant}/{library}")
    if failures:
        raise SystemExit(f"Failed: {', '.join(failures)}")


def _report(docs: bool) -> None:
    print(render_report())
    if docs:
        figures: list[Any] = write_figures()
        write_docs()
        print(f"\n{len(figures)} figures written, documentation page updated")


def main() -> None:
    parser = argparse.ArgumentParser(prog="python -m validation.fast_model")
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("list", help="the cases and their variants")
    planned = commands.add_parser("plan", help="the (case, variant) matrix as JSON")
    planned.add_argument("cases", nargs="*")
    planned.add_argument("--variants", nargs="*", default=[])
    runner = commands.add_parser("run", help="simulate a case with both libraries")
    runner.add_argument("case", choices=list(CASES))
    runner.add_argument("--variants", nargs="*", default=None, help="all variants of the case by default")
    runner.add_argument("--libraries", nargs="*", default=list(LIBRARIES), choices=LIBRARIES)
    runner.add_argument("--force", action="store_true", help="simulate again although the model is cached")
    report = commands.add_parser("report", help="print the Markdown report of the stored results")
    report.add_argument("--docs", action="store_true", help="also write the documentation page and its figures")
    arguments = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s", force=True)

    if arguments.command == "list":
        _list()
    elif arguments.command == "plan":
        print(json.dumps(plan(arguments.cases or None, arguments.variants or None), separators=(",", ":")))
    elif arguments.command == "run":
        _run(arguments.case, arguments.variants, arguments.libraries, arguments.force)
    else:
        _report(arguments.docs)


if __name__ == "__main__":
    main()
