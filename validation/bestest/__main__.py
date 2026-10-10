"""Command line of the BESTEST validation: ``python -m validation.bestest --help``."""

from __future__ import annotations

import argparse
import logging
from pathlib import Path

from validation.bestest.cases import (
    CASES,
    CASES_DIR,
    Case,
    UnsupportedCaseError,
    building_description,
    check_support,
    write_cases,
)
from validation.bestest.harness import LIBRARIES, cached_results, run_cases
from validation.bestest.report import DOCS_PAGE, compare, freeze, render_docs, render_markdown, write_report
from validation.bestest.walkthrough import WALKTHROUGH_PAGE, render_walkthrough
from validation.bestest.reference import load_reference

REPORT_DIR = Path(__file__).parent.joinpath("_reports")


def supported(case: Case, library: str) -> bool:
    try:
        building_description(case)
        check_support(case, library)
    except UnsupportedCaseError:
        return False
    return True


def list_cases() -> None:
    for case in CASES.values():
        try:
            building_description(case)
            support = "yaml"
        except UnsupportedCaseError as error:
            support = str(error)
        print(f"{case.id:6s} {case.description:60s} {support}")


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(prog="python -m validation.bestest", description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("list", help="cases and whether their YAML can be generated")
    generate = commands.add_parser("generate", help="write the case YAML files")
    generate.add_argument("--directory", type=Path, default=CASES_DIR)
    run = commands.add_parser("run", help="simulate cases and compare them with the reference")
    run.add_argument("cases", nargs="*", help="case ids, all when empty")
    run.add_argument("--library", choices=LIBRARIES, default="Buildings")
    run.add_argument("--workers", type=int, default=1, help="simulations run at a time")
    run.add_argument("--force", action="store_true", help="ignore cached results")
    report = commands.add_parser("report", help="render the cached results")
    report.add_argument("--directory", type=Path, default=REPORT_DIR)
    report.add_argument("--docs", action="store_true", help="also write the documentation page")
    freeze_ = commands.add_parser("freeze", help="freeze the cached results as the values the next ones are held to")
    freeze_.add_argument("--library", choices=LIBRARIES, default="Buildings")
    arguments = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(message)s")

    if arguments.command == "list":
        list_cases()
    elif arguments.command == "generate":
        for path in write_cases(arguments.directory):
            print(path)
    elif arguments.command == "run":
        case_ids = arguments.cases or [case.id for case in CASES.values() if supported(case, arguments.library)]
        results = run_cases(case_ids, arguments.library, force=arguments.force, workers=arguments.workers)
        reference = load_reference()
        for case_id, result in results.items():
            for comparison in compare(result.kpis, reference):
                print(
                    f"{case_id:6s} {comparison.kpi:20s} {comparison.value:9.3f} {comparison.unit:5s} "
                    f"[{comparison.lower:.3f}, {comparison.upper:.3f}] {comparison.status}"
                )
    elif arguments.command == "report":
        results = {library: cases for library in LIBRARIES if (cases := cached_results(library))}
        write_report(results, arguments.directory)
        if arguments.docs:
            DOCS_PAGE.parent.mkdir(parents=True, exist_ok=True)
            DOCS_PAGE.write_text(render_docs(results))
            WALKTHROUGH_PAGE.write_text(render_walkthrough(results))
            print(f"wrote {DOCS_PAGE} and {WALKTHROUGH_PAGE}")
        else:
            print(render_markdown(results))
    elif arguments.command == "freeze":
        print(freeze(arguments.library, cached_results(arguments.library)))


if __name__ == "__main__":
    main()
