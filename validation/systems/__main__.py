"""Command line of the systems validation: ``python -m validation.systems --help``."""

from __future__ import annotations

import argparse
import logging
from pathlib import Path

from validation.systems.cases import CASES, CASES_DIR, LIBRARIES, Family, write_cases
from validation.systems.harness import CaseResult, commit_results, load_results, run_cases
from validation.systems.report import DOCS_PAGE, render_docs, render_markdown, write_report

REPORT_DIR = Path(__file__).parent.joinpath("_reports")


def list_cases(family: str | None) -> None:
    for case in CASES.values():
        if family and case.family.value != family:
            continue
        print(f"{case.id:30s} {case.family.value:12s} day {case.start_day:3d} {case.description}")


def selected_cases(case_ids: list[str], family: str | None) -> list[str]:
    if case_ids:
        unknown = set(case_ids) - set(CASES)
        if unknown:
            raise SystemExit(f"Unknown cases {sorted(unknown)}; see `list`")
        return case_ids
    return [case.id for case in CASES.values() if not family or case.family.value == family]


def report_results(directory: Path, docs: bool) -> None:
    results = {library: cases for library in LIBRARIES if (cases := load_results(library))}
    write_report(results, directory)
    if docs:
        DOCS_PAGE.parent.mkdir(parents=True, exist_ok=True)
        DOCS_PAGE.write_text(render_docs(results))
        print(f"wrote {DOCS_PAGE}")
    else:
        print(render_markdown(results))


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(prog="python -m validation.systems", description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    list_ = commands.add_parser("list", help="the cases")
    list_.add_argument("--family", choices=[family.value for family in Family])
    generate = commands.add_parser("generate", help="write the YAML of the cases")
    generate.add_argument("--directory", type=Path, default=CASES_DIR)
    run = commands.add_parser("run", help="simulate cases and extract their KPIs")
    run.add_argument("cases", nargs="*", help="case ids, every case (of the family) when empty")
    run.add_argument("--family", choices=[family.value for family in Family])
    run.add_argument("--library", choices=LIBRARIES, action="append", help="default: the three libraries")
    run.add_argument("--workers", type=int, default=1, help="simulations run at a time")
    run.add_argument("--force", action="store_true", help="ignore the cached results")
    report = commands.add_parser("report", help="the report of the results (cached and committed)")
    report.add_argument("--directory", type=Path, default=REPORT_DIR)
    report.add_argument("--docs", action="store_true", help="also write the documentation page")
    commit = commands.add_parser("commit", help="copy the cached results into results/<library>")
    commit.add_argument("--library", choices=LIBRARIES, action="append")
    arguments = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")

    COMMANDS[arguments.command](arguments)


def _list(arguments: argparse.Namespace) -> None:
    list_cases(arguments.family)


def _generate(arguments: argparse.Namespace) -> None:
    for path in write_cases(arguments.directory):
        print(path)


def _run(arguments: argparse.Namespace) -> None:
    failed = []
    for library in arguments.library or LIBRARIES:
        results = run_cases(
            selected_cases(arguments.cases, arguments.family), library, arguments.force, arguments.workers
        )
        for case_id, outcome in results.items():
            if isinstance(outcome, CaseResult):
                kpis = ", ".join(f"{key}={value:.3g}" for key, value in sorted(outcome.kpis.items()))
                print(f"{library} {case_id}: {kpis}")
            else:
                failed.append(f"{library} {case_id}: {outcome}")
    if failed:
        raise SystemExit("\n".join(failed))


def _report(arguments: argparse.Namespace) -> None:
    report_results(arguments.directory, arguments.docs)


def _commit(arguments: argparse.Namespace) -> None:
    for library in arguments.library or LIBRARIES:
        for path in commit_results(library):
            print(path)


COMMANDS = {"list": _list, "generate": _generate, "run": _run, "report": _report, "commit": _commit}


if __name__ == "__main__":
    main()
