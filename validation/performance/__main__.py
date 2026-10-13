"""Command line of the performance benchmark: ``python -m validation.performance --help``.

Needs Docker and the OpenModelica image of trano. Set ``TRANO_MODELICA_LIBRARIES`` to a directory of
installed Modelica libraries to run without network access (see trano.simulate).
"""

from __future__ import annotations

import argparse
import logging
from pathlib import Path

from validation.performance.accuracy import Comparison, compare
from validation.performance.cases import LIBRARIES, MODELS, PROFILES, select_cases
from validation.performance.report import render
from validation.performance.runner import Settings, run_cases

DEFAULT_DIRECTORY = Path(__file__).resolve().parents[2].joinpath(".cache", "performance")


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(prog="python -m validation.performance", description=__doc__)
    parser.add_argument("--models", nargs="*", choices=[model.name for model in MODELS], help="all when omitted")
    parser.add_argument("--libraries", nargs="*", choices=LIBRARIES)
    parser.add_argument("--profiles", nargs="*", choices=list(PROFILES), help="baseline and fast when omitted")
    parser.add_argument("--include-slow", action="store_true", help="also the models taking minutes per week")
    parser.add_argument("--days", type=int, default=7, help="simulated days from 1 January")
    parser.add_argument("--timeout", type=int, default=3600, help="[s] wall clock allowed to one simulation")
    parser.add_argument("--directory", type=Path, default=DEFAULT_DIRECTORY, help="models, logs and results")
    parser.add_argument("--report", type=Path, help="markdown report, <directory>/report.md by default")
    parser.add_argument("--force", action="store_true", help="run again the cases whose model did not change")
    parser.add_argument("--reuse", action="store_true", help="take every stored successful run (report again)")
    arguments = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(message)s")

    profiles = arguments.profiles or ["baseline", "fast"]
    if "baseline" not in profiles:
        profiles = ["baseline", *profiles]  # every profile is compared with the baseline
    cases = select_cases(arguments.models, arguments.libraries, profiles, include_slow=arguments.include_slow)
    settings = Settings(days=arguments.days, timeout=arguments.timeout)
    runs = run_cases(cases, arguments.directory.resolve(), settings, force=arguments.force, reuse=arguments.reuse)
    by_id = {run.case_id: run for run in runs}
    comparisons: list[Comparison] = []
    for case in cases:
        run, baseline = by_id[case.id], by_id.get(case.baseline.id)
        if case.profile.name == "baseline" or baseline is None or not (run.result_file and baseline.result_file):
            continue
        comparisons.append(compare(case.id, Path(baseline.result_file), Path(run.result_file)))
    report = render(cases, runs, comparisons, settings)
    path = arguments.report or arguments.directory.joinpath("report.md")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(report)
    print(report)


if __name__ == "__main__":
    main()
