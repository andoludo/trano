"""ASHRAE 140 / IEA SHC Task 22 BESTEST envelope validation suite.

Runs every case in validation.bestest.spec.parameters.CASES, compares the
extracted KPIs to the reference bands in validation/bestest/spec/reference.csv,
and writes a report to validation/bestest/_reports/.

Iteration 1: warn-only — KPI deviations beyond the reference bands surface as
``UserWarning`` rather than test failures. Iteration 2 will tighten this.

Skip behaviour:
- Module-level skip if Docker isn't reachable (suite needs OpenModelica via Docker).
- Per-case ``xfail`` for any case requiring HVAC heating+cooling or time-varying
  ventilation (see validation/bestest/README.md for the gap list).
"""
from __future__ import annotations

import warnings
from pathlib import Path

import pytest

from validation.bestest.harness import docker_available, run_case
from validation.bestest.report import compare_to_reference, load_reference_bands, write_report
from validation.bestest.spec.parameters import CASES, KPIBands, KPIResults

if not docker_available():
    pytest.skip(
        "Docker daemon not reachable; BESTEST requires OpenModelica via Docker.",
        allow_module_level=True,
    )

REPORT_DIR: Path = Path(__file__).resolve().parents[1] / "validation" / "bestest" / "_reports"

# Cases that iteration 1 actually expects to produce meaningful KPIs.
RUNNABLE_CASES: frozenset[str] = frozenset({"600FF", "900FF"})


@pytest.fixture(scope="session")
def reference_bands() -> dict[str, list[KPIBands]]:
    return load_reference_bands()


@pytest.fixture(scope="session")
def all_results(reference_bands: dict[str, list[KPIBands]]) -> dict[str, KPIResults]:
    """Run every BESTEST case once per session; emit the report at teardown."""
    results: dict[str, KPIResults] = {}
    for case_id in sorted(CASES):
        try:
            results[case_id] = run_case(case_id)
        except Exception as exc:
            warnings.warn(
                f"BESTEST case {case_id} failed to run: {exc}",
                UserWarning,
                stacklevel=2,
            )
    if results:
        write_report(results, reference_bands, REPORT_DIR)
    return results


def _xfail_marks(case_id: str) -> list[pytest.MarkDecorator]:
    if case_id in RUNNABLE_CASES:
        return []
    return [
        pytest.mark.xfail(
            reason="iteration 1: needs Trano core extensions for cooling control "
            "and/or time-varying ventilation; see validation/bestest/README.md",
            strict=False,
        ),
    ]


@pytest.mark.bestest
@pytest.mark.slow
@pytest.mark.parametrize(
    "case_id",
    [pytest.param(cid, marks=_xfail_marks(cid)) for cid in sorted(CASES)],
)
def test_bestest_case(
    case_id: str,
    all_results: dict[str, KPIResults],
    reference_bands: dict[str, list[KPIBands]],
) -> None:
    if case_id not in all_results:
        pytest.fail(f"case {case_id}: simulation did not produce a result")
    statuses = compare_to_reference(all_results[case_id], reference_bands.get(case_id, []))
    out_of_band = {k: v for k, v in statuses.items() if v not in {"pass", "no_band"}}
    if out_of_band:
        warnings.warn(
            f"BESTEST case {case_id} out of band: {out_of_band}",
            UserWarning,
            stacklevel=2,
        )
