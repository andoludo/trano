"""ASHRAE 140 (BESTEST) validation: simulate the cases and compare the KPIs with the reference.

Year-long simulations in Docker: run with ``pytest -m bestest`` (``make bestest``). Results are cached
in ``.cache/bestest`` by the generated model, so an unchanged case is not simulated again.
"""

import pytest

from validation.bestest.cases import CASES, UnsupportedCaseError, building_description
from validation.bestest.harness import run_case
from validation.bestest.reference import load_reference
from validation.bestest.report import compare

# Cases expected to pass per library; the others are skipped with the feature they wait for.
EXPECTED_TO_PASS: dict[str, set[str]] = {
    "Buildings": {"600FF", "900FF", "680FF", "980FF"},
}


def parameters() -> list[pytest.param]:  # type: ignore[valid-type]
    cases = []
    for library, expected in EXPECTED_TO_PASS.items():
        for case in CASES.values():
            marks = []
            try:
                building_description(case)
            except UnsupportedCaseError as error:
                marks.append(pytest.mark.skip(reason=str(error)))
            if case.id not in expected and not marks:
                marks.append(pytest.mark.xfail(reason=f"{case.id} not validated with {library} yet", strict=False))
            cases.append(pytest.param(case.id, library, id=f"{case.id}-{library}", marks=marks))
    return cases


@pytest.mark.bestest
@pytest.mark.parametrize(("case_id", "library"), parameters())
def test_case_matches_the_reference(case_id: str, library: str) -> None:
    result = run_case(case_id, library)
    comparisons = compare(result.kpis, load_reference())

    assert comparisons, "no reference data for the case"
    failures = [
        f"{c.kpi}: {c.value:.3f} {c.unit} outside [{c.lower:.3f}, {c.upper:.3f}]"
        for c in comparisons
        if c.status != "pass"
    ]
    assert not failures, "\n".join(failures)
