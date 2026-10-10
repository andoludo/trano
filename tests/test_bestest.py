"""ASHRAE 140 (BESTEST) validation: simulate the cases and compare the KPIs with the reference.

Year-long simulations in Docker: run with ``pytest -m bestest`` (``make bestest``). Results are cached
in ``.cache/bestest`` by the generated model, so an unchanged case is not simulated again.
"""

import pytest

from validation.bestest.cases import CASES, UnsupportedCaseError, building_description, check_support
from validation.bestest.harness import run_case
from validation.bestest.reference import load_reference
from validation.bestest.report import compare, regressions

# Libraries whose cases must pass; a case not yet supported by the generator is skipped with the
# feature it waits for, and the deviations listed in ``report.KNOWN_DEVIATIONS`` are accepted.
GATING_LIBRARIES = ("Buildings", "IDEAS")


def parameters() -> list[pytest.param]:  # type: ignore[valid-type]
    cases = []
    for library in GATING_LIBRARIES:
        for case in CASES.values():
            marks = []
            try:
                building_description(case)
                check_support(case, library)
            except UnsupportedCaseError as error:
                marks.append(pytest.mark.skip(reason=str(error)))
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
        if c.status == "fail"
    ]
    assert not failures, "\n".join(failures)
    # The frozen values catch drifts that stay inside the bands: refreeze them on purpose.
    assert not (moved := regressions(library, case_id, result.kpis)), "\n".join(moved)
