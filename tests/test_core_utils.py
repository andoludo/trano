"""Tests for trano.core.utils.is_success."""

from dataclasses import dataclass

from trano.core.utils import is_success
from trano.simulate.simulate import SimulationOptions


@dataclass
class _FakeResult:
    """Stand-in for docker.models.containers.ExecResult."""

    output: bytes


def test_is_success_recognises_success_marker() -> None:
    result = _FakeResult(output=b"... The simulation finished successfully.\n")
    assert is_success(result) is True  # type: ignore[arg-type]


def test_is_success_rejects_other_output() -> None:
    result = _FakeResult(output=b"some other log line\n")
    assert is_success(result) is False  # type: ignore[arg-type]


def test_is_success_check_only_mode_looks_for_true() -> None:
    options = SimulationOptions(check_only=True)
    assert is_success(_FakeResult(output=b"true"), options) is True  # type: ignore[arg-type]
    assert is_success(_FakeResult(output=b"false"), options) is False  # type: ignore[arg-type]
