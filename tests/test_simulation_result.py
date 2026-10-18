"""The typed result of a run, classified from OpenModelica's log."""

from pathlib import Path

import pytest

from tests.fixtures.fake_omc import OMC_LOGS
from trano.simulate.result import LOG_LIMIT, SimulationErrorKind, SimulationResult, classify_log


def omc_log(name: str) -> str:
    return OMC_LOGS.joinpath(name).read_text()


@pytest.mark.parametrize(
    ("log", "kind", "message"),
    [
        ("success.log", None, None),
        (
            "translation_error.log",
            SimulationErrorKind.translation,
            "Variable undefinedThing not found in scope building.",
        ),
        (
            "solver_assertion.log",
            SimulationErrorKind.solver,
            'The following assertion has been violated at time 1.000000 ((x < 2.0)) --> "x went above 2"',
        ),
        (
            "solver_missing_weather.log",
            SimulationErrorKind.solver,
            'Not possible to open file "//modelica://IDEAS/Resources/weatherdata/BEL_VLG_Uccle.064470_TMYx.2007-2021.mos"'
            " for reading: No such file or directory",
        ),
    ],
)
def test_a_run_is_classified_from_its_log(log: str, kind: SimulationErrorKind | None, message: str | None) -> None:
    result = SimulationResult.from_log(omc_log(log))

    assert result.success is (kind is None)
    assert result.error_kind is kind
    assert result.error_message == message


def test_a_check_succeeds_on_the_verdict_of_check_model() -> None:
    assert classify_log(omc_log("solver_assertion.log"), check_only=True) is None
    assert classify_log(omc_log("translation_error.log"), check_only=True) is SimulationErrorKind.translation


def test_an_empty_log_is_a_translation_failure() -> None:
    result = SimulationResult.from_log("")

    assert result.error_kind is SimulationErrorKind.translation
    assert result.error_message == "OpenModelica failed to translate the model; see the log."


def test_the_result_file_is_kept_only_when_it_exists(tmp_path: Path) -> None:
    result_path = tmp_path / "given.building_res.mat"
    assert SimulationResult.from_log(omc_log("success.log"), result_path=result_path).result_path is None
    result_path.write_bytes(b"")
    assert SimulationResult.from_log(omc_log("success.log"), result_path=result_path).result_path == result_path
    failed = SimulationResult.from_log(omc_log("solver_assertion.log"), result_path=result_path)
    assert failed.result_path is None


def test_the_log_keeps_its_end() -> None:
    log = "x" * LOG_LIMIT + omc_log("success.log")

    result = SimulationResult.from_log(log)

    assert result.success
    assert len(result.log) == LOG_LIMIT
    assert result.log.endswith(omc_log("success.log"))


def test_a_result_reads_like_a_docker_exec_result() -> None:
    success = SimulationResult.from_log(omc_log("success.log"))
    failure = SimulationResult.from_log(omc_log("solver_assertion.log"))

    assert (success.exit_code, failure.exit_code) == (0, 1)
    assert success.output == omc_log("success.log").encode()


def test_a_result_is_serialisable() -> None:
    result = SimulationResult.from_log(omc_log("solver_assertion.log"), duration_s=1.5)

    assert SimulationResult.model_validate_json(result.model_dump_json()) == result
