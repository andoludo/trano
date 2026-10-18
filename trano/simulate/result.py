"""The outcome of a simulation or of a model check, read from the OpenModelica log."""

import re
from enum import StrEnum
from pathlib import Path

from pydantic import BaseModel, Field

LOG_LIMIT = 20_000
SIMULATION_SUCCESS = "The simulation finished successfully"
CHECK_SUCCESS = "completed successfully"
# the messages of the record printed by OpenModelica's ``simulate``
EXECUTION_FAILED = "Simulation execution failed"
BUILD_FAILED = "Failed to build model"
ERROR_LINE = re.compile(r"(?:\[[^\]]*\]\s*)?Error:\s*(?P<message>.+)")
MODEL_ERROR_LINE = re.compile(r"\[[^\]]*\.mo:[^\]]*\]\s*Error:\s*(?P<message>.+)")  # an error located in a model
# a log entry of the simulation executable, e.g. ``LOG_ASSERT | info | [file.mo:1:2] <message>``, and the lines
# continuing it, which start with ``|``
SOLVER_ENTRY = re.compile(r"^LOG_(?:ASSERT|STDOUT)\s*\|\s*\w+\s*\|(?P<message>.*(?:\n\|.*)*)", re.MULTILINE)
CONTINUATION = re.compile(r"^\|\s*\|\s*\|\s*\|", re.MULTILINE)
SOURCE_LOCATION = re.compile(r"\[[^\]]*\.mo:[^\]]*\]")


class SimulationErrorKind(StrEnum):
    validation = "validation"  # the YAML model does not satisfy the schema or the model checks
    translation = "translation"  # OpenModelica failed to load, check or build the model
    solver = "solver"  # the model was built but its simulation failed
    timeout = "timeout"
    runtime = "runtime"  # the simulation environment failed (OpenModelica missing, container error, ...)


class SimulationResult(BaseModel):
    success: bool
    result_path: Path | None = None
    log: str = Field(default="", description=f"OpenModelica's output, truncated to the last {LOG_LIMIT} characters.")
    duration_s: float = 0.0
    error_kind: SimulationErrorKind | None = None
    error_message: str | None = Field(default=None, description="One human-readable line.")

    @property
    def output(self) -> bytes:
        """The log as bytes, like the ``output`` of docker's ``ExecResult`` returned by the docker runtime."""
        return self.log.encode()

    @property
    def exit_code(self) -> int:
        """0 on success, 1 otherwise, like the ``exit_code`` of docker's ``ExecResult``."""
        return 0 if self.success else 1

    @classmethod
    def failure(
        cls, kind: SimulationErrorKind, message: str, log: str = "", duration_s: float = 0.0
    ) -> "SimulationResult":
        return cls(
            success=False,
            log=truncate_log(log),
            duration_s=duration_s,
            error_kind=kind,
            error_message=message,
        )

    @classmethod
    def from_log(
        cls, log: str, check_only: bool = False, result_path: Path | None = None, duration_s: float = 0.0
    ) -> "SimulationResult":
        """The result of an OpenModelica run of a ``.mos`` script written by ``create_mos_file``.

        ``result_path`` is kept only for a successful simulation and when the file exists.
        """
        kind = classify_log(log, check_only=check_only)
        if kind is None:
            return cls(
                success=True,
                result_path=None if check_only or result_path is None or not result_path.exists() else result_path,
                log=truncate_log(log),
                duration_s=duration_s,
            )
        return cls.failure(kind, error_message(log, kind), log=log, duration_s=duration_s)


def truncate_log(log: str, limit: int = LOG_LIMIT) -> str:
    return log[-limit:]


def classify_log(log: str, check_only: bool = False) -> SimulationErrorKind | None:
    """The kind of error the log reports, None on success.

    The markers are those of ``trano.utils.utils.is_success``: every ``loadModel`` prints "true", so only
    the verdict of ``checkModel`` (check only) or of the simulation executable tells whether it worked.
    """
    if check_only:
        return None if CHECK_SUCCESS in log else SimulationErrorKind.translation
    if SIMULATION_SUCCESS in log:
        return None
    if EXECUTION_FAILED in log or (CHECK_SUCCESS in log and BUILD_FAILED not in log):
        return SimulationErrorKind.solver
    return SimulationErrorKind.translation


def error_message(log: str, kind: SimulationErrorKind) -> str:
    """The first error of the log, on one line; a generic sentence when the log names none."""
    if kind is SimulationErrorKind.solver and (entry := SOLVER_ENTRY.search(log)):
        message = SOURCE_LOCATION.sub("", CONTINUATION.sub("", entry.group("message")))
        return _one_line(message.replace('\\"', '"'))
    if match := MODEL_ERROR_LINE.search(log) or ERROR_LINE.search(log):
        return _one_line(match.group("message"))
    if kind is SimulationErrorKind.translation:
        return "OpenModelica failed to translate the model; see the log."
    return "The simulation failed; see the log."


def _one_line(text: str) -> str:
    return " ".join(text.split())
