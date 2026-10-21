"""Parameter overrides of an OpenModelica executable and the messages it logs about them.

The executable takes new parameter values with ``-override=name=value,...`` or, for many values, with
``-overrideFile=<file>`` holding one ``name=value`` per line; both are merged, the file winning. The
experiment settings go through their own flags (``-startTime=``, ``-stopTime=``, ``-stepSize=``,
``-tolerance=``, ``-variableFilter=``): OpenModelica 1.26 reports ``-override=stopTime=...`` as "override variable
name not found in model" and keeps the stop time of the init file.

The executable does not stop on an override it cannot apply: it logs a warning and runs with the value
of the init file. :func:`parse_override_messages` collects these warnings so that an ignored override
never passes as a change. The values the simulation actually used are checked too: with ``-output=<names>``
the executable prints them at the end of the run, and :func:`verify_overrides` compares them with the values
set. Neither catches a free parameter the translation used to size arrays (it takes the override without
effect); the parameter guidelines list those.
"""

import enum
import math
import re
from collections.abc import Iterable, Mapping
from dataclasses import dataclass
from pathlib import Path

OverrideValue = float | int | bool | str

EXPERIMENT_SETTINGS = ("startTime", "stopTime", "stepSize", "tolerance", "variableFilter")
"""Experiment settings of a run, each given to the executable with its own flag (``-stopTime=...``)."""

_NOT_OVERRIDABLE = re.compile(r"It is not possible to override the following quantity: (?P<name>\S+)")
_NOT_FOUND = re.compile(r"override variable name not found in model: (?P<name>\S+)")
_REPEATED = re.compile(r"You are overriding variable: (?P<name>[^=\s]+)=\S* again")
_OUTPUT = re.compile(r"^time=[^,]*,(?P<values>.*)$", re.MULTILINE)
_OUTPUT_VALUE = re.compile(r"(?P<name>(?:[^=,\[]|\[[^\]]*\])+)=(?P<value>[^,]*)")
MAX_OUTPUT_ARGUMENT = 100_000
"""Longest ``-output=`` argument (characters); the system limits one argument to 128 KiB."""
RELATIVE_TOLERANCE = 1e-12  # the executable prints about 20 digits


class OverrideIssue(enum.StrEnum):
    not_overridable = "not_overridable"  # structural, final, protected, evaluated or bound parameter
    not_found = "not_found"  # no variable of that name in the executable
    repeated = "repeated"  # the same name given twice


@dataclass(frozen=True)
class OverrideMessage:
    name: str
    issue: OverrideIssue


class IgnoredOverrideError(RuntimeError):
    """The executable ignored overrides that were asked for."""

    def __init__(self, messages: Iterable[OverrideMessage]) -> None:
        self.messages = tuple(messages)
        listed = ", ".join(f"{message.name} ({message.issue})" for message in self.messages)
        super().__init__(f"Overrides ignored by the executable: {listed}")


def format_value(value: OverrideValue) -> str:
    """A value as the executable reads it: shortest exact form of floats, Modelica booleans."""
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError(f"An override must be finite, got {value}")
        return repr(value)
    return str(value)


def override_lines(overrides: Mapping[str, OverrideValue]) -> list[str]:
    for name in overrides:
        if not name or any(character in name for character in "=,\n ") or name in EXPERIMENT_SETTINGS:
            raise ValueError(f"Invalid name of a parameter override: {name!r}")
    return [f"{name}={format_value(value)}" for name, value in sorted(overrides.items())]


def write_override_file(path: Path, overrides: Mapping[str, OverrideValue]) -> Path:
    """Write the parameter overrides, one ``name=value`` per line, sorted by name."""
    path.write_text("\n".join(override_lines(overrides)) + "\n")
    return path


def experiment_flags(settings: Mapping[str, OverrideValue]) -> list[str]:
    """The flags of the experiment settings, ``-stopTime=86400`` for instance."""
    unknown = set(settings) - set(EXPERIMENT_SETTINGS)
    if unknown:
        raise ValueError(f"Unknown experiment settings {sorted(unknown)}, expected some of {EXPERIMENT_SETTINGS}")
    return [f"-{key}={format_value(value)}" for key, value in settings.items()]


def parse_override_messages(log: str) -> list[OverrideMessage]:
    """The overrides the executable reports as not applied, in the order of the log."""
    messages = []
    for line in log.splitlines():
        for pattern, issue in (
            (_NOT_OVERRIDABLE, OverrideIssue.not_overridable),
            (_NOT_FOUND, OverrideIssue.not_found),
            (_REPEATED, OverrideIssue.repeated),
        ):
            if match := pattern.search(line):
                messages.append(OverrideMessage(name=match.group("name"), issue=issue))
    return messages


def ignored_overrides(log: str, names: Iterable[str]) -> list[OverrideMessage]:
    """The messages about the given override names; experiment settings are not model variables."""
    wanted = set(names)
    return [message for message in parse_override_messages(log) if message.name in wanted]


def output_flag(names: Iterable[str]) -> str:
    """The flag making the executable print the values of ``names`` at the end of the run."""
    flag = "-output=" + ",".join(sorted(names))
    if len(flag) > MAX_OUTPUT_ARGUMENT:
        raise ValueError(f"Too many overrides to verify in one run ({len(flag)} characters of names)")
    return flag


def _number(value: str) -> float:
    text = value.strip()
    if text in ("true", "false"):
        return float(text == "true")
    return float(text)


def parse_output_values(log: str) -> dict[str, float]:
    """The values printed by ``-output`` at the end of the run (the last such line of the log)."""
    lines = _OUTPUT.findall(log)
    if not lines:
        return {}
    values = {}
    for match in _OUTPUT_VALUE.finditer(lines[-1]):
        try:
            values[match.group("name")] = _number(match.group("value"))
        except ValueError:
            continue
    return values


def read_override_file(path: Path) -> dict[str, float]:
    """The overrides of a file written by :func:`write_override_file`."""
    overrides = {}
    for line in path.read_text().splitlines():
        if line.strip():
            name, _, value = line.partition("=")
            overrides[name] = _number(value)
    return overrides


@dataclass(frozen=True)
class OverrideVerification:
    """The overrides of a run against what the executable reported and the values the simulation used."""

    expected: Mapping[str, float]
    used: Mapping[str, float]
    messages: tuple[OverrideMessage, ...] = ()

    @property
    def refused(self) -> tuple[OverrideMessage, ...]:
        return tuple(message for message in self.messages if message.name in self.expected)

    @property
    def differs(self) -> dict[str, tuple[float, float]]:
        """The overrides whose used value is not the value set: name -> (set, used)."""
        return {
            name: (float(value), self.used[name])
            for name, value in self.expected.items()
            if name in self.used
            and not math.isclose(float(value), self.used[name], rel_tol=RELATIVE_TOLERANCE, abs_tol=1e-15)
        }

    @property
    def missing(self) -> tuple[str, ...]:
        """The overrides the run did not report (run without ``-output``, or failed)."""
        return tuple(name for name in self.expected if name not in self.used)

    @property
    def ok(self) -> bool:
        return not (self.refused or self.differs or self.missing)

    def summary(self) -> str:
        if self.ok:
            return f"{len(self.expected)} overrides applied: the simulation used every value set"
        parts = []
        if self.refused:
            parts.append("refused: " + ", ".join(f"{message.name} ({message.issue})" for message in self.refused[:5]))
        if self.differs:
            parts.append(
                "used another value: "
                + ", ".join(
                    f"{name} (set {set_:g}, used {used:g})" for name, (set_, used) in list(self.differs.items())[:5]
                )
            )
        if self.missing:
            parts.append(f"not reported: {', '.join(self.missing[:5])}{' ...' if len(self.missing) > 5 else ''}")
        return "; ".join(parts)


class OverrideNotAppliedError(RuntimeError):
    """The simulation did not use the values set."""

    def __init__(self, verification: OverrideVerification) -> None:
        self.verification = verification
        super().__init__(f"Overrides not applied: {verification.summary()}")


def verify_overrides(log: str, overrides: Mapping[str, OverrideValue]) -> OverrideVerification:
    """Check the overrides of a run against its log (refusals and the ``-output`` values)."""
    expected = {name: float(value) for name, value in overrides.items() if not isinstance(value, str)}
    return OverrideVerification(expected, parse_output_values(log), tuple(parse_override_messages(log)))


__all__ = [
    "EXPERIMENT_SETTINGS",
    "IgnoredOverrideError",
    "OverrideIssue",
    "OverrideMessage",
    "OverrideNotAppliedError",
    "OverrideValue",
    "OverrideVerification",
    "experiment_flags",
    "format_value",
    "ignored_overrides",
    "output_flag",
    "override_lines",
    "parse_output_values",
    "parse_override_messages",
    "read_override_file",
    "verify_overrides",
    "write_override_file",
]
