"""Parameter overrides of an OpenModelica executable and the messages it logs about them.

The executable takes new parameter values with ``-override=name=value,...`` or, for many values, with
``-overrideFile=<file>`` holding one ``name=value`` per line; both are merged, the file winning. The
experiment settings go through their own flags (``-startTime=``, ``-stopTime=``, ``-stepSize=``,
``-tolerance=``, ``-variableFilter=``): OpenModelica 1.26 reports ``-override=stopTime=...`` as "override variable
name not found in model" and keeps the stop time of the init file.

The executable does not stop on an override it cannot apply: it logs a warning and runs with the value
of the init file. :func:`parse_override_messages` collects these warnings so that an ignored override
never passes as a change. It does not catch everything: a free parameter the translation used to size
arrays takes the override without a warning and without effect, which only the run-time checks reveal.
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


__all__ = [
    "EXPERIMENT_SETTINGS",
    "IgnoredOverrideError",
    "OverrideIssue",
    "OverrideMessage",
    "OverrideValue",
    "experiment_flags",
    "format_value",
    "ignored_overrides",
    "override_lines",
    "parse_override_messages",
    "write_override_file",
]
