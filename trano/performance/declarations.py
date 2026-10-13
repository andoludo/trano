"""Modifiers added to the component declarations of a generated Modelica model.

A performance option applies to every declaration of a class of the Buildings, IDEAS or AixLib library,
whether it was rendered by a component template or belongs to the Trano package embedded in the model.
Rewriting the declarations once, after rendering, keeps the templates free of the options and treats the
three libraries, whose parameters have different names, in one place.

A declaration that already sets the parameter is left as it is: a value written by a template or by the
user always wins over the option.
"""

import re
from collections.abc import Iterator
from dataclasses import dataclass

LIBRARIES = ("Buildings", "IDEAS", "AixLib")
KEYWORDS = frozenset({"annotation", "constrainedby", "extends", "import", "if", "then", "else", "end", "each"})
PRECEDING_SKIP = frozenset({"extends", "constrainedby", "import"})


@dataclass(frozen=True)
class DeclarationRule:
    """Set a parameter on every declared component of the classes matched by ``classes``.

    ``classes`` is a regular expression of the class path below the library root, ``parameters`` the name
    of the parameter in each library (a library missing from it is not modified).
    """

    classes: str
    parameters: dict[str, str]
    value: str

    def pattern(self) -> re.Pattern[str]:
        libraries = "|".join(re.escape(library) for library in self.parameters)
        classes = self.classes.replace(r"\.", r"\s*\.\s*")  # a class path may be split over lines
        return re.compile(
            rf"(?<![\w.])(?P<library>{libraries})\s*\.\s*(?:{classes})"
            r"\s+(?P<name>[A-Za-z_]\w*)(?:\s*(?P<dimensions>\[[^\]]*\]))?"
        )


ACTUATOR_FILTER = DeclarationRule(
    classes=r"Fluid\.Actuators\.(?:Valves|Dampers)\.\w+",
    parameters={"Buildings": "use_strokeTime", "AixLib": "use_strokeTime", "IDEAS": "use_inputFilter"},
    value="false",
)
MOVER_FILTER = DeclarationRule(
    classes=(
        r"Fluid\.Movers\.(?:Preconfigured\.)?"
        r"(?:SpeedControlled_y|SpeedControlled_Nrpm|FlowControlled_m_flow|FlowControlled_dp)\b"
    ),
    parameters={"Buildings": "use_riseTime", "AixLib": "use_riseTime", "IDEAS": "use_inputFilter"},
    value="false",
)
SENSOR_DYNAMICS = DeclarationRule(
    classes=(
        r"Fluid\.Sensors\.(?:Temperature|RelativeHumidity|MassFraction|TraceSubstances|SpecificEnthalpy|"
        r"SpecificEntropy|Density|PPM)TwoPort\b"
    ),
    parameters={"Buildings": "tau", "AixLib": "tau", "IDEAS": "tau"},
    value="0",
)

_LITERAL_OR_COMMENT = re.compile(r'"(?:\\.|[^"\\])*"|//[^\n]*|/\*.*?\*/', re.DOTALL)


def _masked(text: str) -> str:
    """The text with string literals and comments blanked out (same length), so they are never matched."""
    return _LITERAL_OR_COMMENT.sub(lambda match: " " * len(match.group()), text)


def _closing_parenthesis(masked: str, opening: int) -> int:
    depth = 0
    for index in range(opening, len(masked)):
        if masked[index] == "(":
            depth += 1
        elif masked[index] == ")":
            depth -= 1
            if depth == 0:
                return index
    raise ValueError(f"Unbalanced parentheses in the declaration starting at {opening}")


def _top_level(modifiers: str) -> str:
    """The modifiers without the content of nested parentheses and braces."""
    previous = None
    while previous != modifiers:
        previous = modifiers
        modifiers = re.sub(r"\([^()]*\)|\{[^{}]*\}", "", modifiers)
    return modifiers


def _sets(modifiers: str, parameter: str) -> bool:
    return re.search(rf"(?<![\w.]){re.escape(parameter)}\s*=", _top_level(modifiers)) is not None


def _preceding_word(masked: str, position: int) -> str:
    match = re.search(r"([A-Za-z_]\w*)\s*$", masked[:position])
    return match.group(1) if match else ""


@dataclass(frozen=True)
class _Insertion:
    position: int
    text: str


def _insertions(text: str, masked: str, rule: DeclarationRule) -> Iterator[_Insertion]:
    for match in rule.pattern().finditer(masked):
        if match.group("name") in KEYWORDS or _preceding_word(masked, match.start()) in PRECEDING_SKIP:
            continue
        parameter = rule.parameters[match.group("library")]
        each = "each " if match.group("dimensions") else ""
        modifier = f"{each}{parameter}={rule.value}"
        after = match.end()
        while after < len(masked) and masked[after].isspace():
            after += 1
        if after < len(masked) and masked[after] == "(":
            closing = _closing_parenthesis(masked, after)
            if _sets(masked[after + 1 : closing], parameter):
                continue
            separator = ", " if text[after + 1 : closing].strip() else ""
            yield _Insertion(after + 1, f"{modifier}{separator}")
        else:
            yield _Insertion(match.end(), f"({modifier})")


def apply_rule(text: str, rule: DeclarationRule) -> tuple[str, int]:
    """The text with the rule's parameter set on every matching declaration, and the number of changes."""
    insertions = sorted(_insertions(text, _masked(text), rule), key=lambda insertion: insertion.position)
    pieces: list[str] = []
    start = 0
    for insertion in insertions:
        pieces += [text[start : insertion.position], insertion.text]
        start = insertion.position
    pieces.append(text[start:])
    return "".join(pieces), len(insertions)
