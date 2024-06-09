import json
import re
from pathlib import Path
from typing import Optional

import pytest

from neosim.controller.parser import (
    BooleanInput,
    BooleanOutput,
    ControllerBus,
    IntegerInput,
    IntegerOutput,
    RealInput,
    RealOutput,
)


def get_first_set_before_break_or_parenthesis(text: str) -> Optional[str]:
    match = re.search(r"[^\n\(\) ]+", text)
    if match:
        return match.group(0)
    else:
        return None


@pytest.mark.skip("not a test")
def test_parse_controller() -> None:
    controller = Path(
        "/home/aan/Documents/neosim/Buildings/Controls/OBC/ASHRAE/G36/TerminalUnits/Reheat/Controller.mo"
    )
    control = [x for x in controller.read_text().split(";") if "Interfaces" in x]
    real_input = [
        get_first_set_before_break_or_parenthesis(c.split("RealInput")[-1].strip())
        for c in control
        if "RealInput" in c
    ]
    real_output = [
        get_first_set_before_break_or_parenthesis(c.split("RealOutput")[-1].strip())
        for c in control
        if "RealOutput" in c
    ]
    integer_input = [
        get_first_set_before_break_or_parenthesis(c.split("IntegerInput")[-1].strip())
        for c in control
        if "IntegerInput" in c
    ]
    boolean_input = [
        get_first_set_before_break_or_parenthesis(c.split("BooleanInput")[-1].strip())
        for c in control
        if "BooleanInput" in c
    ]
    boolean_output = [
        get_first_set_before_break_or_parenthesis(c.split("BooleanOutput")[-1].strip())
        for c in control
        if "BooleanOutput" in c
    ]
    integer_output = [
        get_first_set_before_break_or_parenthesis(c.split("IntegerOutput")[-1].strip())
        for c in control
        if "IntegerOutput" in c
    ]
    for output in integer_output + boolean_output + real_output:
        f"connect(databus.{output}, {output});"
    cb = ControllerBus(
        real_inputs=[
            RealInput(
                name=x, target="element.space.name", component="rehBoxCon", port=x
            )
            for x in real_input
        ],
        real_outputs=[
            RealOutput(
                name=x,
                target="element.controllable_element.name",
                component="rehBoxCon",
                port=x,
            )
            for x in real_output
        ],
        integer_inputs=[
            IntegerInput(
                name=x, target="element.space.name", component="rehBoxCon", port=x
            )
            for x in integer_input
        ],
        integer_outputs=[
            IntegerOutput(
                name=x,
                target="element.controllable_element.name",
                component="rehBoxCon",
                port=x,
            )
            for x in integer_output
        ],
        boolean_inputs=[
            BooleanInput(
                name=x, target="element.space.name", component="rehBoxCon", port=x
            )
            for x in boolean_input
        ],
        boolean_outputs=[
            BooleanOutput(
                name=x,
                target="element.controllable_element.name",
                component="rehBoxCon",
                port=x,
            )
            for x in boolean_output
        ],
    )

    Path("/home/aan/Documents/neosim/neosim/controller/vav_controller.json").write_text(
        json.dumps(cb.model_dump(), indent=4)
    )


@pytest.mark.skip("not a test")
def test_parse_controller_ahu() -> None:
    controller = Path(
        "/home/aan/Documents/neosim/Buildings/Controls/OBC/ASHRAE/G36/AHUs/MultiZone/VAV/Controller.mo"
    )
    control = [x for x in controller.read_text().split(";") if "Interfaces" in x]
    real_input = [
        get_first_set_before_break_or_parenthesis(c.split("RealInput")[-1].strip())
        for c in control
        if "RealInput" in c
    ]
    real_output = [
        get_first_set_before_break_or_parenthesis(c.split("RealOutput")[-1].strip())
        for c in control
        if "RealOutput" in c
    ]
    integer_input = [
        get_first_set_before_break_or_parenthesis(c.split("IntegerInput")[-1].strip())
        for c in control
        if "IntegerInput" in c
    ]
    boolean_input = [
        get_first_set_before_break_or_parenthesis(c.split("BooleanInput")[-1].strip())
        for c in control
        if "BooleanInput" in c
    ]
    boolean_output = [
        get_first_set_before_break_or_parenthesis(c.split("BooleanOutput")[-1].strip())
        for c in control
        if "BooleanOutput" in c
    ]
    integer_output = [
        get_first_set_before_break_or_parenthesis(c.split("IntegerOutput")[-1].strip())
        for c in control
        if "IntegerOutput" in c
    ]
    for output in integer_output + boolean_output + real_output:
        f"connect(databus.{output}, {output});"
    cb = ControllerBus(
        real_inputs=[
            RealInput(name=x, target="element.name", component="mulAHUCon", port=x)
            for x in real_input
        ],
        real_outputs=[
            RealOutput(
                name=x,
                target="element.controllable_element.name",
                component="mulAHUCon",
                port=x,
            )
            for x in real_output
        ],
        integer_inputs=[
            IntegerInput(name=x, target="element.name", component="mulAHUCon", port=x)
            for x in integer_input
        ],
        integer_outputs=[
            IntegerOutput(
                name=x,
                target="element.controllable_element.name",
                component="mulAHUCon",
                port=x,
            )
            for x in integer_output
        ],
        boolean_inputs=[
            BooleanInput(name=x, target="element.name", component="mulAHUCon", port=x)
            for x in boolean_input
        ],
        boolean_outputs=[
            BooleanOutput(
                name=x,
                target="element.controllable_element.name",
                component="mulAHUCon",
                port=x,
            )
            for x in boolean_output
        ],
    )
    import json

    Path("/home/aan/Documents/neosim/neosim/controller/ahu_controller.json").write_text(
        json.dumps(cb.model_dump(), indent=4)
    )
