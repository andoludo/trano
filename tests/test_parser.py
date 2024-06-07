import re
from pathlib import Path

from neosim.controller.parser import (
    BooleanInput,
    BooleanOutput,
    ControllerBus,
    IntegerInput,
    IntegerOutput,
    RealInput,
    RealOutput,
)
from neosim.models.elements.ahu import AirHandlingUnit
from neosim.models.elements.base import DynamicComponentTemplate
from neosim.models.elements.bus import DataBus
from neosim.models.elements.control import SpaceControl
from neosim.models.elements.controls.ahu import AhuControl
from neosim.models.elements.damper import VAV
from neosim.models.elements.space import Space


def get_first_set_before_break_or_parenthesis(text):
    match = re.search(r"[^\n\(\) ]+", text)
    if match:
        return match.group(0)
    else:
        return None


def test_parse_controller():
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
    import json

    Path("/home/aan/Documents/neosim/neosim/controller/vav_controller.json").write_text(
        json.dumps(cb.model_dump(), indent=4)
    )
    a = 12


def test_parse_controller_aku():
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
    a = 12


def test_complex_ahu():
    ahu = AirHandlingUnit(name="ahu_2")

    component = dynamic_template.render("test", ahu)
    a = 12


def test_ahu_controller():
    ahu_control = AhuControl(name="ahu_control")
    d

    component = dynamic_template.render("test", ahu_control)
    a = 12


def test_vav_controller():
    space_control = SpaceControl(
        name="space_control",
        controllable_element=VAV(name="vav"),
        space=Space(
            name="space_1",
            volume=10,
            height=1,
            floor_area=1,
            elevation=1,
            external_boundaries=[],
        ),
    )

    component = dynamic_template.render("test", space_control)
    a = 12


def test_ew_dynamic_component_template():
    vav = VAV(name="vav")

    component = dynamic_template.render("test", vav)
    a = 12


def test_data_bus():
    data_bus = DataBus(name="bus", spaces=["room1", "room2", "room3"])

    dynamic_template.render("test", data_bus)
