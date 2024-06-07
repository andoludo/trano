import re
from pathlib import Path
from typing import Type

from pydantic import AliasChoices, BaseModel, ConfigDict, Field, create_model

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
from neosim.models.elements.space import Space, SpaceParameter


def get_first_set_before_break_or_parenthesis(text):
    match = re.search(r"[^\n\(\) ]+", text)
    if match:
        return match.group(0)
    else:
        return None


def test_parse_radiator():
    controller = Path(
        "/home/aan/Documents/neosim/Buildings/Fluid/HeatExchangers/Radiators/RadiatorEN442_2.mo"
    )
    p = []
    for path in [controller]:
        p += get_parameters(path)
    p


def get_parameters(controller):
    parameters = [
        x.strip()
        for x in controller.read_text()
        .split("equation")[0]
        .split("protected")[0]
        .split(";")
        if "parameter" in x
    ]

    r = []
    for p in parameters:
        documentation = None
        default = None
        matches = None
        try:
            documentation = re.findall(r'"(.*?)"', p)[0]
        except:
            pass
        try:
            matches = (
                re.findall(r"parameter(.*?)(\"|\n)", p)[0][0]
                .strip()
                .split(" ")[1]
                .split("(")[0]
                .split("=")[0]
            )
        except:
            continue
        defaults = re.findall(r"parameter(.*?)(\"|\n)", p)[0][0].strip().split("=")
        if len(defaults) > 1:
            default = defaults[1].strip()
        r.append((matches, default, documentation))

    return r


def test_parse_valve():
    p1 = Path(
        "/home/aan/Documents/neosim/Buildings/Fluid/Actuators/Valves/TwoWayEqualPercentage.mo"
    )
    p2 = Path(
        "/home/aan/Documents/neosim/Buildings/Fluid/Actuators/BaseClasses/PartialTwoWayValve.mo"
    )
    p3 = Path(
        "/home/aan/Documents/neosim/Buildings/Fluid/BaseClasses/PartialResistance.mo"
    )
    p4 = Path(
        "/home/aan/Documents/neosim/Buildings/Fluid/Actuators/BaseClasses/ValveParameters.mo"
    )
    p = []
    for path in [p1, p2, p3, p4]:
        p += get_parameters(path)
    p


def test_split():
    p1 = Path("/home/aan/Documents/neosim/Buildings/Fluid/FixedResistances/Junction.mo")
    p2 = Path(
        "/home/aan/Documents/neosim/Buildings/Fluid/BaseClasses/PartialThreeWayResistance.mo"
    )
    p = []
    for path in [p1, p2]:
        p += get_parameters(path)
    p


def test_pump():
    p1 = Path(
        "/home/aan/Documents/neosim/Buildings/Fluid/Movers/Preconfigured/SpeedControlled_y.mo"
    )
    p2 = Path("/home/aan/Documents/neosim/Buildings/Fluid/Movers/SpeedControlled_y.mo")
    p3 = Path(
        "/home/aan/Documents/neosim/Buildings/Fluid/Movers/BaseClasses/PartialFlowMachine.mo"
    )
    p = []
    pr = []
    for path in [p1, p2, p3]:
        # p += get_parameters(path)
        pr += [
            x.strip()
            for x in path.read_text()
            .split("equation")[0]
            .split("protected")[0]
            .split(";")
            if "parameter" in x
        ]

    p


def test_three_way_valve():
    p1 = Path(
        "/home/aan/Documents/neosim/Buildings/Fluid/Actuators/Valves/ThreeWayEqualPercentageLinear.mo"
    )
    p2 = Path(
        "/home/aan/Documents/neosim/Buildings/Fluid/Actuators/BaseClasses/PartialThreeWayValve.mo"
    )
    p4 = Path(
        "/home/aan/Documents/neosim/Buildings/Fluid/Actuators/BaseClasses/ValveParameters.mo"
    )

    p = []
    pr = []
    for path in [p1, p2, p4]:
        # p += get_parameters(path)
        pr += [
            x.strip()
            for x in path.read_text()
            .split("equation")[0]
            .split("protected")[0]
            .split(";")
            if "parameter" in x
        ]

    p


def test_boiler():
    p1 = Path("/home/aan/Documents/neosim/Buildings/Fluid/Boilers/BoilerPolynomial.mo")
    p2 = Path(
        "/home/aan/Documents/neosim/Buildings/Fluid/Boilers/BaseClasses/PartialBoiler.mo"
    )
    p3 = Path(
        "/home/aan/Documents/neosim/Buildings/Fluid/Interfaces/TwoPortFlowResistanceParameters.mo"
    )
    p4 = Path(
        "/home/aan/Documents/neosim/Buildings/Fluid/Interfaces/PartialTwoPortInterface.mo"
    )
    p = []
    pr = []
    for path in [p1, p2, p3, p4]:
        # p += get_parameters(path)
        pr += [
            x.strip()
            for x in path.read_text()
            .split("equation")[0]
            .split("protected")[0]
            .split(";")
            if "parameter" in x
        ]

    p


def test_storage():
    p1 = Path(
        "/home/aan/Documents/neosim/Buildings/Fluid/Storage/BaseClasses/PartialStratified.mo"
    )
    p4 = Path(
        "/home/aan/Documents/neosim/Buildings/Fluid/Interfaces/PartialTwoPortInterface.mo"
    )

    p = []
    pr = []
    for path in [p1, p4]:
        # p += get_parameters(path)
        pr += [
            x.strip()
            for x in path.read_text()
            .split("equation")[0]
            .split("protected")[0]
            .split(";")
            if "parameter" in x
        ]

    p


class Parameters(BaseModel):
    name: str = Field(validation_alias=AliasChoices("last_name", "lname"))
    test: str = Field(validation_alias=AliasChoices("test", "tete"))
    model_config = ConfigDict(populate_by_name=True)


def test_dump():
    param = Parameters(name="tests", test="tests")
    param.model_dump(by_alias=True)


def test_parameters():
    a = SpaceParameter()
    test = create_model(
        "test", **change_alias(SpaceParameter, {"floor_area": "coco"})
    )()
    a = 12
