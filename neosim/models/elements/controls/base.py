from typing import List, Optional

from pydantic import Field

from neosim.models.elements.base import BaseElement, BaseParameter


class Control(BaseElement):
    name: str
    position: Optional[List[float]] = None
    controllable_element: Optional[BaseElement] = None
    space_name: Optional[str] = None


class ControlParameters(BaseParameter):
    ...


class PIDParameters(ControlParameters):
    controller_type: Optional[str] = Field(
        "Buildings.Controls.OBC.CDL.Types.SimpleController.PI",
        alias="controllerType",
        title="Type of controller",
    )
    k: Optional[float] = Field(
        1,
        alias="k",
        title="Gain of controller",
    )
    ti: Optional[float] = Field(
        0.5,
        alias="Ti",
        title="Time constant of integrator block",
    )
    td: Optional[float] = Field(
        0.1,
        alias="Td",
        title="Time constant of derivative block",
    )
    r: Optional[float] = Field(
        1,
        alias="r",
        title="Typical range of control error, used for scaling the control error",
    )
    y_max: Optional[float] = Field(
        1,
        alias="yMax",
        title="Upper limit of output",
    )
    y_min: Optional[float] = Field(
        0,
        alias="yMin",
        title="Lower limit of output",
    )
    ni: Optional[float] = Field(
        0.90,
        alias="Ni",
        title="Ni*Ti is time constant of anti-windup compensation",
    )
    nd: Optional[float] = Field(
        10,
        alias="Nd",
        title="The higher Nd, the more ideal the derivative block",
    )
