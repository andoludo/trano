from enum import Enum

from typing import List, Literal, Optional

from pydantic import BaseModel, Field

DynamicTemplateCategories = Literal["ventilation", "control", "fluid", "boiler"]
SystemContainerTypes = Literal[
    "envelope", "distribution", "emission", "production", "ventilation"
]
ContainerTypes = Literal[SystemContainerTypes, "bus", "solar"]
Pattern = Literal["Solid", "Dot", "Dash", "DashDot"]

TILT_MAPPING = {"wall": 90, "ceiling": 0, "floor": 180}


class Tilt(Enum):
    wall = "wall"
    ceiling = "ceiling"
    floor = "floor"


class Azimuth:
    north = 0
    south = 90
    east = 45
    west = 135


class Flow(str, Enum):
    inlet = "inlet"
    outlet = "outlet"
    radiative = "radiative"
    convective = "convective"
    inlet_or_outlet = "inlet_or_outlet"
    undirected = "undirected"
    interchangeable_port = "interchangeable_port"


class Medium(str, Enum):
    fluid = "fluid"
    heat = "heat"
    data = "data"
    current = "current"
    weather_data = "weather_data"


Boolean = Literal["true", "false"]


class Line(BaseModel):
    template: str
    key: Optional[str] = None
    color: str = "grey"
    label: str
    line_style: str = "solid"
    line_width: float = 1.5


class Axis(BaseModel):
    lines: List[Line] = Field(default=[])
    label: str


class ConnectionView(BaseModel):
    color: Optional[str] = "{255,204,51}"
    thickness: float = 0.1
    disabled: bool = False
    pattern: Pattern = "Solid"


class BaseVariant:
    default: str = "default"
