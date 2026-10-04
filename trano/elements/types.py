import math
from enum import Enum

from typing import Literal

from pydantic import BaseModel, Field

DynamicTemplateCategories = Literal["ventilation", "control", "fluid", "boiler"]
SystemContainerTypes = Literal["envelope", "distribution", "emission", "production", "ventilation"]
ContainerTypes = Literal[SystemContainerTypes, "bus", "solar"]
Pattern = Literal["Solid", "Dot", "Dash", "DashDot"]

TILT_MAPPING = {
    "wall": 90,
    "ceiling": 0,
    "floor": 180,
    "pitched_roof_45": 45,
    "pitched_roof_30": 30,
    "pitched_roof_35": 35,
    "pitched_roof_40": 40,
    "pitched_roof_20": 20,
}

DEFAULT_TILT = ["wall", "ceiling", "floor"]

# Inclination rendered in the Modelica models [rad]; the pitched roofs are rounded the same way
# as in macros.jinja2 (convert_tilt) so that both agree.
TILT_RADIANS = {
    "wall": math.pi / 2,
    "ceiling": 0.0,
    "floor": math.pi,
    "pitched_roof_45": 0.785,
    "pitched_roof_40": 0.698,
    "pitched_roof_35": 0.611,
    "pitched_roof_30": 0.524,
    "pitched_roof_20": 0.349,
}


def wind_pressure_table(tilt: "Tilt") -> str:
    """Wind pressure coefficient table IDEAS selects for a surface of this inclination.

    Mirrors the selection in IDEAS.Buildings.Components.OuterWall (parameter coeffsCp).
    """
    inclination = TILT_RADIANS[tilt.value]
    if inclination <= math.pi / 18:
        return "Cp_Roof_0_10"
    if inclination <= math.pi / 6:
        return "Cp_Roof_11_30"
    if inclination <= math.pi / 4:
        return "Cp_Roof_30_45"
    if abs(inclination - math.pi) < 0.01:
        return "Cp_Floor"
    return "Cp_Wall"


class Tilt(str, Enum):
    wall = "wall"
    ceiling = "ceiling"
    floor = "floor"
    pitched_roof_45 = "pitched_roof_45"
    pitched_roof_40 = "pitched_roof_40"
    pitched_roof_35 = "pitched_roof_35"
    pitched_roof_30 = "pitched_roof_30"
    pitched_roof_20 = "pitched_roof_20"


class Azimuth:
    north = 3.14
    south = 0
    east = -1.57
    west = 1.57


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
    key: str | None = None
    color: str = "grey"
    label: str
    line_style: str = "solid"
    line_width: float = 1.5


class Axis(BaseModel):
    lines: list[Line] = Field(default=[])
    label: str


class ConnectionView(BaseModel):
    color: str | None = "{255,204,51}"
    thickness: float = 0.1
    disabled: bool = False
    pattern: Pattern = "Solid"


class BaseVariant:
    default: str = "default"
