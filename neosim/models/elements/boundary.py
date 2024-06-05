from typing import Callable, List, Optional

from pydantic import Field

from neosim.models.elements.base import (
    AvailableLibraries,
    BaseElement,
    LibraryData,
    Port,
)
from neosim.models.elements.system import AirHandlingUnit
from neosim.models.elements.weather import Weather


class BaseBoundary(LibraryData):
    template: str = """  Buildings.Fluid.Sources.Outside {{ element.name }}
    (nPorts=2,redeclare package Medium = Medium)"""
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(targets=[Weather], names=["weaBus"]),
            Port(
                targets=[AirHandlingUnit],
                names=["ports"],
                multi_connection=True,
                use_counter=False,
            ),
        ]
    )


class Boundary(BaseElement):
    name: str
    position: Optional[List[float]] = None
    libraries_data: AvailableLibraries = AvailableLibraries(
        ideas=[BaseBoundary],
        buildings=[BaseBoundary],
    )
