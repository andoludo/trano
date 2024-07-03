from typing import Callable, List

from pydantic import Field

from trano.models.elements.ahu import AirHandlingUnit
from trano.models.elements.base import (
    AvailableLibraries,
    BaseBoundary,
    LibraryData,
    Port,
)
from trano.models.elements.system import BaseWeather


class BaseBoundaryComponent(LibraryData):
    template: str = """  Buildings.Fluid.Sources.Outside {{ element.name }}
    (nPorts=2,redeclare package Medium = Medium)"""
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(targets=[BaseWeather], names=["weaBus"]),
            Port(
                targets=[AirHandlingUnit],
                names=["ports"],
                multi_connection=True,
                use_counter=False,
            ),
        ]
    )


class Boundary(BaseBoundary):
    libraries_data: AvailableLibraries = AvailableLibraries(
        ideas=[BaseBoundaryComponent],
        buildings=[BaseBoundaryComponent],
    )
