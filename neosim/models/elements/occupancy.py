from typing import Callable, List

from pydantic import Field

from neosim.models.elements.base import AvailableLibraries, LibraryData, Port
from neosim.models.elements.space import Space
from neosim.models.elements.system import BaseOccupancy


class OccupancyComponent(LibraryData):
    template: str = """
    {{package_name}}.Common.Occupancy.SimpleOccupancy {{ element.name }}"""
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(targets=[Space], names=["y"]),
        ]
    )


class Occupancy(BaseOccupancy):
    libraries_data: AvailableLibraries = AvailableLibraries(
        ideas=[OccupancyComponent],
        buildings=[OccupancyComponent],
    )
