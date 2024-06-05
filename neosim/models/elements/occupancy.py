from typing import Callable, List

from pydantic import Field

from neosim.models.elements.base import AvailableLibraries, LibraryData, Port
from neosim.models.elements.system import System


def occupancy_factory():
    from neosim.models.elements.space import Space

    class BaseOccupancy(LibraryData):
        template: str = """
        {{package_name}}.Common.Occupancy.SimpleOccupancy {{ element.name }}"""
        ports_factory: Callable[[], List[Port]] = Field(
            default=lambda: [
                Port(targets=[Space], names=["y"]),
            ]
        )

    return BaseOccupancy()


class Occupancy(System):
    libraries_data: AvailableLibraries = AvailableLibraries(
        ideas=[occupancy_factory],
        buildings=[occupancy_factory],
    )
