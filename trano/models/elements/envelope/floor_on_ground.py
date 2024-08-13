from typing import Callable, List

from pydantic import Field

from trano.models.constants import Azimuth, Tilt
from trano.models.elements.base import AvailableLibraries, LibraryData, Port
from trano.models.elements.envelope.base import BaseFloorOnGround
from trano.models.elements.space import Space


class IdeasFloorOnGround(LibraryData):
    template: str = """
    IDEAS.Buildings.Components.SlabOnGround {{ element.name }}(
    redeclare parameter {{ package_name }}.Data.Constructions.
    {{ element.construction.name }} constructionType,
    redeclare package Medium = Medium,
    A={{  element.surface}})"""
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(targets=[Space], names=["propsBus_a"], multi_connection=False),
        ]
    )


class FloorOnGround(BaseFloorOnGround):
    azimuth: float | int = Azimuth.south
    tilt: Tilt = Tilt.floor
    # libraries_data: AvailableLibraries = AvailableLibraries(ideas=[IdeasFloorOnGround])
