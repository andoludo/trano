from typing import Callable, List

from pydantic import Field

from neosim.models.constants import Flow
from neosim.models.elements.base import AvailableLibraries, LibraryData, Port
from neosim.models.elements.envelope.base import BaseInternalElement
from neosim.models.elements.space import Space


class BuildingsInternalElement(LibraryData):
    template: str = """    Buildings.HeatTransfer.Conduction.MultiLayer
                {{ element.name }}(A =
            {{ element.surface }}, layers =
    {{ element.construction.name }}, stateAtSurface_a = true, stateAtSurface_b = true)
    "Partition wall between the two
    rooms" """
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(targets=[Space], names=["port_a"], flow=Flow.interchangeable_port),
            Port(targets=[Space], names=["port_b"], flow=Flow.interchangeable_port),
        ]
    )


class IdeasInternalElement(LibraryData):
    template: str = """
    IDEAS.Buildings.Components.InternalWall {{ element.name }}
    (redeclare parameter {{ package_name }}.
    Data.Constructions.{{ element.construction.name }} constructionType,
    redeclare package Medium = Medium,
    A = {{ element.surface }}, inc = IDEAS.Types.Tilt.
    {{ element.tilt.value | capitalize }}, azi =
    {{ element.azimuth }}) "Partition wall between the two
    rooms" """
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(
                targets=[Space],
                names=["propsBus_a"],
                multi_connection=False,
                flow=Flow.interchangeable_port,
            ),
            Port(
                targets=[Space],
                names=["propsBus_b"],
                multi_connection=False,
                flow=Flow.interchangeable_port,
            ),
        ]
    )


class InternalElement(BaseInternalElement):
    libraries_data: AvailableLibraries = AvailableLibraries(
        ideas=[IdeasInternalElement],
        buildings=[BuildingsInternalElement],
    )
