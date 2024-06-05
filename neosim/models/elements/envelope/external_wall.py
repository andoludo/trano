from typing import Callable, List

from pydantic import Field

from neosim.models.elements.base import AvailableLibraries, LibraryData, Port
from neosim.models.elements.envelope.base import BaseSimpleWall


def ideas_merged_external_wall_factory():
    from neosim.models.elements.space import Space

    class IdeasMergedExternalWall(LibraryData):
        template: str = """
        {% set tilts = tilts_processing_ideas(element) %}
        IDEAS.Buildings.Components.OuterWall[{{ element.surfaces | length }}]
        {{ element.name }}(
        redeclare parameter {{ package_name }}.Data.Constructions.
        {{ element.constructions[0].name }}
        constructionType,
        A={{  macros.join_list(element.surfaces)}},
        final azi={{macros.join_list(element.azimuths)}},
        redeclare package Medium = Medium,
        final inc={{macros.join_list(tilts)}})"""
        ports_factory: Callable[[], List[Port]] = Field(
            default=lambda: [
                Port(
                    targets=[Space],
                    names=["propsBus_a"],
                    multi_connection=True,
                    multi_object=True,
                    bus_connection=True,
                ),
            ]
        )

    return IdeasMergedExternalWall()


class ExternalWall(BaseSimpleWall):
    libraries_data: AvailableLibraries = AvailableLibraries(
        ideas=[ideas_merged_external_wall_factory]
    )
