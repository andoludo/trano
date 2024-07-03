from typing import Callable, List

from pydantic import Field

from neosim.controller.parser import ControllerBus, IntegerOutput
from neosim.models.elements.base import (
    AvailableLibraries,
    BaseParameter,
    DynamicComponentTemplate,
    LibraryData,
    Port,
)
from neosim.models.elements.bus import DataBus
from neosim.models.elements.space import Space
from neosim.models.elements.system import BaseOccupancy

dynamic_occupancy_template = DynamicComponentTemplate(
    template="""
model Occupancy{{ element.name | capitalize}}
extends {{ package_name }}.Common.Occupancy.SimpleOccupancy ;
{{bus_template}}
equation
{{bus_ports | safe}}
 end Occupancy{{ element.name | capitalize}};
 """,
    category="control",
    bus=ControllerBus(
        integer_outputs=[
            IntegerOutput(
                name="Occupied",
                target="element.space_name",
                component="occSch2",
                port="occupied",
            )
        ]
    ),
)


class OccupancyParameters(BaseParameter):
    occupancy: str = Field(
        "3600*{7, 19}",
        alias="occupancy",
        title="Occupancy table, each entry switching occupancy on or off",
    )
    gain: str = Field(
        "[35; 70; 30]",
        alias="gain",
        title="Gain to convert from occupancy (per person) to radiant, "
        "convective and latent heat in [W/m2] ",
    )
    heat_gain_if_occupied: str = Field(
        "1/6/4", alias="k", title="Heat gain if occupied"
    )


class OccupancyComponent(LibraryData):
    template: str = """
    {{package_name}}.Common.Controls.ventilation.Occupancy{{ element.name | capitalize}}
    {{ element.name }}({{ macros.render_parameters(parameters) | safe}})"""
    component_template: DynamicComponentTemplate = dynamic_occupancy_template
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(targets=[Space], names=["y"]),
            Port(
                targets=[DataBus],
                names=["dataBus"],
                multi_connection=True,
                use_counter=False,
            ),
        ]
    )


class Occupancy(BaseOccupancy):
    parameters: OccupancyParameters = Field(default=OccupancyParameters())
    libraries_data: AvailableLibraries = AvailableLibraries(
        ideas=[OccupancyComponent],
        buildings=[OccupancyComponent],
    )
