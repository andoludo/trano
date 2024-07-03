from pathlib import Path
from typing import TYPE_CHECKING, Callable, List, Optional

from pydantic import Field

from trano.controller.parser import ControllerBus
from trano.models.elements.base import (
    AvailableLibraries,
    BaseElement,
    DynamicComponentTemplate,
    LibraryData,
    Port,
)
from trano.models.elements.bus import DataBus
from trano.models.elements.controls.base import Control
from trano.models.elements.system import System

if TYPE_CHECKING:
    pass

dynamic_vav_control_template = DynamicComponentTemplate(
    template="""model VAVControl{{ element.name | capitalize}}
Buildings.Controls.OBC.ASHRAE.G36.TerminalUnits.Reheat.Controller rehBoxCon(
venStd=Buildings.Controls.OBC.ASHRAE.G36.Types.VentilationStandard.ASHRAE62_1,
have_winSen=true,
have_occSen=true,
have_CO2Sen=true,
have_hotWatCoi=true,
VOccMin_flow=0.003,
VAreMin_flow=0.003,
VAreBreZon_flow=0.003,
VPopBreZon_flow=0.003,
VMin_flow=0.003,
VCooMax_flow=0.003,
VHeaMin_flow=0.003,
VHeaMax_flow=0.003)
{% raw %}annotation (Placement(transformation(extent={{-36,-36},{28,38}}))); {% endraw %}
{{bus_template}}
equation
{{bus_ports | safe}}
end VAVControl{{ element.name  | capitalize}};""",
    category="control",
    bus=ControllerBus.from_configuration(
        Path(__file__).parent.joinpath("config", "vav.json")
    ),
)


class BaseVavControl(LibraryData):
    template: str = """
    {{package_name}}.Common.Controls.ventilation.VAVControl{{ element.name | capitalize}}
    {{ element.name }}"""
    component_template: DynamicComponentTemplate = dynamic_vav_control_template
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(
                targets=[System, DataBus],
                names=["dataBus"],
                multi_connection=True,
                use_counter=False,
            ),
        ]
    )


class VAVControl(Control):
    libraries_data: AvailableLibraries = AvailableLibraries(
        ideas=[BaseVavControl],
        buildings=[BaseVavControl],
    )
    ahu: Optional[BaseElement] = None
