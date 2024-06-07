from typing import Callable, List

from pydantic import Field

from neosim.models.constants import Flow
from neosim.models.elements.base import AvailableLibraries, LibraryData, Port
from neosim.models.elements.controls.base import Control
from neosim.models.elements.system import Sensor


class BaseTemperatureSensor(LibraryData):
    template: str = """    Buildings.Fluid.Sensors.TemperatureTwoPort {{ element.name }}(
    redeclare package Medium = MediumW,
    m_flow_nominal=mRad_flow_nominal) "Radiator" """
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(
                names=["port_a"],
                flow=Flow.inlet,
                multi_connection=True,
                use_counter=False,
            ),
            Port(
                names=["port_b"],
                flow=Flow.outlet,
                multi_connection=True,
                use_counter=False,
            ),
            Port(targets=[Control], names=["T"]),
        ]
    )


class TemperatureSensor(Sensor):
    libraries_data: AvailableLibraries = AvailableLibraries(
        ideas=[BaseTemperatureSensor],
        buildings=[BaseTemperatureSensor],
    )
