from typing import Callable, List

from pydantic import Field

from trano.models.constants import Flow
from trano.models.elements.base import LibraryData, Port
from trano.models.elements.system import Ventilation


class BaseDuct(LibraryData):
    template: str = """  {{ library_name }}.Fluid.FixedResistances.PressureDrop
    {{ element.name }}(
    m_flow_nominal=100*1.2/3600,
    redeclare package Medium = Medium,
    allowFlowReversal = false,
    dp_nominal=40) "Pressure drop for return duct" """
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(names=["port_a"], flow=Flow.inlet),
            Port(names=["port_b"], flow=Flow.outlet),
        ]
    )


class Duct(Ventilation):
    ...
