from typing import List, Optional, TYPE_CHECKING, Type, Union

from trano.elements import Control
from trano.elements.base import BaseElement
from trano.elements.types import BaseVariant, ContainerTypes
from pydantic import model_validator, BaseModel

from trano.exceptions import WrongSystemFlowError
import networkx as nx

if TYPE_CHECKING:
    from trano.topology import Network
    from trano.elements import Space


class System(BaseElement):
    control: Optional[Control] = None

    @model_validator(mode="after")
    def _validator(self) -> "System":
        if self.control:
            self.control.container_type = self.container_type
        return self

    def system_ports_connected(self) -> bool:
        return all(port.connected for port in self.ports)


class Sensor(System): ...


class EmissionVariant(BaseVariant):
    radiator: str = "radiator"
    ideal: str = "ideal"


class SpaceSystem(System):
    linked_space: Optional[str] = None


class SpaceHeatingSystem(SpaceSystem):
    container_type: ContainerTypes = "emission"


class Emission(SpaceHeatingSystem): ...


class Ventilation(SpaceSystem):
    container_type: ContainerTypes = "ventilation"


class BaseWeather(System): ...


class BaseOccupancy(System):
    space_name: Optional[str] = None
    include_in_layout: bool = False
    component_size: float = 3


class DistributionSystem(System):
    container_type: ContainerTypes = "distribution"


class Weather(BaseWeather): ...


class Valve(SpaceHeatingSystem): ...


class ThreeWayValve(DistributionSystem): ...


class TemperatureSensor(Sensor): ...


class SplitValve(DistributionSystem): ...


class Radiator(Emission): ...


class HydronicSystemControl(BaseModel):
    def configure(self, network: "Network") -> None:
        from trano.elements import CollectorControl

        if hasattr(self, "control") and isinstance(self.control, CollectorControl):
            self.control.valves = self._get_linked_valves(network)

    def _get_linked_valves(self, network: "Network") -> List[Valve]:
        valves_: List[Valve] = []
        valves = [node for node in network.graph.nodes if isinstance(node, Valve)]
        for valve in valves:
            path = list(nx.shortest_path(network.graph, self, valve))
            p = path[1:-1]
            if (
                p
                and all(isinstance(p_, System) for p_ in p)
                and not any(isinstance(p_, Valve) for p_ in p)
            ):
                valves_.append(valve)
        return valves_


class Pump(HydronicSystemControl, DistributionSystem): ...


class Occupancy(BaseOccupancy): ...


class Duct(Ventilation): ...


class DamperVariant(BaseVariant):
    complex: str = "complex"


class Damper(Ventilation): ...


class VAV(Damper):
    variant: str = DamperVariant.default


class ProductionSystem(System):
    container_type: ContainerTypes = "production"


class Boiler(HydronicSystemControl, ProductionSystem):
    def configure(self, network: "Network") -> None:
        from trano.elements import BoilerControl

        if hasattr(self, "control") and isinstance(self.control, BoilerControl):
            self.control.pumps = self._get_linked_pumps(network)

    def _get_linked_pumps(self, network: "Network") -> List[Pump]:
        pumps_: List[Pump] = []
        pumps = [node for node in network.graph.nodes if isinstance(node, Pump)]
        for pump in pumps:
            path = list(nx.shortest_path(network.graph, self, pump))
            p = path[1:-1]
            if (
                p
                and all(isinstance(p_, System) for p_ in p)
                and not any(isinstance(p_, Pump) for p_ in p)
            ) or not bool(p):
                pumps_.append(pump)
        return pumps_


class AirHandlingUnit(Ventilation):
    def configure(self, network: "Network") -> None:
        from trano.elements import AhuControl

        if self.control and isinstance(self.control, AhuControl):
            self.control.spaces = self._get_ahu_space_elements(network)
            self.control.vavs = self._get_ahu_vav_elements(network)

    def _get_ahu_space_elements(self, network: "Network") -> List["Space"]:
        from trano.elements import Space

        return [
            x for x in self._get_ahu_elements(Space, network) if isinstance(x, Space)
        ]

    def _get_ahu_vav_elements(self, network: "Network") -> List[VAV]:
        return [x for x in self._get_ahu_elements(VAV, network) if isinstance(x, VAV)]

    def _get_ahu_elements(
        self, element_type: Type[Union[VAV, "Space"]], network: "Network"
    ) -> List[Union[VAV, "Space"]]:
        elements_: List[Union[VAV, "Space"]] = []
        elements = [
            node for node in network.graph.nodes if isinstance(node, element_type)
        ]
        for element in elements:
            try:
                paths = nx.shortest_path(network.graph, self, element)
            except Exception as e:
                raise WrongSystemFlowError(
                    "Wrong AHU system configuration flow."
                ) from e
            p = paths[1:-1]
            if p and all(isinstance(p_, Ventilation) for p_ in p):
                elements_.append(element)
        return elements_
