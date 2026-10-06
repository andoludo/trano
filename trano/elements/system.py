from typing import TYPE_CHECKING, Union

from trano.elements import Control
from trano.elements.base import BaseElement
from trano.elements.types import BaseVariant, ContainerTypes
from pydantic import model_validator, BaseModel, Field

from trano.exceptions import InvalidSensorInletError, WrongSystemFlowError
import networkx as nx

if TYPE_CHECKING:
    from trano.topology import Network
    from trano.elements import Space


class System(BaseElement):
    control: Control | None = None

    @model_validator(mode="after")
    def _validator(self) -> "System":
        if self.control:
            self.control.container_type = self.container_type
        return self

    def system_ports_connected(self) -> bool:
        return all(port.connected for port in self.ports if not port.no_check)


class Sensor(System): ...


class EmissionVariant(BaseVariant):
    radiator: str = "radiator"
    ideal: str = "ideal"
    ideal_bus: str = "idealbus"


class SpaceSystem(System):
    linked_space: str | None = None


class SpaceHeatingSystem(SpaceSystem):
    container_type: ContainerTypes = "emission"


class Emission(SpaceHeatingSystem): ...


class Ventilation(SpaceSystem):
    container_type: ContainerTypes = "ventilation"


class BaseWeather(System): ...


class BaseOccupancy(System):
    space_name: str | None = None
    include_in_layout: bool = False
    component_size: float = 3


class DistributionSystem(System):
    container_type: ContainerTypes = "distribution"


class Weather(BaseWeather): ...


class Valve(SpaceHeatingSystem): ...


class ThreeWayValve(DistributionSystem): ...


class TemperatureSensor(Sensor): ...


class HeatMeterSensor(Sensor): ...


class SplitValve(DistributionSystem): ...


class Radiator(Emission): ...


class PowerSensor(Sensor):
    """Sums, through the data bus, the heating power of the ideal radiators wired to it."""

    radiators: list[Radiator] = Field(default=[])

    def validate_inlet(self, inlet: BaseElement) -> None:
        """Reject any inlet whose heating power is not published on the data bus."""
        if not isinstance(inlet, Radiator) or inlet.variant != EmissionVariant.ideal_bus:
            raise InvalidSensorInletError(
                f"Inlet {inlet.name} of type {type(inlet).__name__} with variant "
                f"{inlet.variant} cannot be measured by power sensor {self.name}. "
                f"Only radiators with variant {EmissionVariant.ideal_bus} publish "
                f"their heating power on the data bus."
            )

    def configure(self, network: "Network") -> None:
        inlets = sorted(network.graph.predecessors(self), key=lambda node: node.name)  # type: ignore
        for inlet in inlets:
            self.validate_inlet(inlet)
        self.radiators = inlets


class HydronicSystemControl(BaseModel):
    def configure(self, network: "Network") -> None:
        from trano.elements import CollectorControl

        if hasattr(self, "control") and isinstance(self.control, CollectorControl):
            self.control.valves = self._get_linked_valves(network)

    def _get_linked_valves(self, network: "Network") -> list[Valve]:
        valves_: list[Valve] = []
        valves = [node for node in network.graph.nodes if isinstance(node, Valve)]
        for valve in valves:
            path = list(nx.shortest_path(network.graph, self, valve))
            p = path[1:-1]
            if p and all(isinstance(p_, System) for p_ in p) and not any(isinstance(p_, Valve) for p_ in p):
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

    def _get_linked_pumps(self, network: "Network") -> list[Pump]:
        pumps_: list[Pump] = []
        pumps = [node for node in network.graph.nodes if isinstance(node, Pump)]
        for pump in pumps:
            path = list(nx.shortest_path(network.graph, self, pump))
            p = path[1:-1]
            if (
                p and all(isinstance(p_, System) for p_ in p) and not any(isinstance(p_, Pump) for p_ in p)
            ) or not bool(p):
                pumps_.append(pump)
        return pumps_


class Chiller(ProductionSystem):
    """Cooling production (rendered by the ``mpc`` library only)."""


class DhwTank(ProductionSystem):
    """Domestic hot water tank fed by the production system upstream (``mpc`` library only)."""


class Battery(System):
    """Stationary battery (``mpc`` library only; the electrical container)."""

    container_type: ContainerTypes = "solar"


class EvCharger(System):
    """Electric vehicle charger (``mpc`` library only; the electrical container)."""

    container_type: ContainerTypes = "solar"


class AirHandlingUnit(Ventilation):
    def configure(self, network: "Network") -> None:
        from trano.elements import AhuControl

        if self.control and isinstance(self.control, AhuControl):
            self.control.spaces = self._get_ahu_space_elements(network)
            self.control.vavs = self._get_ahu_vav_elements(network)

    def _get_ahu_space_elements(self, network: "Network") -> list["Space"]:
        from trano.elements import Space

        return [x for x in self._get_ahu_elements(Space, network) if isinstance(x, Space)]

    def _get_ahu_vav_elements(self, network: "Network") -> list[VAV]:
        return [x for x in self._get_ahu_elements(VAV, network) if isinstance(x, VAV)]

    def _get_ahu_elements(
        self, element_type: type[Union[VAV, "Space"]], network: "Network"
    ) -> list[Union[VAV, "Space"]]:
        elements_: list[VAV | Space] = []
        elements = [node for node in network.graph.nodes if isinstance(node, element_type)]
        for element in elements:
            try:
                paths = nx.shortest_path(network.graph, self, element)
            except Exception as e:
                raise WrongSystemFlowError("Wrong AHU system configuration flow.") from e
            p = paths[1:-1]
            if p and all(isinstance(p_, Ventilation) for p_ in p):
                elements_.append(element)
        return elements_
