from typing import List, Optional

from trano.elements import Control
from trano.elements.base import BaseElement
from trano.elements.types import BaseVariant, ContainerTypes
from pydantic import model_validator

class System(BaseElement):
    position: Optional[List[float]] = None
    control: Optional[Control] = None

    @model_validator(mode="after")
    def _validator(self):
        if self.control:
            self.control.container_type = self.container_type
        return self


class Sensor(System): ...


class EmissionVariant(BaseVariant):
    radiator: str = "radiator"
    ideal: str = "ideal"


class SpaceSystem(System):
    linked_space: Optional[str] = None

class SpaceHeatingSystem(SpaceSystem):
    container_type: ContainerTypes = "emission"

class Emission(SpaceHeatingSystem): ...


class Ventilation(SpaceSystem): ...


class BaseWeather(System): ...


class BaseOccupancy(System):
    space_name: Optional[str] = None

class DistributionSystem(System):
    container_type: ContainerTypes = "distribution"

class Weather(BaseWeather): ...


class Valve(SpaceHeatingSystem): ...


class ThreeWayValve(DistributionSystem): ...


class TemperatureSensor(Sensor): ...


class SplitValve(DistributionSystem): ...


class Radiator(Emission): ...


class Pump(DistributionSystem): ...


class Occupancy(BaseOccupancy): ...


class Duct(Ventilation): ...


class DamperVariant(BaseVariant):
    complex: str = "complex"


class Damper(Ventilation): ...


class VAV(Damper):
    variant: str = DamperVariant.default

class ProductionSystem(System):
    container_type: ContainerTypes = "production"

class Boiler(ProductionSystem): ...


class AirHandlingUnit(Ventilation): ...
