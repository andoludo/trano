from typing import List, Optional

from trano.elements import Control
from trano.elements.base import BaseElement
from trano.elements.types import BaseVariant


class System(BaseElement):
    position: Optional[List[float]] = None
    control: Optional[Control] = None


class Sensor(System): ...


class EmissionVariant(BaseVariant):
    radiator: str = "radiator"
    ideal: str = "ideal"


class SpaceSystem(System):
    linked_space: Optional[str] = None


class Emission(SpaceSystem): ...


class Ventilation(SpaceSystem): ...


class BaseWeather(System): ...


class BaseOccupancy(System):
    space_name: Optional[str] = None


class Weather(BaseWeather): ...


class Valve(SpaceSystem): ...


class ThreeWayValve(System): ...


class TemperatureSensor(Sensor): ...


class SplitValve(System): ...


class Radiator(Emission): ...


class Pump(System): ...


class Occupancy(BaseOccupancy): ...


class Duct(Ventilation): ...


class DamperVariant(BaseVariant):
    complex: str = "complex"


class Damper(Ventilation): ...


class VAV(Damper):
    variant: str = DamperVariant.default


class Boiler(System): ...


class AirHandlingUnit(Ventilation): ...
