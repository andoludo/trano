from typing import List, Optional


from neosim.models.elements.base import BaseElement, BaseVariant
from neosim.models.elements.control import Control


class System(BaseElement):
    name: str
    position: Optional[List[float]] = None
    control: Optional["Control"] = None

class Sensor(System):
    ...
class TemperatureSensor(Sensor):
    ...

class EmissionVariant(BaseVariant):
    radiator: str = "radiator"
    ideal: str = "ideal"


class DamperVariant(BaseVariant):
    complex: str = "complex"


class SpaceSystem(System):
    linked_space: Optional[str] = None


class Emission(SpaceSystem):
    variant: str = EmissionVariant.radiator


class Valve(SpaceSystem):
    ...


class SplitValve(System):
    ...


class ThreeWayValve(System):
    ...


class Pump(System):
    ...


class Boiler(System):
    ...


class Occupancy(System):
    ...


class Weather(System):
    ...


class Ventilation(SpaceSystem):
    ...


class AirHandlingUnit(Ventilation):
    ...


class Duct(Ventilation):
    ...


class Damper(Ventilation):
    ...


class VAV(Damper):
    variant: str = DamperVariant.default
