from typing import List, Optional

from neosim.models.elements.base import BaseElement, BaseVariant
from neosim.models.elements.control import Control


class System(BaseElement):
    name: str
    position: Optional[List[float]] = None
    control: Optional["Control"] = None


class EmissionVariant(BaseVariant):
    radiator: str = "radiator"
    ideal: str = "ideal"


class Emission(System):
    variant: str = EmissionVariant.radiator


class Valve(System):
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


class Ventilation(System):
    ...


class AirHandlingUnit(Ventilation):
    ...


class Duct(Ventilation):
    ...


class Damper(Ventilation):
    ...


class VAV(Damper):
    ...
