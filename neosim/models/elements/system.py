from typing import List, Optional

from neosim.models.elements.base import BaseElement, BaseVariant
from neosim.models.elements.controls.base import Control


class System(BaseElement):
    name: str
    position: Optional[List[float]] = None
    control: Optional["Control"] = None


class Sensor(System):
    ...


class EmissionVariant(BaseVariant):
    radiator: str = "radiator"
    ideal: str = "ideal"


class DamperVariant(BaseVariant):
    complex: str = "complex"


class SpaceSystem(System):
    linked_space: Optional[str] = None


class Emission(SpaceSystem):
    ...


class Ventilation(SpaceSystem):
    ...


class AirHandlingUnit(Ventilation):
    ...


class Damper(Ventilation):
    ...


class VAV(Damper):
    variant: str = DamperVariant.default
