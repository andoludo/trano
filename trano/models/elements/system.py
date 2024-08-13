
from typing import List, Optional
from pydantic import Field,field_validator
from trano.models.elements.base import BaseElement, BaseVariant, AvailableLibraries
from trano.models.elements.controls.base import Control


class System(BaseElement):
    position: Optional[List[float]] = None
    control: Optional["Control"] = None



class Sensor(System):
    ...


class EmissionVariant(BaseVariant):
    radiator: str = "radiator"
    ideal: str = "ideal"


class SpaceSystem(System):
    linked_space: Optional[str] = None


class Emission(SpaceSystem):
    ...


class Ventilation(SpaceSystem):
    ...


class BaseWeather(System):
    ...


class BaseOccupancy(System):
    space_name: Optional[str] = None
