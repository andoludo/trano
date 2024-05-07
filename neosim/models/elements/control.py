from typing import List, Optional

from neosim.models.elements.base import BaseElement


class Control(BaseElement):
    name: str
    position: Optional[List[float]] = None


class SpaceControl(Control):
    ...


class SpaceVentilationControl(Control):
    ...


class SpaceSubstanceVentilationControl(SpaceVentilationControl):
    ...


class AhuControl(Control):
    ...


class DataBus(BaseElement):
    name: str
    position: Optional[List[float]] = None
    number_of_spaces: int
    space_names: List[str]
