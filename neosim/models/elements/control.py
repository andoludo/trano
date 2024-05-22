from typing import TYPE_CHECKING, List, Optional

from pydantic import Field

from neosim.controller.parser import BaseInput
from neosim.models.elements.base import BaseElement

if TYPE_CHECKING:
    pass


class Control(BaseElement):
    name: str
    position: Optional[List[float]] = None
    controllable_element: Optional[BaseElement] = None
    space_name: Optional[str] = None


class SpaceControl(Control):
    ahu: BaseElement = None


class SpaceVentilationControl(Control):
    ...


class SpaceSubstanceVentilationControl(SpaceVentilationControl):
    ...


class AhuControl(Control):
    spaces: Optional[List[str]] = None
    vavs: Optional[List[str]] = None


class EmissionControl(Control):
    ...

class DataBus(BaseElement):
    name: str
    position: Optional[List[float]] = None
    spaces: List[str]
    non_connected_ports: List[BaseInput] = Field(default=[])
