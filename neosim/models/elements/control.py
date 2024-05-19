from typing import TYPE_CHECKING, List, Optional

from pydantic import Field

from neosim.models.elements.base import BaseElement

if TYPE_CHECKING:
    from neosim.models.elements.space import Space


class Control(BaseElement):
    name: str
    position: Optional[List[float]] = None
    controllable_element: Optional[BaseElement] = None


class SpaceControl(Control):
    space: Optional["Space"] = None
    neighbors: List[BaseElement] = Field(default=[])


class SpaceVentilationControl(Control):
    ...


class SpaceSubstanceVentilationControl(SpaceVentilationControl):
    ...


class AhuControl(Control):
    ...


class DataBus(BaseElement):
    name: str
    position: Optional[List[float]] = None
    spaces: List[str]
