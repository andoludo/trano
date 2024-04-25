from typing import List, Optional

from neosim.models.elements.base import BaseElement


class Control(BaseElement):
    name: str
    position: Optional[List[float]] = None


class SpaceControl(Control):
    ...
