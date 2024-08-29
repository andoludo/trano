from typing import List, Optional

from trano.models.elements.base import BaseElement, BaseParameter


class Control(BaseElement):
    position: Optional[List[float]] = None
    controllable_element: Optional[BaseElement] = None
    space_name: Optional[str] = None


class ControlParameters(BaseParameter):
    ...

