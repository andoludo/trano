from typing import List, Optional

from neosim.models.elements.base import BaseElement


class Boundary(BaseElement):
    name: str
    position: Optional[List[float]] = None
