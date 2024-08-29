from typing import Optional

from trano.models.elements.base import (
    BaseElement,
)
from trano.models.elements.controls.base import Control


class VAVControl(Control):
    ahu: Optional[BaseElement] = None
