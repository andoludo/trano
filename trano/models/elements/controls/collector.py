from typing import List, Optional

from trano.models.elements.controls.base import Control
from trano.models.elements.valve import Valve


class CollectorControl(Control):
    valves: Optional[List[Valve]] = None
