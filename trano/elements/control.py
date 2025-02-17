from typing import List, Optional

from pydantic import computed_field

from trano.elements.common_base import BaseParameter
from trano.elements.base import BaseElement, Control
from trano.elements.space import Space
from trano.elements.system import VAV, Valve


class ControlParameters(BaseParameter): ...


class VAVControl(Control):
    ahu: Optional[BaseElement] = None


class ThreeWayValveControl(Control): ...


class EmissionControl(Control): ...


class CollectorControl(Control):
    valves: Optional[List[Valve]] = None


class BoilerControl(Control): ...


class AhuControl(Control):
    spaces: Optional[List[Space]] = None
    vavs: Optional[List[VAV]] = None

    @computed_field
    def zon_gro_mat(self) -> str:
        if self.vavs is None:
            return "[1]"
        return str([1] * len(self.vavs))

    @computed_field
    def zon_gro_mat_tra(self) -> str:
        if self.vavs is None:
            return "[1]"
        return str([1] * len(self.vavs)).replace(",", ";")
