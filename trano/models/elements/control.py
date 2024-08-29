from typing import List, Optional

from pydantic import computed_field

from trano.models.elements.base import BaseElement, BaseParameter, Control
from trano.models.elements.space import Space
from trano.models.elements.system import Valve, VAV


class ControlParameters(BaseParameter):
    ...


class VAVControl(Control):
    ahu: Optional[BaseElement] = None


class ThreeWayValveControl(Control):
    ...


class EmissionControl(Control):
    ...


class CollectorControl(Control):
    valves: Optional[List[Valve]] = None


class BoilerControl(Control):
    ...


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
