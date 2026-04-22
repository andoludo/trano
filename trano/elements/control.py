from pydantic import computed_field

from trano.elements.common_base import BaseParameter
from trano.elements.base import BaseElement, Control
from trano.elements.space import Space
from trano.elements.system import VAV, Valve, Pump


class ControlParameters(BaseParameter): ...


class VAVControl(Control):
    ahu: BaseElement | None = None


class ThreeWayValveControl(Control): ...


class EmissionControl(Control): ...


class CollectorControl(Control):
    valves: list[Valve] | None = None


class BoilerControl(Control):
    pumps: list[Pump] | None = None


class AhuControl(Control):
    spaces: list[Space] | None = None
    vavs: list[VAV] | None = None

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
