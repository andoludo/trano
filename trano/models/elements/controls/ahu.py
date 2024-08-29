from typing import List, Optional

from pydantic import computed_field

from trano.models.elements.controls.base import Control
from trano.models.elements.damper import VAV
from trano.models.elements.space import Space


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
