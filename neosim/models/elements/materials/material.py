from typing import List, Optional, Union

from pydantic import Field

from neosim.construction import Construction
from neosim.glass import Glass
from neosim.material import Material
from neosim.models.elements.materials.base import BaseData


class BuildingsMaterial(BaseData):
    template: Optional[str] = None
    constructions: List[Union[Construction, Material, Glass]] = Field(default=[])


class IdeasMaterial(BaseData):
    template: str = Field(
        default="""
    record {{ construction.name }} = IDEAS.Buildings.Data.Interfaces.Material (
 k={{construction.thermal_conductivity}},
      c={{construction.specific_heat_capacity}},
      rho={{construction.density}},
      epsLw=0.88,
      epsSw=0.55);"""
    )