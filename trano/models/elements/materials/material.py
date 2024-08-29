from typing import List, Optional, Union

from pydantic import Field

from trano.models.elements.construction import Construction, Glass, Material
from trano.models.elements.materials.base import BaseData


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
