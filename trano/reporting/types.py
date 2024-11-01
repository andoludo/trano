import abc
from abc import abstractmethod
from pathlib import Path
from typing import Any, Dict, List, Literal, Optional

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_serializer

from trano.reporting.utils import to_html_construction, to_html_space, to_html_system

Topic = Literal["Spaces", "Construction", "Systems", "Base"]


class BaseReporting(BaseModel):
    model_config = ConfigDict(populate_by_name=True, extra="forbid")


class BaseTable(abc.ABC, BaseReporting):
    @abstractmethod
    def to_html(self) -> str: ...


class BaseNestedTable(BaseReporting):
    name: Optional[str] = None
    parameters: Dict[str, Any] = Field(default_factory=dict)

    @model_serializer
    def serializer(self) -> Dict[str, Any]:
        if not self.parameters:
            return {}
        return self.parameters | {"name": self.name}


class ContentDocumentation(BaseModel):
    title: Optional[str] = None
    introduction: Optional[str] = None
    conclusions: Optional[str] = None


class BoundaryTable(BaseReporting):
    name: str
    surface: float
    tilt: str
    construction: str
    azimuth: Optional[float] = None

    @field_validator("construction", mode="before")
    @classmethod
    def _construction_validator(cls, value: Dict[str, Any]) -> Optional[str]:
        return value.get("name")


class EmissionTable(BaseNestedTable): ...


class OccupancyTable(BaseNestedTable): ...


class SystemTable(BaseTable, BaseNestedTable):
    def to_html(self) -> str:
        return to_html_system(self.model_dump(exclude_none=True))


class SpaceTable(BaseTable):
    name: str
    parameters: Dict[str, Any]
    external_boundaries: List[BoundaryTable]
    internal_elements: List[BoundaryTable] = Field(default_factory=list)
    emissions: List[EmissionTable] = Field(default_factory=list)
    occupancy: Optional[OccupancyTable] = Field(default_factory=OccupancyTable)

    def to_html(self) -> str:
        return to_html_space(self.model_dump(exclude_none=True))


class LayerTable(BaseReporting):
    name: str
    k: float
    c: float
    rho: float
    epsLw: float  # noqa: N815
    epsSw: float  # noqa: N815
    thickness: float
    solar_transmittance: List[float] = Field(default_factory=list)
    solar_reflectance_outside_facing: List[float] = Field(default_factory=list)
    solar_reflectance_room_facing: List[float] = Field(default_factory=list)
    infrared_transmissivity: Optional[float] = None
    infrared_absorptivity_outside_facing: Optional[float] = None
    infrared_absorptivity_room_facing: Optional[float] = None


class ConstructionTable(BaseTable):
    name: str
    layers: List[LayerTable]
    u_value_frame: Optional[float] = None

    @field_validator("layers", mode="before")
    @classmethod
    def layers_validator(cls, layers: List[Dict[str, Any]]) -> List[LayerTable]:
        return [
            LayerTable(**(layer["material"] | {"thickness": layer["thickness"]}))
            for layer in layers
        ]

    def to_html(self) -> str:
        return to_html_construction(self.model_dump())


class ResultFile(BaseModel):
    path: Path
    type: Literal["openmodelica", "dymola"] = "openmodelica"
