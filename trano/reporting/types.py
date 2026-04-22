import abc
from abc import abstractmethod
from pathlib import Path
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_serializer

from trano.reporting.utils import to_html_construction, to_html_space, to_html_system

Topic = Literal["Spaces", "Construction", "Systems", "Base"]


class BaseReporting(BaseModel):
    model_config = ConfigDict(populate_by_name=True, extra="forbid")


class BaseTable(abc.ABC, BaseReporting):
    @abstractmethod
    def to_html(self) -> str: ...


class BaseNestedTable(BaseReporting):
    name: str | None = None
    parameters: dict[str, Any] = Field(default_factory=dict)

    @model_serializer
    def serializer(self) -> dict[str, Any]:
        if not self.parameters:
            return {}
        return self.parameters | {"name": self.name}


class ContentDocumentation(BaseModel):
    title: str | None = None
    introduction: str | None = None
    conclusions: str | None = None


class BoundaryTable(BaseReporting):
    name: str
    surface: float
    tilt: str
    construction: str
    azimuth: float | None = None

    @field_validator("construction", mode="before")
    @classmethod
    def _construction_validator(cls, value: dict[str, Any]) -> str | None:
        return value.get("name")


class EmissionTable(BaseNestedTable): ...


class OccupancyTable(BaseNestedTable): ...


class SystemTable(BaseTable, BaseNestedTable):
    def to_html(self) -> str:
        return to_html_system(self.model_dump(exclude_none=True))


class SpaceTable(BaseTable):
    name: str
    parameters: dict[str, Any]
    external_boundaries: list[BoundaryTable]
    internal_elements: list[BoundaryTable] = Field(default_factory=list)
    emissions: list[EmissionTable] = Field(default_factory=list)
    occupancy: OccupancyTable | None = Field(default_factory=OccupancyTable)

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
    solar_transmittance: list[float] = Field(default_factory=list)
    solar_reflectance_outside_facing: list[float] = Field(default_factory=list)
    solar_reflectance_room_facing: list[float] = Field(default_factory=list)
    infrared_transmissivity: float | None = None
    infrared_absorptivity_outside_facing: float | None = None
    infrared_absorptivity_room_facing: float | None = None


class ConstructionTable(BaseTable):
    name: str
    layers: list[LayerTable]
    u_value_frame: float | None = None

    @field_validator("layers", mode="before")
    @classmethod
    def layers_validator(cls, layers: list[dict[str, Any]]) -> list[LayerTable]:
        return [LayerTable(**(layer["material"] | {"thickness": layer["thickness"]})) for layer in layers]

    def to_html(self) -> str:
        return to_html_construction(self.model_dump())


class ResultFile(BaseModel):
    path: Path
    type: Literal["openmodelica", "dymola"] = "openmodelica"
