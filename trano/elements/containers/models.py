from typing import cast

from pydantic import BaseModel, Field, computed_field, model_validator

from trano.elements import BaseElement, Connection, Port
from trano.elements.common_base import BaseProperties, MediumTemplate, Point
from trano.elements.connection import (
    BasePartialConnectionWithContainerType,
    PartialConnection,
)
from trano.elements.types import ContainerTypes, Medium


class PortGroup(BaseModel):
    connected_container_names: list[ContainerTypes]
    ports: list[Port]


class PortGroupMedium(BaseModel):
    medium: Medium
    port: Port


class BaseContainer(BaseModel):
    component_size: Point = Field(default=Point(x=20, y=20))


class ContainerInput(BaseModel):
    nodes: list[BaseElement]
    connections: list[Connection]
    data: BaseProperties
    medium: MediumTemplate


class ContainerLayout(BaseModel):
    bottom_left: Point = Point(x=-100, y=-100)
    top_right: Point = Point(x=100, y=100)
    global_origin: Point

    @computed_field  # type: ignore[prop-decorator]
    @property
    def scale(self) -> int:
        scale_x = int(self.top_right.c_.x - self.bottom_left.c_.x)
        scale_y = int(self.top_right.c_.y - self.bottom_left.c_.y)
        if scale_x != scale_y:
            raise ValueError("Scale x and y must be equal")
        return scale_x


class MainContainerConnection(BaseModel):
    left: PartialConnection
    right: PartialConnection
    annotation: str | None = None

    @classmethod
    def from_list(cls, connections: list[BasePartialConnectionWithContainerType]) -> "MainContainerConnection":
        return cls(left=connections[0], right=connections[1])

    def get_equation(self) -> str:
        return (
            f"connect({self.left.container_type}1.{self.left.equation}, "
            f"{self.right.container_type}1.{self.right.equation})"
        )


class Location(BaseModel):
    point_1: Point
    point_2: Point


class ConnectionList(BaseModel):
    connection_type: list[str]
    annotation: str


class BusConnection(BaseModel):
    connection_type: tuple[str, str]
    location: str

    @model_validator(mode="after")
    def _validator(self) -> "BusConnection":
        self.connection_type = cast(tuple[str, str], tuple(sorted(self.connection_type)))

        return self

    def equation(self) -> str:
        return (
            f"connect({self.connection_type[0]}1.dataBus, {self.connection_type[1]}1.dataBus) "
            f"annotation (Line(points={self.location}, color={{255,204,51}}, thickness=0.5));"
        )
