import logging
from pathlib import Path

from jinja2 import Environment, FileSystemLoader
from pydantic import BaseModel, Field, computed_field, model_validator

from trano.elements.connection.port import (
    BasePartialConnectionWithContainerType,
    PartialConnection,
)
from trano.elements.types import ConnectionView
from trano.exceptions import IncompatiblePortsError

INCOMPATIBLE_PORTS = [sorted(["dataBus", "y"])]

logger = logging.getLogger(__name__)


class Connection(BaseModel):
    right: PartialConnection
    left: PartialConnection
    connection_view: ConnectionView = Field(default=ConnectionView())

    def in_the_same_container(self) -> bool:
        return bool(self.left.name is not None) and bool(self.right.name is not None)

    def reset_counters(self) -> "Connection":
        self.left.reset_port_counter()
        self.right.reset_port_counter()
        return self

    def equation_view(self) -> tuple[str, ...]:
        return tuple(sorted([self.left.equation, self.right.equation]))

    @computed_field
    def equation(self) -> str:
        environment = Environment(
            trim_blocks=True,
            lstrip_blocks=True,
            loader=FileSystemLoader(str(Path(__file__).parents[2].joinpath("templates"))),
            autoescape=True,
        )
        annotation_template = environment.from_string(
            """{% import 'macros.jinja2' as macros %}
        connect({{ connection.left.equation }},{{ connection.right.equation }})
            {% if not connection.connection_view.disabled %}
        annotation (Line(
        points={{ macros.connect_path(connection.path) }},
            {% if connection.connection_view.color %}
        color={{ connection.connection_view.color }},
            {% endif %}
        thickness={{ connection.connection_view.thickness }},pattern =
        LinePattern.{{ connection.connection_view.pattern }},
        smooth=Smooth.None))
            {% endif %}
            ;"""
        )

        return annotation_template.render(
            connection=self,
        )

    @model_validator(mode="after")
    def _connection_validator(self) -> "Connection":
        if self.right.position.container.is_empty() or self.left.position.container.is_empty():
            logger.debug(f"Connection position still empty for {self.right.name} and {self.left.name}.")
        if sorted([part.split(".")[-1] for part in [self.right.equation, self.left.equation]]) in INCOMPATIBLE_PORTS:
            raise IncompatiblePortsError(f"Incompatible ports {self.right.equation} and {self.left.equation}.")
        return self

    @property
    def path(self) -> list[list[float] | tuple[float, float]]:
        if self.left.position.global_.location.c_.x < self.right.position.global_.location.c_.x:
            mid_path = (self.right.position.global_.location.c_.x - self.left.position.global_.location.c_.x) / 2
            return [
                self.left.position.global_.coordinate(),
                (
                    self.left.position.global_.location.c_.x + mid_path,
                    self.left.position.global_.location.c_.y,
                ),
                (
                    self.right.position.global_.location.c_.x - mid_path,
                    self.right.position.global_.location.c_.y,
                ),
                self.right.position.global_.coordinate(),
            ]

        else:
            mid_path = (self.left.position.global_.location.c_.x - self.right.position.global_.location.c_.x) / 2
            return [
                self.left.position.global_.coordinate(),
                (
                    self.left.position.global_.location.c_.x - mid_path,
                    self.left.position.global_.location.c_.y,
                ),
                (
                    self.right.position.global_.location.c_.x + mid_path,
                    self.right.position.global_.location.c_.y,
                ),
                self.right.position.global_.coordinate(),
            ]


class ContainerConnection(Connection):
    source: tuple[str, str]

    def get_container_equation(
        self,
    ) -> BasePartialConnectionWithContainerType | None:
        if self.right.name is None:
            return self.right
        if self.left.name is None:
            return self.left
        return None

    @property
    def path(self) -> list[list[float] | tuple[float, float]]:
        if self.left.position.container.location.c_.x < self.right.position.container.location.c_.x:
            mid_path = (self.right.position.container.location.c_.x - self.left.position.container.location.c_.x) / 2
            return [
                self.left.position.container.coordinate(),
                (
                    self.left.position.container.location.c_.x + mid_path,
                    self.left.position.container.location.c_.y,
                ),
                (
                    self.right.position.container.location.c_.x - mid_path,
                    self.right.position.container.location.c_.y,
                ),
                self.right.position.container.coordinate(),
            ]

        else:
            mid_path = (self.left.position.container.location.c_.x - self.right.position.container.location.c_.x) / 2
            return [
                self.left.position.container.coordinate(),
                (
                    self.left.position.container.location.c_.x - mid_path,
                    self.left.position.container.location.c_.y,
                ),
                (
                    self.right.position.container.location.c_.x + mid_path,
                    self.right.position.container.location.c_.y,
                ),
                self.right.position.container.coordinate(),
            ]
