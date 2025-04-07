from typing import Optional, Tuple

from pydantic import BaseModel, Field, ConfigDict

from trano.elements.types import ContainerTypes


class BaseProperties(BaseModel):
    container_type: ContainerTypes
    is_package: bool
    data: str


class MandatoryPoint(BaseModel):
    x: float
    y: float


class Point(BaseModel):
    x: Optional[float] = None
    y: Optional[float] = None

    def is_valid(self) -> bool:
        return self.x is not None and self.y is not None

    @property
    def c_(self) -> MandatoryPoint:
        return MandatoryPoint(x=self.x, y=self.y)


class ElementPosition(BaseModel):
    location: Point = Field(default_factory=Point)
    annotation: str = """annotation (
    Placement(transformation(origin = {{ macros.join_list(element.position.global_.coordinate()) }},
    extent = {% raw %}{{ 5, -5}, {-5, 5}}
    {% endraw %})));"""

    def set_container_annotation(self, size: float) -> None:
        self.annotation = """annotation (
    Placement(transformation(origin = {{ macros.join_list(element.position.container.coordinate()) }},
    extent = {% raw %}{{ #size#, -#size#}, {-#size#, #size#}}
    {% endraw %})));""".replace(
            "#size#", str(size)
        )  # TODO: lazy hack..

    def is_empty(self) -> bool:
        return not self.location.is_valid()

    def coordinate(self) -> Tuple[float, float]:
        return self.location.c_.x, self.location.c_.y


class BasePosition(BaseModel):
    container: ElementPosition = Field(
        default_factory=lambda: ElementPosition(
            annotation="""annotation (
        Placement(transformation(origin = {{ macros.join_list(element.position.container.coordinate()) }},
        extent = {% raw %}{{5, -5}, {-5, 5}}
        {% endraw %})));"""
        )
    )
    global_: ElementPosition = Field(default_factory=ElementPosition)

    def set_contaienr_annotation(self, size: float) -> None:
        self.global_.set_container_annotation(size)
        self.container.set_container_annotation(size)

    def set_global(self, x: float, y: float) -> None:
        self.global_.location.x = x
        self.global_.location.y = y

    def set_container(self, x: float, y: float) -> None:
        self.container.location.x = x
        self.container.location.y = y

    @property
    def x_container(self) -> float:
        return self.container.location.c_.x

    @property
    def y_container(self) -> float:
        return self.container.location.c_.y

    @property
    def x_global(self) -> float:
        x = self.global_.location.x
        if x is None:
            raise ValueError("x_global is None")
        return x

    @property
    def y_global(self) -> float:
        y = self.global_.location.y
        if y is None:
            raise ValueError("x_global is None")
        return y

    def is_global_empty(self) -> bool:
        return self.global_.is_empty()

    def is_container_empty(self) -> bool:
        return self.container.is_empty()

    def set(self, x: float, y: float) -> None:
        self.set_global(x, y)
        self.set_container(x, y)

    def between_two_objects(
        self, position_1: ElementPosition, position_2: ElementPosition
    ) -> None:
        self.set(
            (position_1.location.c_.x - position_2.location.c_.x) / 2,
            (position_1.location.c_.y + position_2.location.c_.y) / 2,
        )


class ComponentModel(BaseModel):
    id: int
    model: str
    container: str


class BaseElementPosition(BaseModel):
    position: BasePosition = Field(default_factory=BasePosition)
    model_config = ConfigDict(validate_assignment=True)


class BaseParameter(BaseModel):
    model_config = ConfigDict(populate_by_name=True, extra="allow")


class MediumTemplate(BaseModel):
    air: Optional[str] = None
    water: Optional[str] = None

    def is_empty(self) -> bool:
        return self.air is None or self.water is None
