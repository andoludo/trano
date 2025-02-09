from typing import Optional, List, Dict, Any, Tuple

from pydantic import BaseModel, Field, field_validator, ConfigDict

from trano.elements.types import Medium


class ElementLocation(BaseModel):
    x: Optional[float] = None
    y: Optional[float] = None

    def is_valid(self) ->bool:
        return self.x is not None and self.y is not None


class ElementPosition(BaseModel):
    location: ElementLocation = Field(default_factory=ElementLocation)
    annotation: str = """annotation (
    Placement(transformation(origin = {{ macros.join_list(element.location) }},
    extent = {% raw %}{{10, -10}, {-10, 10}}
    {% endraw %})));"""

    def is_empty(self) ->bool:
        return not self.location.is_valid()
    def coordinate(self)-> Tuple[float, float]:
        return self.location.x, self.location.y


class BasePosition(BaseModel):
    container:ElementPosition = Field(default_factory=ElementPosition)
    global_:ElementPosition = Field(default_factory=ElementPosition)

    def set_global(self, x: float, y: float) -> None:
        self.global_.location.x = x
        self.global_.location.y = y

    def set_container(self, x: float, y: float) -> None:
        self.container.location.x = x
        self.container.location.y = y
    def is_empty(self) -> bool:
        return self.container.is_empty() and self.global_.is_empty()

    def set(self, x: float, y: float) -> None:
        self.set_global(x, y)
        self.set_container(x, y)
    def between_two_objects(self, position_1: ElementPosition, position_2: ElementPosition) -> None:
        self.set((position_1.location.x - position_2.location.x) / 2, (position_1.location.y + position_2.location.y) / 2)




class BaseElementPosition(BaseModel):
    position: BasePosition = Field(default_factory=BasePosition)
    model_config = ConfigDict(validate_assignment=True)



class MediumLimitConnection(BaseModel):
    number: int
    medium: Medium

class LimitConnection(BaseModel):
    limits: List[MediumLimitConnection] = Field(default_factory=list)

    def limit_reached(self, current_connections: int, medium: Medium) -> bool:
        return any(limit.number <= current_connections for limit in self.limits if limit.medium == medium)

    def medium_has_limit(self, medium: Medium) -> bool:
        return any(limit.medium == medium for limit in self.limits)
