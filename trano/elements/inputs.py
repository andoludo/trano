from typing import Optional

from pydantic import BaseModel, computed_field, field_validator


class Target(BaseModel):
    main: str
    sub: Optional[str] = None
    evaluated_element: str = ""
    value: str = ""
    @field_validator("value")
    def validate_value(cls, value: str) -> str:
        return value.capitalize()

    def __hash__(self) -> int:
        return hash((self.main, self.sub))
    def commands(self)->list[str]:
        return self.main.split(".")[1:]

    def sub_commands(self):
        if self.sub is None:
            raise Exception("Target sub is None")
        return self.sub.split(".")

    def capitalize(self):
        self.main = self.main.capitalize()
        return self



class BaseInput(BaseModel):
    name: str
    component: str
    port: str
    multi: bool = False
    target: Target
    input_template: str
    default: float | str | int

    def __hash__(self) -> int:
        return hash((self.name, self.target.value))

    def __eq__(self, other: object) -> bool:
        return hash(self) == hash(other)

    @computed_field  # type: ignore
    @property
    def input_model(self) -> str:
        if self.target.evaluated_element:
            return f"""{self.input_template}
            {self.name}{self.target.evaluated_element.capitalize()}
            (y={self.default});"""
        return ""



class RealInput(BaseInput):
    default: float = 0.0
    input_template: str = "Modelica.Blocks.Sources.RealExpression"


class IntegerInput(BaseInput):
    default: int = 0
    input_template: str = "Modelica.Blocks.Sources.IntegerExpression"


class BooleanInput(BaseInput):
    default: str = "false"
    input_template: str = "Modelica.Blocks.Sources.BooleanExpression"


class BooleanOutput(BaseInput):
    default: str = "false"
    input_template: str = "Modelica.Blocks.Sources.BooleanExpression"


class IntegerOutput(BaseInput):
    default: int = 0
    input_template: str = "Modelica.Blocks.Sources.IntegerExpression"


class RealOutput(BaseInput):
    default: float = 0.0
    input_template: str = "Modelica.Blocks.Sources.RealExpression"
