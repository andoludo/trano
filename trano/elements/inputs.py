from pydantic import BaseModel, computed_field


class BaseInput(BaseModel):
    name: str
    component: str
    port: str
    multi: bool = False
    target: str
    input_template: str
    default: float | str | int
    evaluated_element_name: str = ""

    def __hash__(self) -> int:
        return hash((self.name, self.target))

    def __eq__(self, other: object) -> bool:
        return hash(self) == hash(other)

    @computed_field  # type: ignore
    @property
    def input_model(self) -> str:
        if self.evaluated_element_name:
            return f"""{self.input_template}
            {self.name}{self.evaluated_element_name.capitalize()}
            (y={self.default});"""
        return ""


class RealInput(BaseInput):
    default: float = 0.0
    target: str = "Space"
    input_template: str = "Modelica.Blocks.Sources.RealExpression"


class IntegerInput(BaseInput):
    default: int = 0
    target: str = "Space"
    input_template: str = "Modelica.Blocks.Sources.IntegerExpression"


class BooleanInput(BaseInput):
    default: str = "false"
    target: str = "Space"
    input_template: str = "Modelica.Blocks.Sources.BooleanExpression"


class BooleanOutput(BaseInput):
    default: str = "false"
    target: str = "Controlled"
    input_template: str = "Modelica.Blocks.Sources.BooleanExpression"


class IntegerOutput(BaseInput):
    default: int = 0
    target: str = "Controlled"
    input_template: str = "Modelica.Blocks.Sources.IntegerExpression"


class RealOutput(BaseInput):
    default: float = 0.0
    target: str = "Controlled"
    input_template: str = "Modelica.Blocks.Sources.RealExpression"
