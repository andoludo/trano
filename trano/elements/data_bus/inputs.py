from typing import List, Optional, Literal

from pydantic import BaseModel, computed_field, field_validator


class Target(BaseModel):
    main: str
    sub: Optional[str] = None
    evaluated_element: str = ""
    value: str = ""

    @field_validator("value")
    @classmethod
    def validate_value(cls, value: str) -> str:
        return value.capitalize()

    def __hash__(self) -> int:
        return hash((self.main, self.sub))

    def commands(self) -> list[str]:
        return self.main.split(".")[1:]

    def sub_commands(self) -> List[str]:
        if self.sub is None:
            raise Exception("Target sub is None")
        return self.sub.split(".")


LoadType = Literal["Resistive", "Inductive", "Capacitive"]
PrimaryEnergyType = Literal["Electricity", "Gas"]


class PowerInput(BaseModel):
    energy: PrimaryEnergyType
    load_type: Optional[LoadType] = None

    def name(self) -> str:
        if self.load_type:
            return f"{self.energy.lower()}_{self.load_type}"
        return self.energy.lower()


class BaseInputOutput(BaseModel):
    name: str
    component: str
    port: str
    multi: bool = False
    target: Target
    input_template: str
    default: float | str | int
    power: Optional[PowerInput] = None

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

    @computed_field  # type: ignore
    @property
    def input_name(self) -> str:
        if self.power is None:
            return self.name
        return self.power.name()


class BaseInput(BaseInputOutput): ...


class RealInput(BaseInput):
    default: float = 0.0
    input_template: str = "Modelica.Blocks.Sources.RealExpression"


class IntegerInput(BaseInput):
    default: int = 0
    input_template: str = "Modelica.Blocks.Sources.IntegerExpression"


class BooleanInput(BaseInput):
    default: str = "false"
    input_template: str = "Modelica.Blocks.Sources.BooleanExpression"


class BooleanOutput(BaseInputOutput):
    default: str = "false"
    input_template: str = "Modelica.Blocks.Sources.BooleanExpression"


class IntegerOutput(BaseInputOutput):
    default: int = 0
    input_template: str = "Modelica.Blocks.Sources.IntegerExpression"


class RealOutput(BaseInputOutput):
    default: float = 0.0
    input_template: str = "Modelica.Blocks.Sources.RealExpression"
