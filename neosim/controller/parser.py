import json
from pathlib import Path

from pydantic import BaseModel, Field, computed_field


class BaseInput(BaseModel):
    component: str
    port: str


class RealInput(BaseInput):
    name: str
    default: float = 0.0
    target: str = "Space"

    @computed_field
    def default_template(self) -> str:
        return (
            f"""Modelica.Blocks.Sources.RealExpression {self.name}(y={self.default});"""
        )


class IntegerInput(BaseInput):
    name: str
    default: int = 0.0
    target: str = "Space"

    @computed_field
    def default_template(self) -> str:
        return f"""Modelica.Blocks.Sources.IntegerExpression {self.name}(y={self.default});"""


class BooleanInput(BaseInput):
    name: str
    default: bool = False
    target: str = "Space"

    @computed_field
    def default_template(self) -> str:
        return f"""Modelica.Blocks.Sources.BooleanExpression {self.name}(y={self.default});"""


class BooleanOutput(BaseInput):
    name: str
    default: bool = False
    target: str = "Controlled"


class IntegerOutput(BaseInput):
    name: str
    default: int = 0.0
    target: str = "Controlled"


class RealOutput(BaseInput):
    name: str
    default: float = 0.0
    target: str = "Controlled"


class ControllerBus(BaseModel):
    template: str = """Controls.BaseClasses.DataBus dataBus annotation (Placement(transformation(
  extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-120,62},{-78,98}})));"""
    real_inputs: list[RealInput] = Field(default=[])
    real_outputs: list[RealOutput] = Field(default=[])
    integer_inputs: list[IntegerInput] = Field(default=[])
    integer_outputs: list[IntegerOutput] = Field(default=[])
    boolean_inputs: list[BooleanInput] = Field(default=[])
    boolean_outputs: list[BooleanOutput] = Field(default=[])

    @classmethod
    def from_configuration(cls, file_path: Path):
        return cls(**json.loads(file_path.read_text()))

    def inputs(self):
        return (
            self.real_inputs
            + self.real_outputs
            + self.integer_inputs
            + self.integer_outputs
            + self.boolean_inputs
            + self.boolean_outputs
        )

    def _get_targets(self):
        return {
            input.target: [
                input_ for input_ in self.inputs() if input_.target == input.target
            ]
            for input in self.inputs()
        }

    def list_ports(self, element, **kwargs) -> str:
        ports = {
            "RealOutput": [],
            "RealInput": [],
            "IntegerOutput": [],
            "IntegerInput": [],
            "BooleanOutput": [],
            "BooleanInput": [],
        }
        for target, inputs in self._get_targets().items():
            target_value = eval(target)
            for input in inputs:
                if isinstance(target_value, list):
                    for i, target_ in enumerate(target_value):
                        ports[type(input).__name__].append(
                            (input.name, target_.capitalize())
                        )
                else:
                    ports[type(input).__name__].append(
                        (input.name, target_value.capitalize())
                    )
        return ports

    def bus_ports(self, element, **kwargs) -> str:
        ports = []
        for target, inputs in self._get_targets().items():
            target_value = eval(target)
            for input in inputs:
                if isinstance(target_value, list):
                    for i, target_ in enumerate(target_value):
                        ports.append(

                                f"connect(dataBus.{input.name}{target_.capitalize()}, "
                                f"{input.component}[{i+1}].{input.port});"

                        )
                else:
                    ports.append(

                            f"connect(dataBus.{input.name}{target_value.capitalize()}, "
                            f"{input.component}.{input.port});"

                    )
        return ports
