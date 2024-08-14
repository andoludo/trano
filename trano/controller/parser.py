import json
from pathlib import Path
from typing import TYPE_CHECKING, Any, Dict, List

from pydantic import BaseModel, Field, computed_field

if TYPE_CHECKING:

    from trano.models.elements.base import BaseElement


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


class ControllerBus(BaseModel):
    template: str = """Controls.BaseClasses.DataBus dataBus
    annotation (Placement(transformation(
  extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-120,62},{-78,98}})));"""
    real_inputs: list[RealInput] = Field(default=[])
    real_outputs: list[RealOutput] = Field(default=[])
    integer_inputs: list[IntegerInput] = Field(default=[])
    integer_outputs: list[IntegerOutput] = Field(default=[])
    boolean_inputs: list[BooleanInput] = Field(default=[])
    boolean_outputs: list[BooleanOutput] = Field(default=[])

    @classmethod
    def from_configuration(cls, file_path: Path) -> "ControllerBus":
        return cls(**json.loads(file_path.read_text()))

    def inputs(
        self,
    ) -> List[
        BooleanInput
        | IntegerOutput
        | IntegerInput
        | RealOutput
        | RealInput
        | BooleanOutput
    ]:
        return (
            self.real_inputs
            + self.real_outputs
            + self.integer_inputs
            + self.integer_outputs
            + self.boolean_inputs
            + self.boolean_outputs
        )

    def _get_targets(self) -> Dict[str, List[BaseInput]]:
        return {
            input.target: [
                input_ for input_ in self.inputs() if input_.target == input.target
            ]
            for input in self.inputs()
        }

    def list_ports(
        self, element: "BaseElement", **kwargs: Any  # noqa: ANN401
    ) -> Dict[str, List[BaseInput]]:
        ports: Dict[str, List[BaseInput]] = {
            "RealOutput": [],
            "RealInput": [],
            "IntegerOutput": [],
            "IntegerInput": [],
            "BooleanOutput": [],
            "BooleanInput": [],
        }
        for target, inputs in self._get_targets().items():
            # TODO: Fix this
            try:
                target_value = eval(target)  # noqa: S307
            except:
                raise
            if target_value is None:
                raise Exception("Target value is None")
            for input in inputs:
                if isinstance(target_value, list):
                    for target_ in target_value:
                        ports[type(input).__name__].append(
                            type(input)(
                                **(
                                    input.model_dump()
                                    | {
                                        "target": target_.capitalize(),
                                        "evaluated_element_name": element.name,
                                    }
                                )
                            )
                        )
                else:
                    ports[type(input).__name__].append(
                        type(input)(
                            **(
                                input.model_dump()
                                | {
                                    "target": target_value.capitalize(),
                                    "evaluated_element_name": element.name,
                                }
                            )
                        )
                    )
        return ports

    def bus_ports(
        self, element: "BaseElement", **kwargs: Any  # noqa: ANN401
    ) -> List[str]:
        ports: List[str] = []
        for target, inputs in self._get_targets().items():
            # TODO: Fix this
            target_value = eval(target)  # noqa: S307
            for input in inputs:
                if isinstance(target_value, list):
                    for i, target_ in enumerate(target_value):
                        if input.multi:
                            ports.append(
                                f"connect(dataBus.{input.name}{target_.capitalize()}, "
                                f"{input.component}.{input.port}[{i + 1}]);"
                            )

                        else:
                            ports.append(
                                f"connect(dataBus.{input.name}{target_.capitalize()}, "
                                f"{input.component}[{i + 1}].{input.port});"
                            )
                else:  # noqa : PLR5501
                    if input.port:
                        ports.append(
                            f"connect(dataBus.{input.name}{target_value.capitalize()}, "
                            f"{input.component}.{input.port});"
                        )
                    else:
                        ports.append(
                            f"connect(dataBus.{input.name}{target_value.capitalize()}, "
                            f"{input.component});"
                        )
        return ports
