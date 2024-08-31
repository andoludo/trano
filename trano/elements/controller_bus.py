import ast
import copy
import json
from pathlib import Path
from typing import TYPE_CHECKING, Any, Dict, List

from pydantic import BaseModel, Field

from trano.elements.inputs import (
    BaseInput,
    BooleanInput,
    BooleanOutput,
    IntegerInput,
    IntegerOutput,
    RealInput,
    RealOutput,
    Target,
)

if TYPE_CHECKING:
    from trano.elements import BaseElement


def _evaluate(element: "BaseElement", commands: List[str]) -> Any:
    for command in commands:
        if hasattr(element, command):
            element = getattr(element, command)
        else:
            raise Exception(f" Element {element.name} has no command {command}")
    return element


def _evaluate_target(target: Target, element: "BaseElement"):
    from trano.elements import BaseElement

    target_ = _evaluate(element, target.commands())
    if target_ is None:
        raise Exception("Target value is None")
    if isinstance(target_, list) and isinstance(target_[0], BaseElement):
        return [_evaluate(sub, target.sub_commands()) for sub in target_]
    return target_


def _append_to_port(
    input_: BaseInput,
    ports: Dict[str, List[BaseInput]],
    target: Target,
    evaluated_element: str,
    element: "BaseElement",
) -> Dict[str, List[BaseInput]]:
    ports[type(input_).__name__].append(
        type(input_)(
            **(
                input_.model_dump()
                | {
                    "target": Target(
                        **(
                            target.model_dump()
                            | {
                                "evaluated_element": element.name,
                                "value": evaluated_element,
                            }
                        )
                    ),
                }
            )
        )
    )
    return ports


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

    def _get_targets(self) -> Dict[Target, List[BaseInput]]:
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
            evaluated_element = _evaluate_target(target, element)
            for input in inputs:
                if isinstance(evaluated_element, list):
                    for evaluated_element_ in evaluated_element:
                        ports = _append_to_port(
                            input, ports, target, evaluated_element_, element
                        )
                else:
                    ports = _append_to_port(
                        input, ports, target, evaluated_element, element
                    )
        return ports

    def bus_ports(
        self, element: "BaseElement", **kwargs: Any  # noqa: ANN401
    ) -> List[str]:
        ports: List[str] = []
        for target, inputs in self._get_targets().items():
            target_value = _evaluate_target(target, element)
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
