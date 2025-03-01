import json
import logging
from pathlib import Path
from typing import TYPE_CHECKING, Any, Dict, List, Optional, Union

from pydantic import BaseModel, Field

from trano.elements.data_bus.inputs import (
    BaseInputOutput,
    BooleanInput,
    BooleanOutput,
    IntegerInput,
    IntegerOutput,
    RealInput,
    RealOutput,
    Target,
)
from trano.exceptions import ControllerBusPortError

logger = logging.getLogger(__name__)
if TYPE_CHECKING:
    from trano.elements import BaseElement


def _evaluate(
    element: "BaseElement", commands: List[str]
) -> Optional[Union[str, "BaseElement", List["BaseElement"], List[str]]]:
    for command in commands:
        if hasattr(element, command):
            element = getattr(element, command)
        else:
            raise Exception(f" Element {element.name} has no command {command}")
    return element


def _evaluate_target(target: Target, element: "BaseElement") -> str | List[str]:
    from trano.elements import BaseElement

    target_ = _evaluate(element, target.commands())
    if not target_:
        message = (
            f"Element {element.name} of type {type(element).__name__} "
            f"has no valid target {target.commands()}. "
            f"This indicates that the selected element is incompatible."
        )
        logger.error(message)
        raise ControllerBusPortError(message)
    if isinstance(target_, list) and all(isinstance(t, BaseElement) for t in target_):
        return [
            _evaluate(sub, target.sub_commands())  # type: ignore
            for sub in target_
            if isinstance(sub, BaseElement)
        ]
    return target_  # type: ignore


def _append_to_port(
    input_: BaseInputOutput,
    ports: Dict[str, List[BaseInputOutput]],
    target: Target,
    evaluated_element: str,
    element: "BaseElement",
) -> Dict[str, List[BaseInputOutput]]:
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
    template: str = """Trano.Controls.BaseClasses.DataBus dataBus
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

    def main_targets(self) -> List[str]:
        return list({input.target.main for input in self.inputs()})

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

    def _get_targets(self) -> Dict[Target, List[BaseInputOutput]]:
        return {
            input.target: [
                input_ for input_ in self.inputs() if input_.target == input.target
            ]
            for input in self.inputs()
        }

    def list_ports(
        self, element: "BaseElement", **kwargs: Any  # noqa: ANN401
    ) -> Dict[str, List[BaseInputOutput]]:
        ports: Dict[str, List[BaseInputOutput]] = {
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
                        ports = _append_ports(
                            ports,
                            input,
                            target_,
                            "multi",
                            f".{input.port}[{i + 1}]",
                            f"[{i + 1}].{input.port}",
                        )
                else:  # : PLR5501
                    ports = _append_ports(
                        ports,
                        input,
                        target_value,
                        "port",
                        f".{input.port}",
                        "",
                    )
        return ports


def _append_ports(  # noqa: PLR0913
    ports: List[str],
    input: BaseInputOutput,
    evaluated_target: str,
    case_1_condition: str,
    case_1: str,
    case_2: str,
) -> List[str]:
    if getattr(input, case_1_condition):

        ports.append(
            f"connect(dataBus.{input.input_name}{evaluated_target.capitalize()}, "
            f"{input.component}{case_1});"
        )
    else:
        ports.append(
            f"connect(dataBus.{input.input_name}{evaluated_target.capitalize()}, "
            f"{input.component}{case_2});"
        )
    return ports
