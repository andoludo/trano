import json
from pathlib import Path
from typing import List, Dict, Any, TYPE_CHECKING

from pydantic import BaseModel, Field


from trano.elements.inputs import RealInput, RealOutput, IntegerInput, IntegerOutput, BooleanInput, BooleanOutput, \
    BaseInput

if TYPE_CHECKING:
    from trano.elements import BaseElement

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
            target_value = eval(target)  # noqa: S307
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
