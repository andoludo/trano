from typing import Callable, List, Optional

from pydantic import Field

from trano.controller.parser import ControllerBus, RealInput, RealOutput
from trano.models.constants import Flow
from trano.models.elements.base import (
    AvailableLibraries,
    BaseParameter,
    DynamicComponentTemplate,
    LibraryData,
    Port,
)
from trano.models.elements.bus import DataBus
from trano.models.elements.controls.base import Control
from trano.models.elements.system import System


class PumpParameters(BaseParameter):
    m_flow_nominal: Optional[float] = Field(
        0.008,
        alias="m_flow_nominal",
        title="Nominal mass flow rate for configuration of pressure curve",
    )
    dp_nominal: Optional[float] = Field(
        10000,
        alias="dp_nominal",
        title="Nominal pressure head for configuration of pressure curve",
    )
    per: Optional[str] = Field(None, alias="per", title="Record with performance data")
    control_input_type: Optional[str] = Field(
        None,
        alias="inputType",
        title="Control input type",
    )
    constant_input_set_point: Optional[float] = Field(
        None, alias="constInput", title="Constant input set point"
    )
    vector_of_input_set_points_corresponding_to_stages: Optional[str] = Field(
        None,
        alias="stageInputs[:]",
        title="Vector of input set points corresponding to stages",
    )
    compute_power_using_similarity_laws: Optional[bool] = Field(
        None,
        alias="computePowerUsingSimilarityLaws",
        title="= true, compute power exactly, using similarity laws. Otherwise approximate.",
    )

    class Config:
        allow_population_by_field_name = True


dynamic_pump_template = DynamicComponentTemplate(
    template="""
model Pump{{ element.name | capitalize}}
extends {{ package_name }}.Common.Fluid.Ventilation.PartialPump;
{{bus_template}}
equation
{{bus_ports | safe}}
 end Pump{{ element.name | capitalize}};
 """,
    category="ventilation",
    bus=ControllerBus(
        real_inputs=[
            RealInput(name="y", target="element.name", component="pumRad", port="y")
        ],
        real_outputs=[
            RealOutput(
                name="y_gain",
                target="element.name",
                component="gain",
                port="y",
            ),
            RealOutput(
                name="T",
                target="element.control.name",
                component="temSup",
                port="T",
            ),
        ],
    ),
)


class BasePump(LibraryData):
    template: str = """  {{ package_name }}.Common.
    Fluid.Ventilation.Pump{{ element.name | capitalize }}
     {{ element.name }}(
     {{ macros.render_parameters(parameters) | safe}},
    redeclare package Medium = MediumW

    )"""
    component_template: DynamicComponentTemplate = dynamic_pump_template
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(
                names=["port_a"],
                flow=Flow.inlet,
                multi_connection=True,
                use_counter=False,
            ),
            Port(
                names=["port_b"],
                flow=Flow.outlet,
                multi_connection=True,
                use_counter=False,
            ),
            Port(
                targets=[Control, DataBus],
                names=["dataBus"],
                multi_connection=True,
                use_counter=False,
            ),
        ]
    )


class Pump(System):
    parameters: PumpParameters = Field(default=PumpParameters())
    libraries_data: AvailableLibraries = AvailableLibraries(
        ideas=[BasePump],
        buildings=[BasePump],
    )
