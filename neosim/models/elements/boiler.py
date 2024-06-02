from typing import Callable, List, Optional

from pydantic import Field

from neosim.controller.parser import ControllerBus, RealInput, RealOutput
from neosim.models.constants import Flow
from neosim.models.elements.base import (
    AvailableLibraries,
    BaseParameter,
    Boolean,
    DynamicComponentTemplate,
    LibraryData,
    Port,
)
from neosim.models.elements.control import Control, DataBus
from neosim.models.elements.system import System


class BoilerParameters(BaseParameter):
    # boiler
    coefficients_for_efficiency_curve: str = Field(
        "{0.9}", alias="a", title="Coefficients for efficiency curve"
    )
    effcur: str = Field(
        "Buildings.Fluid.Types.EfficiencyCurves.Constant", alias="effCur", title=""
    )
    temperature_used_to_compute_nominal_efficiency: float = Field(
        353.15,
        alias="T_nominal",
        title="Temperature used to compute nominal efficiency (only used if efficiency curve depends on temperature)",
    )
    fuel_type: Optional[str] = Field(
        "Buildings.Fluid.Data.Fuels.HeatingOilLowerHeatingValue()",
        alias="fue",
        title="Fuel type",
    )
    nominal_heating_power: Optional[float] = Field(
        2000, alias="Q_flow_nominal", title="Nominal heating power"
    )
    dp_nominal: Optional[float] = Field(5000, alias="dp_nominal", title="Pa")
    use_linear_relation_between_m_flow_and_dp_for_any_flow_rate: Boolean = Field(
        "false",
        alias="linearizeFlowResistance",
        title="= true, use linear relation between m_flow and dp for any flow rate",
    )
    fraction_of_nominal_flow_rate_where_flow_transitions_to_laminar: float = Field(
        0.1,
        alias="deltaM",
        title="Fraction of nominal flow rate where flow transitions to laminar",
    )
    nominal_mass_flow_rate: Optional[float] = Field(
        0.008, alias="m_flow_nominal", title="Nominal mass flow rate"
    )

    if_actual_temperature_at_port_is_computed: Boolean = Field(
        "false",
        alias="show_T",
        title="= true, if actual temperature at port is computed",
    )
    tank_volume: Optional[float] = Field(0.2, alias="VTan", title="Tank volume")
    height_of_tank_without_insulation: Optional[float] = Field(
        2, alias="hTan", title="Height of tank (without insulation)"
    )
    number_of_volume_segments: int = Field(
        4, alias="nSeg", title="Number of volume segments"
    )
    thickness_of_insulation: int = Field(
        0.002, alias="dIns", title="Thickness of insulation"
    )

    class Config:
        allow_population_by_field_name = True


dynamic_boiler_template = DynamicComponentTemplate(
    template="""
    model BoilerWithStorage{{ element.name | capitalize}}
    extends {{ package_name }}.Common.Fluid.Boilers.PartialBoilerWithStorage;
    {{bus_template}}
    equation
    {{bus_ports | safe}}
     end BoilerWithStorage{{ element.name | capitalize}};
     """,
    category="boiler",
    bus=ControllerBus(
        real_inputs=[
            RealInput(name="yBoiCon", target="element.name", component="boi", port="y"),
            RealInput(
                name="yPumBoi", target="element.name", component="pumBoi", port="y"
            ),
        ],
        real_outputs=[
            RealOutput(
                name="TStoTop", target="element.name", component="tanTemTop", port="T"
            ),
            RealOutput(
                name="TStoBot",
                target="element.name",
                component="tanTemBot",
                port="T",
            ),
        ],
    ),
)


class BaseBoiler(LibraryData):
    template: str = """
    {{package_name}}.Common.Fluid.Boilers.BoilerWithStorage{{ element.name | capitalize}} {{ element.name }}(
    {{ macros.render_parameters(parameters) | safe}}
    redeclare package MediumW = MediumW) "Boiler" """
    component_template: Optional[DynamicComponentTemplate] = dynamic_boiler_template
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


class Boiler(System):
    parameters: BoilerParameters = Field(default=BoilerParameters())
    libraries_data: AvailableLibraries = AvailableLibraries(
        ideas=[BaseBoiler()],
        buildings=[BaseBoiler()],
    )
