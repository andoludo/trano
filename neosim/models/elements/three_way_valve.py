from typing import Callable, List, Optional

from pydantic import Field

from neosim.models.constants import Flow
from neosim.models.elements.base import (
    AvailableLibraries,
    BaseParameter,
    LibraryData,
    Port,
)
from neosim.models.elements.controls.base import Control
from neosim.models.elements.system import System


class ThreeWayValveParameters(BaseParameter):
    rangeability: float = Field(
        50, alias="R", title="Rangeability, R=50...100 typically"
    )
    range_of_significant_deviation_from_equal_percentage_law: float = Field(
        0.01,
        alias="delta0",
        title="Range of significant deviation from equal percentage law",
    )
    dpFixed_nominal: str = Field(
        "{100,0}",
        alias="dpFixed_nominal",
        title="Nominal pressure drop of pipes and other equipment in flow legs at port_1 and port_3",
    )
    fraK: float = Field(0.7, alias="fraK", title=None)
    valve_leakage: str = Field(
        "{0.01,0.01}", alias="l", title="Valve leakage, l=Kv(y=0)/Kv(y=1)"
    )
    fraction_of_nominal_flow_rate_where_linearization_starts_if_y_1: float = Field(
        0.02,
        alias="deltaM",
        title="Fraction of nominal flow rate where linearization starts, if y=1",
    )
    use_linear_relation_between_m_flow_and_dp_for_any_flow_rate: str = Field(
        "{false, false}",
        alias="linearized",
        title="= true, use linear relation between m_flow and dp for any flow rate",
    )
    Kv: Optional[float] = Field(
        None, alias="Kv", title="Kv (metric) flow coefficient [m3/h/(bar)^(1/2)]"
    )
    Cv: Optional[float] = Field(
        None, alias="Cv", title="Cv (US) flow coefficient [USG/min/(psi)^(1/2)]"
    )
    Av: Optional[str] = Field(None, alias="Av", title="Av (metric) flow coefficient")
    m_flow_nominal: Optional[float] = Field(
        0.0078, alias="m_flow_nominal", title="Nominal mass flow rate"
    )
    dp_valve_nominal: Optional[float] = Field(6000, alias="dpValve_nominal", title="Pa")
    rhoStd: Optional[float] = Field(
        None,
        alias="rhoStd",
        title="Inlet density for which valve coefficients are defined",
    )


class BaseThreeWayValve(LibraryData):
    template: str = """    {{library_name}}.Fluid.Actuators.Valves.ThreeWayEqualPercentageLinear
             {{ element.name }}(
    redeclare package Medium = MediumW,
    use_inputFilter=false,
    {{ macros.render_parameters(parameters) | safe}},
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState) "Three-wayvalve" """
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(names=["port_1"], flow=Flow.inlet),
            Port(
                names=["port_2"],
                flow=Flow.outlet,
                # multi_connection=True,
                # use_counter=False,
            ),
            Port(names=["port_3"], flow=Flow.inlet_or_outlet),
            Port(targets=[Control], names=["y"]),
        ]
    )


class ThreeWayValve(System):
    parameters: ThreeWayValveParameters = Field(default=ThreeWayValveParameters())
    libraries_data: AvailableLibraries = AvailableLibraries(
        ideas=[BaseThreeWayValve],
        buildings=[BaseThreeWayValve],
    )
