from typing import Callable, List, Optional

from pydantic import Field

from neosim.models.constants import Flow
from neosim.models.elements.base import (
    AvailableLibraries,
    BaseParameter,
    Boolean,
    LibraryData,
    Port,
)
from neosim.models.elements.controls.base import Control
from neosim.models.elements.system import SpaceSystem


class ValveParameters(BaseParameter):
    rangeability: float = Field(
        50, alias="R", title="Rangeability, R=50...100 typically"
    )
    range_of_significant_deviation_from_equal_percentage_law: float = Field(
        0.01,
        alias="delta0",
        title="Range of significant deviation from equal percentage law",
    )
    dpFixed_nominal: Optional[float] = Field(6000, alias="dpFixed_nominal", title="Pa")
    valve_leakage: float = Field(
        0.0001, alias="l", title="Valve leakage, l=Kv(y=0)/Kv(y=1)"
    )
    k_fixed: Optional[str] = Field(None, alias="kFixed", title="")
    use_m_flow_f_dp_else_dp_f_m_flow: Boolean = Field(
        "true", alias="from_dp", title="= true, use m_flow = f(dp) else dp = f(m_flow)"
    )
    use_linear_relation_between_m_flow_and_dp_for_any_flow_rate: Boolean = Field(
        "false",
        alias="linearized",
        title="= true, use linear relation between m_flow and dp for any flow rate",
    )
    kv: Optional[float] = Field(
        None, alias="Kv", title="Kv (metric) flow coefficient [m3/h/(bar)^(1/2)]"
    )
    cv: Optional[float] = Field(
        None, alias="Cv", title="Cv (US) flow coefficient [USG/min/(psi)^(1/2)]"
    )
    av: Optional[str] = Field(None, alias="Av", title="Av (metric) flow coefficient")
    fraction_of_nominal_flow_rate_where_linearization_starts_if_y_1: float = Field(
        0.02,
        alias="deltaM",
        title="Fraction of nominal flow rate where linearization starts, if y=1",
    )
    m_flow_nominal: Optional[float] = Field(
        0.01, alias="m_flow_nominal", title="Nominal mass flow rate"
    )
    dp_valve_nominal: Optional[float] = Field(6000, alias="dpValve_nominal", title="Pa")

    class Config:
        allow_population_by_field_name = True


class BaseValve(LibraryData):
    template: str = """    {{ library_name }}.Fluid.Actuators.Valves.TwoWayEqualPercentage
            {{ element.name }}(
                {{ macros.render_parameters(parameters) | safe}},
    redeclare package Medium = MediumW

    ) "Radiator valve" """
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(names=["port_a"], flow=Flow.inlet),
            Port(names=["port_b"], flow=Flow.outlet),
            Port(targets=[Control], names=["y"]),
        ]
    )


class Valve(SpaceSystem):
    parameters: ValveParameters = Field(default=ValveParameters())
    libraries_data: AvailableLibraries = AvailableLibraries(
        ideas=[BaseValve],
        buildings=[BaseValve],
    )
