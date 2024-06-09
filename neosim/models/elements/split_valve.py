from typing import Callable, List, Optional

from pydantic import Field, field_serializer

from neosim.models.constants import Flow
from neosim.models.elements.base import (
    AvailableLibraries,
    BaseParameter,
    Boolean,
    LibraryData,
    Port,
)
from neosim.models.elements.system import System


class SplitValveParameters(BaseParameter):
    m_flow_nominal: float = Field(
        0.008,
        title="Mass flow rate. Set negative at outflowing ports.",
    )
    dp_nominal: str = Field("{10000,-1,-1}", alias="dp_nominal", title="Pa")
    fraction_of_nominal_mass_flow_rate_where_transition_to_turbulent_occurs: float = Field(
        0.3,
        alias="deltaM",
        title="Fraction of nominal mass flow rate where transition to turbulent occurs",
    )
    use_linear_relation_between_m_flow_and_dp_for_any_flow_rate: Boolean = Field(
        "false",
        alias="linearized",
        title="= true, use linear relation between m_flow and dp for any flow rate",
    )
    time_constant_at_nominal_flow_for_dynamic_energy_and_momentum_balance: Optional[
        float
    ] = Field(
        None,
        alias="tau",
        title="Time constant at nominal flow for dynamic energy and momentum balance",
    )
    nominal_mass_flow_rate_for_dynamic_momentum_and_energy_balance: Optional[
        float
    ] = Field(
        None,
        alias="mDyn_flow_nominal",
        title="Nominal mass flow rate for dynamic momentum and energy balance",
    )

    @field_serializer("m_flow_nominal")
    @staticmethod
    def m_flow_nominal_serializer(m_flow_nominal: float) -> str:
        return f"{m_flow_nominal}" + "*{1, -1, -1}"


class BaseSplitValve(LibraryData):
    template: str = """    {{library_name}}.Fluid.FixedResistances.Junction {{ element.name }} (
    {{ macros.render_parameters(parameters) | safe}},
    redeclare package Medium = MediumW,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState)
    "Flow splitter" """
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(
                names=["port_1"],
                flow=Flow.inlet,
                multi_connection=True,
                use_counter=False,
            ),
            Port(names=["port_2"], flow=Flow.outlet),
            Port(names=["port_3"], flow=Flow.inlet_or_outlet),
        ]
    )


class SplitValve(System):
    parameters: SplitValveParameters = Field(default=SplitValveParameters())
    libraries_data: AvailableLibraries = AvailableLibraries(
        ideas=[BaseSplitValve],
        buildings=[BaseSplitValve],
    )
