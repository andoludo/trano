from functools import partial
from typing import Any, Callable, Dict, List, Literal, Optional

from pydantic import Field, computed_field

from trano.models.constants import Flow
from trano.models.elements.base import (
    AvailableLibraries,
    BaseParameter,
    BaseVariant,
    LibraryData,
    Port,
    exclude_parameters,
    modify_alias,
)
from trano.models.elements.controls.base import Control
from trano.models.elements.space import Space
from trano.models.elements.system import Emission


class RadiatorVariant(BaseVariant):
    ideal: str = "ideal"


class RadiatorParameter(BaseParameter):
    number_of_elements_used_in_the_discretization: int = Field(
        1, alias="nEle", title="Number of elements used in the discretization"
    )
    fraction_radiant_heat_transfer: float = Field(
        0.3, alias="fraRad", title="Fraction radiant heat transfer"
    )
    nominal_heating_power_positive_for_heating: float = Field(
        2000,
        alias="Q_flow_nominal",
        title="Nominal heating power (positive for heating)",
    )
    water_inlet_temperature_at_nominal_condition: Optional[float] = Field(
        273.15 + 90,
        alias="T_a_nominal",
        title="Water inlet temperature at nominal condition",
    )
    water_outlet_temperature_at_nominal_condition: Optional[float] = Field(
        273.15 + 80,
        alias="T_b_nominal",
        title="Water outlet temperature at nominal condition",
    )
    air_temperature_at_nominal_condition: float = Field(
        293.15, alias="TAir_nominal", title="Air temperature at nominal condition"
    )
    radiative_temperature_at_nominal_condition: float = Field(
        293.15, alias="TRad_nominal", title="Radiative temperature at nominal condition"
    )
    exponent_for_heat_transfer: float = Field(
        1.24, alias="n", title="Exponent for heat transfer"
    )
    fraction_of_nominal_mass_flow_rate_where_transition_to_turbulent_occurs: float = Field(
        0.01,
        alias="deltaM",
        title="Fraction of nominal mass flow rate where transition to turbulent occurs",
    )
    use_m_flow_f_dp_else_dp_f_m_flow: Literal["false", "true"] = Field(
        "false", alias="from_dp", title="= true, use m_flow = f(dp) else dp = f(m_flow)"
    )
    dp_nominal: Optional[float] = Field(0, alias="dp_nominal", title="Pa")
    use_linear_relation_between_m_flow_and_dp_for_any_flow_rate: Literal[
        "false", "true"
    ] = Field(
        "false",
        alias="linearized",
        title="= true, use linear relation between m_flow and dp for any flow rate",
    )

    @computed_field(alias="VWat", title="Water volume of radiator")
    def water_volume_of_radiator(self) -> float:
        return 5.8e-5 * abs(self.nominal_heating_power_positive_for_heating)

    @computed_field(
        alias="mDry",
        title="Dry mass of radiator that will be lumped to water heat capacity",
    )
    def dry_mass_of_radiator_that_will_be_lumped_to_water_heat_capacity(self) -> float:
        return 0.0263 * abs(self.nominal_heating_power_positive_for_heating)


class BaseRadiator(LibraryData):
    template: str = """    {{library_name}}.Fluid.HeatExchangers.Radiators.
            RadiatorEN442_2 {{ element.name }}(
            {{ macros.render_parameters(parameters) | safe}},
    redeclare package Medium = MediumW,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial) "Radiator" """
    parameter_processing: Callable[
        [RadiatorParameter], Dict[str, Any]
    ] = exclude_parameters
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(targets=[Space], names=["heatPortCon", "heatPortRad"]),
            Port(names=["port_a"], flow=Flow.inlet),
            Port(names=["port_b"], flow=Flow.outlet),
        ]
    )


class BaseIdealRadiator(LibraryData):
    variant: str = RadiatorVariant.ideal
    template: str = """
    {{package_name}}.Common.HeatTransfer.IdealHeatingSystem.IdealHeatEmission
    {{ element.name }}"""
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(targets=[Space], names=["heatPortCon", "heatPortRad"]),
            Port(targets=[Control], names=["y"]),
        ]
    )
    parameter_processing: Callable[[RadiatorParameter], Dict[str, Any]] = partial(
        modify_alias,
        mapping={
            "nominal_heating_power_positive_for_heating": "power",
            "fraction_radiant_heat_transfer": "frad",
        },
    )


class Radiator(Emission):
    parameters: RadiatorParameter = Field(default=RadiatorParameter())
    # libraries_data: AvailableLibraries = AvailableLibraries(
    #     ideas=[BaseRadiator, BaseIdealRadiator],
    #     buildings=[BaseRadiator, BaseIdealRadiator],
    # )
