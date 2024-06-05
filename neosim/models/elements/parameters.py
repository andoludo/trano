from typing import Literal, Optional

from pydantic import BaseModel, Field, computed_field


class RadiatorParameter(BaseModel):
    number_of_elements_used_in_the_discretization: int = Field(
        1, alias="nEle", title="Number of elements used in the discretization"
    )
    fraction_radiant_heat_transfer: float = Field(
        0, alias="fraRad", title="Fraction radiant heat transfer"
    )
    nominal_heating_power_positive_for_heating: Optional[float] = Field(
        2000,
        alias="Q_flow_nominal",
        title="Nominal heating power (positive for heating)",
    )
    water_inlet_temperature_at_nominal_condition: Optional[float] = Field(
        273.15 + 80,
        alias="T_a_nominal",
        title="Water inlet temperature at nominal condition",
    )
    water_outlet_temperature_at_nominal_condition: Optional[float] = Field(
        273.15 + 90,
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
        return self.water_volume_of_radiator

    @computed_field(
        alias="mDry",
        title="Dry mass of radiator that will be lumped to water heat capacity",
    )
    def dry_mass_of_radiator_that_will_be_lumped_to_water_heat_capacity(self) -> float:
        return self.dry_mass_of_radiator_that_will_be_lumped_to_water_heat_capacity
