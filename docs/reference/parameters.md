# Parameters

Every parameter is optional: a parameter left out of the YAML is not written to the Modelica model, so the library's own default applies ("library default" below), unless trano has a default of its own. The library columns give the Modelica name the parameter is rendered under; a dash means the library does not take it (a value given anyway is ignored with a warning). *numerical* marks simulation settings rather than physical properties. The page is generated from `trano/data_models/parameters.yaml`.

## AhuParameters

Elements: `airhandlingunit`

| Parameter | Description | Default | Buildings | IDEAS | AixLib reduced order | ISO 13790 | mpc |
|---|---|---|---|---|---|---|---|
| `m_flow_nominal` | Nominal mass flow rate [kg/s] | `2*100*1.2/3600` | `m_flow_nominal` | `m_flow_nominal` | `m_flow_nominal` | `m_flow_nominal` | `m_flow_nominal` |
| `dp_nominal` | Nominal pressure raise [Pa] | `200` | `dp_nominal` | `dp_nominal` | `dp_nominal` | `dp_nominal` | `dp_nominal` |
| `heat_exchanger_effectiveness` | Heat exchanger effectiveness [1] | `0.8` | `eps` | `eps` | `eps` | `eps` | `eps` |

## BatteryParameters

Elements: `battery`

| Parameter | Description | Default | Buildings | IDEAS | AixLib reduced order | ISO 13790 | mpc |
|---|---|---|---|---|---|---|---|
| `capacity` | Usable energy capacity [kWh] | `10.0` | - | - | - | - | `capacity` |
| `max_charge_power` | Maximum charging power [W] | `5000.0` | - | - | - | - | `max_charge_power` |
| `max_discharge_power` | Maximum discharging power [W] | `5000.0` | - | - | - | - | `max_discharge_power` |
| `charge_efficiency` | Charging efficiency [1] | `0.95` | - | - | - | - | `charge_efficiency` |
| `discharge_efficiency` | Discharging efficiency [1] | `0.95` | - | - | - | - | `discharge_efficiency` |
| `min_soc` | Lower bound of the state of charge [1] | `0.1` | - | - | - | - | `min_soc` |
| `max_soc` | Upper bound of the state of charge [1] | `0.9` | - | - | - | - | `max_soc` |
| `initial_soc` | Initial state of charge [1] | `0.5` | - | - | - | - | `initial_soc` |

## BoilerControlParameters

Elements: `boilercontrol`

| Parameter | Description | Default | Buildings | IDEAS | AixLib reduced order | ISO 13790 | mpc |
|---|---|---|---|---|---|---|---|
| `threshold_outdoor_air_cutoff` | Outdoor air temperature threshold; output is true when the outdoor air is below this heating cut-off limit [K] | `288.15` | `threshold_outdoor_air_cutoff` | `threshold_outdoor_air_cutoff` | `threshold_outdoor_air_cutoff` | `threshold_outdoor_air_cutoff` | `threshold_outdoor_air_cutoff` |
| `threshold_to_switch_off_boiler` | Tank bottom temperature above which the boiler switches off; defaults to tsup_nominal + 5 K [K] | library default | `threshold_to_switch_off_boiler` | `threshold_to_switch_off_boiler` | `threshold_to_switch_off_boiler` | `threshold_to_switch_off_boiler` | `threshold_to_switch_off_boiler` |
| `tsup_nominal` | Supply temperature set point; the boiler starts when the tank top drops 1 K below it [K] | `353.15` | `TSup_nominal` | `TSup_nominal` | `TSup_nominal` | `TSup_nominal` | `TSup_nominal` |

## BoilerParameters

Elements: `boiler`

| Parameter | Description | Default | Buildings | IDEAS | AixLib reduced order | ISO 13790 | mpc |
|---|---|---|---|---|---|---|---|
| `coefficients_for_efficiency_curve` | Coefficients for efficiency curve | `{0.9}` | `a` | `a` | `a` | `a` | `a` |
| `diff_pressure` | Pressure rise values defining the boiler-circuit pump pressure curve [Pa] | `5000*{2,1}` | `dp` | `dp` | `dp` | `dp` | `dp` |
| `dp_nominal` | Pressure difference [Pa] | `5000.0` | `dp_nominal` | `dp_nominal` | `dp_nominal` | `dp_nominal` | `dp_nominal` |
| `dt_boi_nominal` | Nominal temperature difference for the boiler loop [K] | `20.0` | - | - | - | - | - |
| `dt_rad_nominal` | Nominal temperature difference for the radiator loop [K] | `10.0` | - | - | - | - | - |
| `effcur` | Curve used to compute the efficiency | `Buildings.Fluid.Types.EfficiencyCurves.Constant` | `effCur` | `effCur` | `effCur` | `effCur` | `effCur` |
| `fraction_of_nominal_flow_rate_where_flow_transitions_to_laminar` | *numerical.* Fraction of nominal flow rate where flow transitions to laminar [1] | `0.1` | `deltaM` | `deltaM` | `deltaM` | `deltaM` | `deltaM` |
| `height_of_tank_without_insulation` | Height of tank (without insulation) [m] | `2.0` | `hTan` | `hTan` | `hTan` | `hTan` | `hTan` |
| `if_actual_temperature_at_port_is_computed` | *numerical.* = true, if actual temperature at port is computed | `false` | `show_T` | `show_T` | `show_T` | `show_T` | `show_T` |
| `nominal_heating_power` | Nominal heating power [W] | `20000.0` | `Q_flow_nominal` | `Q_flow_nominal` | `Q_flow_nominal` | `Q_flow_nominal` | `Q_flow_nominal` |
| `nominal_mass_flow_radiator_loop` | Nominal mass flow rate of the radiator loop [kg/s] | computed | `nominal_mass_flow_radiator_loop` | `nominal_mass_flow_radiator_loop` | `nominal_mass_flow_radiator_loop` | `nominal_mass_flow_radiator_loop` | `nominal_mass_flow_radiator_loop` |
| `nominal_mass_flow_rate_boiler` | Nominal mass flow rate of the boiler loop [kg/s] | computed | `nominal_mass_flow_rate_boiler` | `nominal_mass_flow_rate_boiler` | `nominal_mass_flow_rate_boiler` | `nominal_mass_flow_rate_boiler` | `nominal_mass_flow_rate_boiler` |
| `number_of_volume_segments` | *numerical.* Number of volume segments [1] | `4` | `nSeg` | `nSeg` | `nSeg` | `nSeg` | `nSeg` |
| `sca_fac_rad` | Scaling factor to scale the power (and mass flow rate) of the radiator loop [1] | `1.5` | - | - | - | - | - |
| `tank_volume` | Tank volume [m3] | `0.2` | `VTan` | `VTan` | `VTan` | `VTan` | `VTan` |
| `temperature_used_to_compute_nominal_efficiency` | Temperature used to compute nominal efficiency (only used if efficiency curve depends on temperature) [K] | `353.15` | `T_nominal` | `T_nominal` | `T_nominal` | `T_nominal` | `T_nominal` |
| `thickness_of_insulation` | Thickness of insulation [m] | `0.1` | `dIns` | `dIns` | `dIns` | `dIns` | `dIns` |
| `use_linear_relation_between_m_flow_and_dp_for_any_flow_rate` | *numerical.* = true, use linear relation between m_flow and dp for any flow rate | `true` | `linearizeFlowResistance` | `linearizeFlowResistance` | `linearizeFlowResistance` | `linearizeFlowResistance` | `linearizeFlowResistance` |
| `v_flow` | Volume flow rate values defining the boiler-circuit pump pressure curve [m3/s] | computed | `V_flow` | `V_flow` | `V_flow` | `V_flow` | `V_flow` |
| `use_storage_tank` | None | `false` | `useStorageTank` | `useStorageTank` | `useStorageTank` | `useStorageTank` | `useStorageTank` |
| `temperature_heatpump_source` | None | `286.15` | `TSouSet` | `TSouSet` | `TSouSet` | `TSouSet` | `TSouSet` |
| `temperature_supply_setpoint_heat_pump` | None | `323.15` | `TSet` | `TSet` | `TSet` | `TSet` | `TSet` |
| `cop_nominal` | Heat pump COP at the rating point A7/W35, used by the mpc library [1] | `4.5` | - | - | - | - | - |
| `cop_outdoor_slope` | Heat pump COP change per kelvin of outdoor temperature, mpc library [1/K] | `0.11` | - | - | - | - | - |
| `cop_supply_slope` | Heat pump COP change per kelvin of supply temperature, mpc library [1/K] | `-0.075` | - | - | - | - | - |
| `cop_cross_term` | Heat pump COP cross term outdoor x supply deviation, mpc library [1/K2] | `-0.0025` | - | - | - | - | - |
| `max_electrical_power` | Maximum electrical power of the heat pump compressor, mpc library [W] (nominal heating power / COP when absent) | library default | - | - | - | - | - |

## ChillerParameters

Elements: `chiller`

| Parameter | Description | Default | Buildings | IDEAS | AixLib reduced order | ISO 13790 | mpc |
|---|---|---|---|---|---|---|---|
| `nominal_cooling_power` | Nominal cooling power [W] | `10000.0` | - | - | - | - | `nominal_cooling_power` |
| `eer_nominal` | EER at 35 degC outdoor temperature [1] | `3.5` | - | - | - | - | `eer_nominal` |
| `eer_outdoor_slope` | EER change per kelvin of outdoor temperature [1/K] | `-0.06` | - | - | - | - | `eer_outdoor_slope` |
| `max_electrical_power` | Maximum electrical power of the compressor [W] (nominal cooling power / EER when absent) | library default | - | - | - | - | `max_electrical_power` |

## DhwTankParameters

Elements: `dhwtank`

| Parameter | Description | Default | Buildings | IDEAS | AixLib reduced order | ISO 13790 | mpc |
|---|---|---|---|---|---|---|---|
| `volume` | Water volume of the tank [m3] | `0.2` | - | - | - | - | `volume` |
| `set_temperature` | Upper comfort bound of the tank temperature [K] | `328.15` | - | - | - | - | `set_temperature` |
| `min_temperature` | Lower comfort bound of the tank temperature [K] | `318.15` | - | - | - | - | `min_temperature` |
| `loss_coefficient` | Standing loss coefficient of the tank [W/K] | `2.0` | - | - | - | - | `loss_coefficient` |
| `supply_offset` | Heat pump supply temperature above the tank temperature [K] | `5.0` | - | - | - | - | `supply_offset` |
| `daily_draw_off_energy` | Heat drawn by the hot water taps per day [kWh] | `8.0` | - | - | - | - | `daily_draw_off_energy` |
| `zone` | Space receiving the standing losses of the tank (first space when absent) | library default | - | - | - | - | `zone` |

## EmissionControlParameters

Elements: `emissioncontrol`

| Parameter | Description | Default | Buildings | IDEAS | AixLib reduced order | ISO 13790 | mpc |
|---|---|---|---|---|---|---|---|
| `schedule` | Schedule of switching times that toggles the heating set point between the occupied set point and the setback value | `3600*{7, 19}` | `schedule` | `schedule` | `schedule` | `schedule` | `schedule` |
| `temperature_heating_setpoint` | Room air temperature heating set point during occupied period [K] | `297.0` | `THeaSet` | `THeaSet` | `THeaSet` | `THeaSet` | `THeaSet` |
| `temperature_heating_setback` | Room air temperature heating set point during the setback period (e.g. at night) [K] | `289.0` | `THeaSetBack` | `THeaSetBack` | `THeaSetBack` | `THeaSetBack` | `THeaSetBack` |
| `controller_gain` | Gain of controller [1] | `5.0` | `k` | `k` | `k` | `k` | `k` |
| `data` | Occupancy data sources | library default | `data` | `data` | `data` | `data` | `data` |

## EvChargerParameters

Elements: `evcharger`

| Parameter | Description | Default | Buildings | IDEAS | AixLib reduced order | ISO 13790 | mpc |
|---|---|---|---|---|---|---|---|
| `battery_capacity` | Battery capacity of the vehicle [kWh] | `60.0` | - | - | - | - | `battery_capacity` |
| `max_charge_power` | Maximum charging power [W] | `7400.0` | - | - | - | - | `max_charge_power` |
| `charge_efficiency` | Charging efficiency [1] | `0.9` | - | - | - | - | `charge_efficiency` |
| `initial_soc` | Initial state of charge [1] | `0.5` | - | - | - | - | `initial_soc` |
| `arrival_hour` | Hour of the day the vehicle plugs in [h] | `18.0` | - | - | - | - | `arrival_hour` |
| `departure_hour` | Hour of the day the vehicle leaves [h] | `7.0` | - | - | - | - | `departure_hour` |
| `energy_per_day` | Energy consumed by driving per day [kWh] | `10.0` | - | - | - | - | `energy_per_day` |
| `target_soc` | State of charge required at departure [1] | `0.8` | - | - | - | - | `target_soc` |
| `weekend_present` | The vehicle stays plugged in during the weekend | `true` | - | - | - | - | `weekend_present` |

## IdealHeatingCoolingParameter

Elements: `idealheatingcooling`

| Parameter | Description | Default | Buildings | IDEAS | AixLib reduced order | ISO 13790 | mpc |
|---|---|---|---|---|---|---|---|
| `controller_gain` | Gain of the heating and cooling PI controllers, output fraction of the capacity per kelvin of error [1/K] | `0.1` | `k` | `k` | `k` | `k` | `k` |
| `controller_integral_time` | Integral time of the heating and cooling PI controllers [s] | `300.0` | `Ti` | `Ti` | `Ti` | `Ti` | `Ti` |
| `cooling_setpoint_schedule` | Day schedule of the cooling set point, rows of time since midnight [s] and set point [K], repeated every day; one row for a constant set point | `[0, 300.15]` | `TSetCoo` | `TSetCoo` | `TSetCoo` | `TSetCoo` | `TSetCoo` |
| `heating_setpoint_schedule` | Day schedule of the heating set point, rows of time since midnight [s] and set point [K], repeated every day; one row for a constant set point | `[0, 293.15]` | `TSetHea` | `TSetHea` | `TSetHea` | `TSetHea` | `TSetHea` |
| `maximum_cooling_power` | Cooling capacity; zero switches cooling off [W] | `1000000.0` | `QCoo_flow_max` | `QCoo_flow_max` | `QCoo_flow_max` | `QCoo_flow_max` | `QCoo_flow_max` |
| `maximum_heating_power` | Heating capacity; zero switches heating off [W] | `1000000.0` | `QHea_flow_max` | `QHea_flow_max` | `QHea_flow_max` | `QHea_flow_max` | `QHea_flow_max` |
| `radiative_fraction` | Fraction of the heat flow exchanged with the radiative temperature of the zone, the rest with its air [1] | `0.0` | `frad` | `frad` | `frad` | `frad` | `frad` |

## OccupancyParameters

Elements: `occupancy`

| Parameter | Description | Default | Buildings | IDEAS | AixLib reduced order | ISO 13790 | mpc |
|---|---|---|---|---|---|---|---|
| `gain` | Gain to convert from occupancy (per person) to radiant, convective and latent heat in [W/m2] | `[35; 70; 30]` | `gain` | `gain` | `gain` | `gain` | `gain` |
| `heat_gain_if_occupied`, `occupant_density` | Heat gain if occupied | `1/6/4` | `k` | `k` | `k` | `k` | `k` |
| `occupancy` | Occupancy table, each entry switching occupancy on or off | `3600*{7, 19}` | `occupancy` | `occupancy` | `occupancy` | `occupancy` | `occupancy` |
| `ach` | *deprecated: the infiltration of a zone is the `ach` parameter of its space.* Infiltration [1/h] | `0.9` | `ACH` | `ACH` | `ACH` | `ACH` | `ACH` |
| `floor_area` | Floor area [m2] | library default | `AFlo` (co2 variant) | `AFlo` (co2 variant) | - | - | - |
| `data` | Occupancy data sources | library default | `data` | `data` | `data` | `data` | `data` |
| `sensible_heat_per_person` | Sensible heat released per occupant [W]; with the latent heat and the radiant fraction it gives the `gain` matrix when that one is absent | library default | taken another way | taken another way | taken another way | taken another way | taken another way |
| `latent_heat_per_person` | Latent heat released per occupant [W] | library default | taken another way | taken another way | taken another way | taken another way | taken another way |
| `radiant_fraction` | Radiant share of the sensible heat of the occupants [1] | library default | taken another way | taken another way | taken another way | taken another way | taken another way |
| `lighting_power_density` | Heat released by the lighting per floor area, all day [W/m2] | library default | `lightingPower` | - | taken another way | `lightingPower` | - |
| `lighting_radiant_fraction` | Radiant share of the lighting heat [1], 0.4 by default | library default | `lightingRadiantFraction` | - | taken another way | `lightingRadiantFraction` | - |
| `equipment_power_density` | Heat released by the equipment per floor area, all day [W/m2] | library default | `equipmentPower` | - | taken another way | `equipmentPower` | - |
| `equipment_radiant_fraction` | Radiant share of the equipment heat [1], 0.4 by default | library default | `equipmentRadiantFraction` | - | taken another way | `equipmentRadiantFraction` | - |
| `activity_degree` | Activity level of the occupants of the reduced-order zone [met], 1.2 by default | library default | - | - | taken another way | - | - |
| `co2_generation_per_person` | CO2 released per occupant [m3/s], 5.2e-6 by default (CO2 variant) | library default | `gCO2` (co2 variant) | `gCO2` (co2 variant) | - | - | - |
| `outdoor_co2_concentration` | CO2 concentration of the outdoor air [ppm], 420 by default (CO2 variant) | library default | `ppmOut` (co2 variant) | `ppmOut` (co2 variant) | - | - | - |

## PIDParameters

Elements: `threewayvalvecontrol`, `collectorcontrol`

| Parameter | Description | Default | Buildings | IDEAS | AixLib reduced order | ISO 13790 | mpc |
|---|---|---|---|---|---|---|---|
| `controller_type` | Type of controller | `Buildings.Controls.OBC.CDL.Types.SimpleController.P` | `controllerType` | `controllerType` | `controllerType` | `controllerType` | `controllerType` |
| `k` | Gain of controller [1] | `1.0` | `k` | `k` | `k` | `k` | `k` |
| `nd` | The higher Nd, the more ideal the derivative block [1] | `10.0` | `Nd` | `Nd` | `Nd` | `Nd` | `Nd` |
| `ni` | Ni*Ti is time constant of anti-windup compensation [1] | `0.9` | `Ni` | `Ni` | `Ni` | `Ni` | `Ni` |
| `r` | Typical range of control error, used for scaling the control error | `1.0` | `r` | `r` | `r` | `r` | `r` |
| `td` | Time constant of derivative block [s] | `0.1` | `Td` | `Td` | `Td` | `Td` | `Td` |
| `ti` | Time constant of integrator block [s] | `0.5` | `Ti` | `Ti` | `Ti` | `Ti` | `Ti` |
| `y_max` | Upper limit of output [1] | `1.0` | `yMax` | `yMax` | `yMax` | `yMax` | `yMax` |
| `y_min` | Lower limit of output [1] | `0.0` | `yMin` | `yMin` | `yMin` | `yMin` | `yMin` |

## PhotovoltaicParameters

Elements: `photovoltaic`

| Parameter | Description | Default | Buildings | IDEAS | AixLib reduced order | ISO 13790 | mpc |
|---|---|---|---|---|---|---|---|
| `area` | Module area [m2] | `20.0` | `area` | `area` | `area` | `area` | `area` |
| `efficiency` | Module and inverter efficiency [1] | `0.18` | `efficiency` | `efficiency` | `efficiency` | `efficiency` | `efficiency` |
| `azimuth` | Surface azimuth [deg], 0 for south, 90 for west | `0.0` | `azimuth` | `azimuth` | `azimuth` | `azimuth` | `azimuth` |
| `tilt` | Surface tilt [deg], 0 for a roof, 90 for a wall | `35.0` | `tilt` | `tilt` | `tilt` | `tilt` | `tilt` |

## PumpParameters

Elements: `pump`

| Parameter | Description | Default | Buildings | IDEAS | AixLib reduced order | ISO 13790 | mpc |
|---|---|---|---|---|---|---|---|
| `constant_input_set_point` | *deprecated: it never reached the pump model and has no effect.* Constant input set point | library default | - | - | - | - | - |
| `dp_nominal` | Nominal pressure raise [Pa] | `30000.0` | `dp_nominal` | `dp_nominal` | `dp_nominal` | `dp_nominal` | `dp_nominal` |
| `m_flow_nominal` | Nominal mass flow rate [kg/s] | `0.15` | `m_flow_nominal` | `m_flow_nominal` | `m_flow_nominal` | `m_flow_nominal` | `m_flow_nominal` |

## RadiatorParameter

Elements: `radiator`

| Parameter | Description | Default | Buildings | IDEAS | AixLib reduced order | ISO 13790 | mpc |
|---|---|---|---|---|---|---|---|
| `air_temperature_at_nominal_condition` | Air temperature at nominal condition [K] | `293.15` | `TAir_nominal` | `TAir_nominal` | `TAir_nominal` | `TAir_nominal` | `TAir_nominal` |
| `dp_nominal` | Pressure drop at nominal mass flow rate [Pa] | `2000.0` | `dp_nominal` | `dp_nominal` | `dp_nominal` | `dp_nominal` | `dp_nominal` |
| `dry_mass_of_radiator_that_will_be_lumped_to_water_heat_capacity` | Dry mass of radiator that will be lumped to water heat capacity [kg] | computed | `mDry` | `mDry` | `mDry` | `mDry` | `mDry` |
| `exponent_for_heat_transfer` | Exponent for heat transfer | `1.24` | `n` | `n` | `n` | `n` | `n` |
| `fraction_of_nominal_mass_flow_rate_where_transition_to_turbulent_occurs` | *numerical.* Fraction of nominal mass flow rate where transition to turbulent occurs [1] | `0.01` | `deltaM` | `deltaM` | `deltaM` | `deltaM` | `deltaM` |
| `fraction_radiant_heat_transfer` | Fraction radiant heat transfer [1] | `0.3` | `fraRad` | `fraRad` | `fraRad` | `fraRad` | `fraRad` |
| `nominal_heating_power_positive_for_heating` | Nominal heating power (positive for heating) [W] | `5000.0` | `Q_flow_nominal` | `Q_flow_nominal` | `Q_flow_nominal` | `Q_flow_nominal` | `Q_flow_nominal` |
| `number_of_elements_used_in_the_discretization` | *numerical.* Number of elements used in the discretization [1] | `1` | `nEle` | `nEle` | `nEle` | `nEle` | `nEle` |
| `radiative_temperature_at_nominal_condition` | Radiative temperature at nominal condition [K] | `293.15` | `TRad_nominal` | `TRad_nominal` | `TRad_nominal` | `TRad_nominal` | `TRad_nominal` |
| `use_linear_relation_between_m_flow_and_dp_for_any_flow_rate` | *numerical.* = true, use linear relation between m_flow and dp for any flow rate | `true` | `linearized` | `linearized` | `linearized` | `linearized` | `linearized` |
| `use_m_flow_f_dp_else_dp_f_m_flow` | *numerical.* = true, use m_flow = f(dp) else dp = f(m_flow) | `false` | `from_dp` | `from_dp` | `from_dp` | `from_dp` | `from_dp` |
| `water_inlet_temperature_at_nominal_condition` | Water inlet temperature at nominal condition [K] | `353.15` | `T_a_nominal` | `T_a_nominal` | `T_a_nominal` | `T_a_nominal` | `T_a_nominal` |
| `water_outlet_temperature_at_nominal_condition` | Water outlet temperature at nominal condition [K] | `333.15` | `T_b_nominal` | `T_b_nominal` | `T_b_nominal` | `T_b_nominal` | `T_b_nominal` |
| `water_volume_of_radiator` | Water volume of radiator [m3] | computed | `VWat` | `VWat` | `VWat` | `VWat` | `VWat` |

## SensorParameters

Elements: `temperaturesensor`, `heatmetersensor`

| Parameter | Description | Default | Buildings | IDEAS | AixLib reduced order | ISO 13790 | mpc |
|---|---|---|---|---|---|---|---|
| `nominal_mass_flow_rate` | Nominal mass flow rate through the sensor, used for its dynamics and its small-flow regularization [kg/s] | `0.15` | `m_flow_nominal` | `m_flow_nominal` | `m_flow_nominal` | `m_flow_nominal` | `m_flow_nominal` |

## SpaceParameter

Elements: `space`

| Parameter | Description | Default | Buildings | IDEAS | AixLib reduced order | ISO 13790 | mpc |
|---|---|---|---|---|---|---|---|
| `average_room_height` | Average room height [m] | `2.0` | `hRoo` | `hZone` | - | - | `hRoo` |
| `floor_area` | Floor area [m2] | `20.0` | `AFlo` | taken another way | `AZone` | `AFlo` | `AFlo` |
| `linearize_emissive_power` | *numerical.* Set to true to linearize emissive power | `true` | `linearizeRadiation` | - | - | - | `linearizeRadiation` |
| `nominal_mass_flow_rate` | Nominal mass flow rate [kg/s] | `0.01` | `m_flow_nominal` | `m_flow_nominal` (when given) | - | - | `m_flow_nominal` |
| `sensible_thermal_mass_scaling_factor` | Factor for scaling the sensible thermal mass of the zone air volume [1] | `1.0` | `mSenFac` | `mSenFac` | `mSenFac` (when given) | - | `mSenFac` |
| `ach` | Infiltration [1/h] | library default | `ACH` (infiltration variant) | taken another way | taken another way | taken another way | `ACH` |
| `ventilation_schedule` | Day schedule of outdoor air brought into the zone on top of the infiltration, rows of time since midnight [s] and mass flow rate [kg/s], repeated every day (Buildings infiltration variant) | library default | `ventilationSchedule` (infiltration variant) | taken another way | - | - | `ventilationSchedule` |
| `temperature_initial` | Initial temperature [K] | `294.15` | `T_start` | `T_start` | `T_start` (when given) | - | `T_start` |
| `volume` | Air volume of the zone [m3] | computed | - | `V` | `VAir` | `VRoo` | `volume` |
| `n50` | Air change rate at a 50 Pa pressure difference, the airtightness of the zone [1/h]; gives the infiltration `ach` as n50 / n50_to_ach when `ach` is absent (IDEAS takes it directly) | library default | taken another way | taken another way | taken another way | taken another way | `n50` |
| `n50_to_ach` | Ratio between the air change rate at 50 Pa and the infiltration rate [1] | `20.0` | taken another way | `n50toAch` (when given) | taken another way | taken another way | `n50toAch` |
| `interior_convection_coefficient` | Fixed convective heat transfer coefficient of the inside surfaces [W/(m2.K)]; Buildings and the ISO 13790 zone switch to a fixed coefficient (3 and 3.45 by default), the reduced-order zone applies it to its walls, windows, floor and roof (2.7 by default) | library default | `hIntFixed` | - | taken another way | `hInt` | `hIntFixed` |
| `exterior_convection_coefficient` | Fixed convective heat transfer coefficient of the outside surfaces [W/(m2.K)]; Buildings switches to a fixed coefficient (10 by default), the reduced-order zone applies it to its walls, windows and roof (20 by default) | library default | `hExtFixed` | - | taken another way | - | `hExtFixed` |
| `thermal_mass_class` | Building mass class of the ISO 13790 zone, light, medium or heavy; derived from the heat capacity of the constructions when absent | library default | - | - | - | taken another way | `thermal_mass_class` |
| `ground_heat_transfer_factor` | Adjustment factor of the heat transfer through the floor to the ground of the ISO 13790 zone [1], 0.5 by default | library default | - | - | - | `b` | `b` |
| `shading_reduction_factor` | Factor on the solar gains through the windows for external shading [1]; the ISO 13790 zone applies it always (1 by default), the reduced-order zone when the irradiance exceeds the sunblind threshold (0.7 by default) | library default | - | - | taken another way | `shaRedFac` | `shaRedFac` |
| `sunblind_irradiance_threshold` | Irradiance on a window above which the reduced-order zone applies the shading reduction factor [W/m2], 100 by default | library default | - | - | taken another way | - | `maxIrr` |

## SplitValveParameters

Elements: `splitvalve`

| Parameter | Description | Default | Buildings | IDEAS | AixLib reduced order | ISO 13790 | mpc |
|---|---|---|---|---|---|---|---|
| `dp_nominal` | Pressure drop at nominal mass flow rate, set to zero or negative number at outflowing ports [Pa] | `{5000,-1,-1}` | `dp_nominal` | `dp_nominal` | `dp_nominal` | `dp_nominal` | `dp_nominal` |
| `fraction_of_nominal_mass_flow_rate_where_transition_to_turbulent_occurs` | *numerical.* Fraction of nominal mass flow rate where transition to turbulent occurs [1] | `0.3` | `deltaM` | `deltaM` | `deltaM` | `deltaM` | `deltaM` |
| `m_flow_nominal` | Mass flow rate; set negative at outflowing ports [kg/s] | `0.15*{1,-1,-1}` | `m_flow_nominal` | `m_flow_nominal` | `m_flow_nominal` | `m_flow_nominal` | `m_flow_nominal` |
| `nominal_mass_flow_rate_for_dynamic_momentum_and_energy_balance` | *numerical.* Nominal mass flow rate for dynamic momentum and energy balance [kg/s] | library default | `mDyn_flow_nominal` | `mDyn_flow_nominal` | `mDyn_flow_nominal` | `mDyn_flow_nominal` | `mDyn_flow_nominal` |
| `time_constant_at_nominal_flow_for_dynamic_energy_and_momentum_balance` | *numerical.* Time constant at nominal flow for dynamic energy and momentum balance [s] | library default | `tau` | `tau` | `tau` | `tau` | `tau` |
| `use_linear_relation_between_m_flow_and_dp_for_any_flow_rate` | *numerical.* = true, use linear relation between m_flow and dp for any flow rate | `true` | `linearized` | `linearized` | `linearized` | `linearized` | `linearized` |

## ThreeWayValveParameters

Elements: `threewayvalve`

| Parameter | Description | Default | Buildings | IDEAS | AixLib reduced order | ISO 13790 | mpc |
|---|---|---|---|---|---|---|---|
| `Av` | Av (metric) flow coefficient [m2] | library default | `Av` | `Av` | `Av` | `Av` | `Av` |
| `Cv` | Cv (US) flow coefficient [USG/min/(psi)^(1/2)] | library default | `Cv` | `Cv` | `Cv` | `Cv` | `Cv` |
| `Kv` | Kv (metric) flow coefficient [m3/h/(bar)^(1/2)] | library default | `Kv` | `Kv` | `Kv` | `Kv` | `Kv` |
| `dp_fixed_nominal` | Nominal pressure drop of pipes and other equipment in flow legs at port_1 and port_3 [Pa] | `{2000,0}` | `dpFixed_nominal` | `dpFixed_nominal` | `dpFixed_nominal` | `dpFixed_nominal` | `dpFixed_nominal` |
| `dp_valve_nominal` | Nominal pressure drop of fully open valve, used if CvData=Buildings.Fluid.Types.CvTypes.OpPoint [Pa] | `6000.0` | `dpValve_nominal` | `dpValve_nominal` | `dpValve_nominal` | `dpValve_nominal` | `dpValve_nominal` |
| `fra_k` | Fraction Kv(port_3->port_2)/Kv(port_1->port_2) [1] | `0.7` | `fraK` | `fraK` | `fraK` | `fraK` | `fraK` |
| `fraction_of_nominal_flow_rate_where_linearization_starts_if_y_1` | *numerical.* Fraction of nominal flow rate where linearization starts, if y=1 [1] | `0.02` | `deltaM` | `deltaM` | `deltaM` | `deltaM` | `deltaM` |
| `m_flow_nominal` | Nominal mass flow rate [kg/s] | `0.15` | `m_flow_nominal` | `m_flow_nominal` | `m_flow_nominal` | `m_flow_nominal` | `m_flow_nominal` |
| `range_of_significant_deviation_from_equal_percentage_law` | Range of significant deviation from equal percentage law [1] | `0.01` | `delta0` | `delta0` | `delta0` | `delta0` | `delta0` |
| `rangeability` | Rangeability, R=50...100 typically [1] | `50.0` | `R` | `R` | `R` | `R` | `R` |
| `rho_std` | Inlet density for which valve coefficients are defined [kg/m3] | library default | `rhoStd` | `rhoStd` | `rhoStd` | `rhoStd` | `rhoStd` |
| `use_linear_relation_between_m_flow_and_dp_for_any_flow_rate` | *numerical.* = true, use linear relation between m_flow and dp for any flow rate | `{true, true}` | `linearized` | `linearized` | `linearized` | `linearized` | `linearized` |
| `valve_leakage` | Valve leakage, l=Kv(y=0)/Kv(y=1) [1] | `{0.01,0.01}` | `l` | `l` | `l` | `l` | `l` |

## ValveParameters

Elements: `valve`

| Parameter | Description | Default | Buildings | IDEAS | AixLib reduced order | ISO 13790 | mpc |
|---|---|---|---|---|---|---|---|
| `av` | Av (metric) flow coefficient [m2] | library default | `Av` | `Av` | `Av` | `Av` | `Av` |
| `cv` | Cv (US) flow coefficient [USG/min/(psi)^(1/2)] | library default | `Cv` | `Cv` | `Cv` | `Cv` | `Cv` |
| `dp_fixed_nominal` | Pressure drop of pipe and other resistances that are in series [Pa] | `5000.0` | `dpFixed_nominal` | `dpFixed_nominal` | `dpFixed_nominal` | `dpFixed_nominal` | `dpFixed_nominal` |
| `dp_valve_nominal` | Nominal pressure drop of fully open valve, used if CvData=Buildings.Fluid.Types.CvTypes.OpPoint [Pa] | `10000.0` | `dpValve_nominal` | `dpValve_nominal` | `dpValve_nominal` | `dpValve_nominal` | `dpValve_nominal` |
| `fraction_of_nominal_flow_rate_where_linearization_starts_if_y_1` | *numerical.* Fraction of nominal flow rate where linearization starts, if y=1 [1] | `0.02` | `deltaM` | `deltaM` | `deltaM` | `deltaM` | `deltaM` |
| `k_fixed` | Flow coefficient of fixed resistance that may be in series with valve, k=m_flow/sqrt(dp), with unit=(kg.m)^(1/2) | library default | `kFixed` | `kFixed` | `kFixed` | `kFixed` | `kFixed` |
| `kv` | Kv (metric) flow coefficient [m3/h/(bar)^(1/2)] | library default | `Kv` | `Kv` | `Kv` | `Kv` | `Kv` |
| `m_flow_nominal` | Nominal mass flow rate [kg/s] | `0.06` | `m_flow_nominal` | `m_flow_nominal` | `m_flow_nominal` | `m_flow_nominal` | `m_flow_nominal` |
| `range_of_significant_deviation_from_equal_percentage_law` | Range of significant deviation from equal percentage law [1] | `0.01` | `delta0` | `delta0` | `delta0` | `delta0` | `delta0` |
| `rangeability` | Rangeability, R=50...100 typically [1] | `50.0` | `R` | `R` | `R` | `R` | `R` |
| `use_linear_relation_between_m_flow_and_dp_for_any_flow_rate` | *numerical.* = true, use linear relation between m_flow and dp for any flow rate | `true` | `linearized` | `linearized` | `linearized` | `linearized` | `linearized` |
| `use_m_flow_f_dp_else_dp_f_m_flow` | *numerical.* = true, use m_flow = f(dp) else dp = f(m_flow) | `true` | `from_dp` | `from_dp` | `from_dp` | `from_dp` | `from_dp` |
| `valve_leakage` | Valve leakage, l=Kv(y=0)/Kv(y=1) [1] | `0.0001` | `l` | `l` | `l` | `l` | `l` |

## WeatherParameters

Elements: `weather`

| Parameter | Description | Default | Buildings | IDEAS | AixLib reduced order | ISO 13790 | mpc |
|---|---|---|---|---|---|---|---|
| `path` | Name of weather data file | library default | `filNam` | `filNam` | `filNam` | `filNam` | `filNam` |
| `atmospheric_pressure_source` | Source of the atmospheric pressure, e.g. Buildings.BoundaryConditions.Types.DataSource.File to read it from the weather file; the library default (a constant 101325 Pa) when absent | library default | `pAtmSou` | - | `pAtmSou` | `pAtmSou` | `pAtmSou` |
| `atmospheric_pressure` | Atmospheric pressure [Pa] when it is not read from the weather file, 101325 by default | library default | `pAtm` | - | `pAtm` | `pAtm` | `pAtm` |
| `outdoor_co2_concentration` | CO2 concentration of the outdoor air of the IDEAS simulation manager [ppm], 400 by default | library default | - | `ppmCO2` | - | - | - |
| `building_height` | Height of the building [m] for the wind speed profile of the IDEAS simulation manager, 10 by default | library default | - | `H` | - | - | - |
| `default_n50` | Air change rate at 50 Pa of the IDEAS zones that do not give their own [1/h], 3 by default | library default | - | `n50` | - | - | - |
