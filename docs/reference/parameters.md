
## BoilerControlParameters
The following parameters are valid for the following classes boilercontrol
```yaml
threshold_outdoor_air_cutoff:
  alias: threshold_outdoor_air_cutoff
  description: Outdoor air temperature threshold; output is true when the outdoor
    air is below this heating cut-off limit [K]
  ifabsent: float(288.15)
  range: float
threshold_to_switch_off_boiler:
  alias: threshold_to_switch_off_boiler
  description: Tank bottom temperature above which the boiler switches off; defaults
    to tsup_nominal + 5 K [K]
  range: float
tsup_nominal:
  alias: TSup_nominal
  description: Supply temperature set point; the boiler starts when the tank top drops
    1 K below it [K]
  ifabsent: float(353.15)
  range: float

```



## BoilerParameters
The following parameters are valid for the following classes boiler
```yaml
coefficients_for_efficiency_curve:
  alias: a
  description: Coefficients for efficiency curve
  ifabsent: string({0.9})
  range: string
cop_cross_term:
  description: Heat pump COP cross term outdoor x supply deviation, mpc library [1/K2]
  ifabsent: float(-0.0025)
  range: float
cop_nominal:
  description: Heat pump COP at the rating point A7/W35, used by the mpc library [1]
  ifabsent: float(4.5)
  range: float
cop_outdoor_slope:
  description: Heat pump COP change per kelvin of outdoor temperature, mpc library
    [1/K]
  ifabsent: float(0.11)
  range: float
cop_supply_slope:
  description: Heat pump COP change per kelvin of supply temperature, mpc library
    [1/K]
  ifabsent: float(-0.075)
  range: float
diff_pressure:
  alias: dp
  description: Pressure rise values defining the boiler-circuit pump pressure curve
    [Pa]
  ifabsent: string(5000*{2,1})
  range: string
dp_nominal:
  alias: dp_nominal
  description: Pressure difference [Pa]
  ifabsent: float(5000)
  range: float
dt_boi_nominal:
  alias: dTBoi_nominal
  description: Nominal temperature difference for the boiler loop [K]
  ifabsent: float(20)
  range: float
dt_rad_nominal:
  alias: dTRad_nominal
  description: Nominal temperature difference for the radiator loop [K]
  ifabsent: float(10)
  range: float
effcur:
  alias: effCur
  description: Curve used to compute the efficiency
  ifabsent: string(Buildings.Fluid.Types.EfficiencyCurves.Constant)
  range: string
fraction_of_nominal_flow_rate_where_flow_transitions_to_laminar:
  alias: deltaM
  description: Fraction of nominal flow rate where flow transitions to laminar [1]
  ifabsent: float(0.1)
  range: float
height_of_tank_without_insulation:
  alias: hTan
  description: Height of tank (without insulation) [m]
  ifabsent: float(2)
  range: float
if_actual_temperature_at_port_is_computed:
  alias: show_T
  description: = true, if actual temperature at port is computed
  ifabsent: string(false)
  range: string
max_electrical_power:
  description: Maximum electrical power of the heat pump compressor, mpc library [W]
    (nominal heating power / COP when absent)
  ifabsent: float(None)
  range: float
nominal_heating_power:
  alias: Q_flow_nominal
  description: Nominal heating power [W]
  ifabsent: float(20000)
  range: float
nominal_mass_flow_radiator_loop:
  alias: null
  description: Nominal mass flow rate of the radiator loop [kg/s]
  func: 'lambda self: self.sca_fac_rad * self.nominal_heating_power / self.dt_rad_nominal
    / 4200'
  type: float
nominal_mass_flow_rate_boiler:
  alias: null
  description: Nominal mass flow rate of the boiler loop [kg/s]
  func: 'lambda self: self.sca_fac_rad * self.nominal_heating_power / self.dt_boi_nominal
    / 4200'
  type: float
number_of_volume_segments:
  alias: nSeg
  description: Number of volume segments [1]
  ifabsent: int(4)
  range: integer
sca_fac_rad:
  alias: scaFacRad
  description: Scaling factor to scale the power (and mass flow rate) of the radiator
    loop [1]
  ifabsent: float(1.5)
  range: float
tank_volume:
  alias: VTan
  description: Tank volume [m3]
  ifabsent: float(0.2)
  range: float
temperature_heatpump_source:
  alias: TSouSet
  description: None
  ifabsent: float(286.15)
  range: float
temperature_supply_setpoint_heat_pump:
  alias: TSet
  description: None
  ifabsent: float(323.15)
  range: float
temperature_used_to_compute_nominal_efficiency:
  alias: T_nominal
  description: Temperature used to compute nominal efficiency (only used if efficiency
    curve depends on temperature) [K]
  ifabsent: float(353.15)
  range: float
thickness_of_insulation:
  alias: dIns
  description: Thickness of insulation [m]
  ifabsent: float(0.1)
  range: float
use_linear_relation_between_m_flow_and_dp_for_any_flow_rate:
  alias: linearizeFlowResistance
  description: = true, use linear relation between m_flow and dp for any flow rate
  ifabsent: string(true)
  range: string
use_storage_tank:
  alias: useStorageTank
  description: None
  ifabsent: string(false)
  range: string
v_flow:
  alias: V_flow
  description: Volume flow rate values defining the boiler-circuit pump pressure curve
    [m3/s]
  func: 'lambda self: f''{self.nominal_mass_flow_rate_boiler}'' ''/1000*{0.5,1}'''
  type: str

```



## EmissionControlParameters
The following parameters are valid for the following classes emissioncontrol
```yaml
controller_gain:
  alias: k
  description: Gain of controller [1]
  ifabsent: float(5)
  range: float
data:
  alias: data
  description: Occupancy data sources
  inlined: true
  inlined_as_list: true
  multivalued: true
  range: DataSource
schedule:
  alias: schedule
  description: Schedule of switching times that toggles the heating set point between
    the occupied set point and the setback value
  ifabsent: string(3600*{7, 19})
  range: string
temperature_heating_setback:
  alias: THeaSetBack
  description: Room air temperature heating set point during the setback period (e.g.
    at night) [K]
  ifabsent: float(289)
  range: float
temperature_heating_setpoint:
  alias: THeaSet
  description: Room air temperature heating set point during occupied period [K]
  ifabsent: float(297)
  range: float

```



## OccupancyParameters
The following parameters are valid for the following classes occupancy
```yaml
ach:
  alias: ACH
  description: Infiltration [1/h]
  ifabsent: float(0.9)
  range: float
data:
  alias: data
  description: Occupancy data sources
  inlined: true
  inlined_as_list: true
  multivalued: true
  range: DataSource
floor_area:
  alias: AFlo
  description: Floor area [m2]
  range: float
gain:
  alias: gain
  description: Gain to convert from occupancy (per person) to radiant, convective
    and latent heat in [W/m2]
  ifabsent: string([35; 70; 30])
  range: string
heat_gain_if_occupied:
  alias: k
  description: Heat gain if occupied
  ifabsent: string(1/6/4)
  range: string
occupancy:
  alias: occupancy
  description: Occupancy table, each entry switching occupancy on or off
  ifabsent: string(3600*{7, 19})
  range: string

```



## PIDParameters
The following parameters are valid for the following classes threewayvalvecontrol,collectorcontrol
```yaml
controller_type:
  alias: controllerType
  description: Type of controller
  ifabsent: string(Buildings.Controls.OBC.CDL.Types.SimpleController.P)
  range: string
k:
  alias: k
  description: Gain of controller [1]
  ifabsent: float(1)
  range: float
nd:
  alias: Nd
  description: The higher Nd, the more ideal the derivative block [1]
  ifabsent: float(10)
  range: float
ni:
  alias: Ni
  description: Ni*Ti is time constant of anti-windup compensation [1]
  ifabsent: float(0.9)
  range: float
r:
  alias: r
  description: Typical range of control error, used for scaling the control error
  ifabsent: float(1)
  range: float
td:
  alias: Td
  description: Time constant of derivative block [s]
  ifabsent: float(0.1)
  range: float
ti:
  alias: Ti
  description: Time constant of integrator block [s]
  ifabsent: float(0.5)
  range: float
y_max:
  alias: yMax
  description: Upper limit of output [1]
  ifabsent: float(1)
  range: float
y_min:
  alias: yMin
  description: Lower limit of output [1]
  ifabsent: float(0)
  range: float

```



## PumpParameters
The following parameters are valid for the following classes pump
```yaml
constant_input_set_point:
  alias: constInput
  description: Constant input set point
  range: float
dp_nominal:
  alias: dp_nominal
  description: Nominal pressure raise [Pa]
  ifabsent: float(30000)
  range: float
m_flow_nominal:
  alias: m_flow_nominal
  description: Nominal mass flow rate [kg/s]
  ifabsent: float(0.15)
  range: float

```



## RadiatorParameter
The following parameters are valid for the following classes radiator
```yaml
air_temperature_at_nominal_condition:
  alias: TAir_nominal
  description: Air temperature at nominal condition [K]
  ifabsent: float(293.15)
  range: float
dp_nominal:
  alias: dp_nominal
  description: Pressure drop at nominal mass flow rate [Pa]
  ifabsent: float(2000)
  range: float
dry_mass_of_radiator_that_will_be_lumped_to_water_heat_capacity:
  alias: mDry
  description: Dry mass of radiator that will be lumped to water heat capacity [kg]
  func: 'lambda self: 0.0263 * abs(self.nominal_heating_power_positive_for_heating)'
  type: float
exponent_for_heat_transfer:
  alias: n
  description: Exponent for heat transfer
  ifabsent: float(1.24)
  range: float
fraction_of_nominal_mass_flow_rate_where_transition_to_turbulent_occurs:
  alias: deltaM
  description: Fraction of nominal mass flow rate where transition to turbulent occurs
    [1]
  ifabsent: float(0.01)
  range: float
fraction_radiant_heat_transfer:
  alias: fraRad
  description: Fraction radiant heat transfer [1]
  ifabsent: float(0.3)
  range: float
nominal_heating_power_positive_for_heating:
  alias: Q_flow_nominal
  description: Nominal heating power (positive for heating) [W]
  ifabsent: float(5000)
  range: float
number_of_elements_used_in_the_discretization:
  alias: nEle
  description: Number of elements used in the discretization [1]
  ifabsent: int(1)
  range: integer
radiative_temperature_at_nominal_condition:
  alias: TRad_nominal
  description: Radiative temperature at nominal condition [K]
  ifabsent: float(293.15)
  range: float
use_linear_relation_between_m_flow_and_dp_for_any_flow_rate:
  alias: linearized
  description: = true, use linear relation between m_flow and dp for any flow rate
  ifabsent: string(true)
  range: string
use_m_flow_f_dp_else_dp_f_m_flow:
  alias: from_dp
  description: = true, use m_flow = f(dp) else dp = f(m_flow)
  ifabsent: string(false)
  range: string
water_inlet_temperature_at_nominal_condition:
  alias: T_a_nominal
  description: Water inlet temperature at nominal condition [K]
  ifabsent: float(353.15)
  range: float
water_outlet_temperature_at_nominal_condition:
  alias: T_b_nominal
  description: Water outlet temperature at nominal condition [K]
  ifabsent: float(333.15)
  range: float
water_volume_of_radiator:
  alias: VWat
  description: Water volume of radiator [m3]
  func: lambda self:5.8e-6 * abs(self.nominal_heating_power_positive_for_heating)
  type: float

```



## SpaceParameter
The following parameters are valid for the following classes space
```yaml
ach:
  alias: ACH
  description: Infiltration [1/h]
  range: float
average_room_height:
  alias: hRoo
  description: Average room height [m]
  ifabsent: float(2)
  range: float
floor_area:
  alias: AFlo
  description: Floor area [m2]
  ifabsent: float(20)
  range: float
linearize_emissive_power:
  alias: linearizeRadiation
  description: Set to true to linearize emissive power
  ifabsent: string(true)
  range: string
nominal_mass_flow_rate:
  alias: m_flow_nominal
  description: Nominal mass flow rate [kg/s]
  ifabsent: float(0.01)
  range: float
sensible_thermal_mass_scaling_factor:
  alias: mSenFac
  description: Factor for scaling the sensible thermal mass of the zone air volume
    [1]
  ifabsent: float(1)
  range: float
temperature_initial:
  alias: T_start
  description: Initial temperature [K]
  ifabsent: float(294.15)
  range: float
volume:
  alias: null
  description: Air volume of the zone [m3]
  func: lambda self:self.floor_area * self.average_room_height
  type: float

```



## SplitValveParameters
The following parameters are valid for the following classes splitvalve
```yaml
dp_nominal:
  alias: dp_nominal
  description: Pressure drop at nominal mass flow rate, set to zero or negative number
    at outflowing ports [Pa]
  ifabsent: string({5000,-1,-1})
  range: string
fraction_of_nominal_mass_flow_rate_where_transition_to_turbulent_occurs:
  alias: deltaM
  description: Fraction of nominal mass flow rate where transition to turbulent occurs
    [1]
  ifabsent: float(0.3)
  range: float
m_flow_nominal:
  alias: None
  description: Mass flow rate; set negative at outflowing ports [kg/s]
  ifabsent: string(0.15*{1,-1,-1})
  range: string
nominal_mass_flow_rate_for_dynamic_momentum_and_energy_balance:
  alias: mDyn_flow_nominal
  description: Nominal mass flow rate for dynamic momentum and energy balance [kg/s]
  range: float
time_constant_at_nominal_flow_for_dynamic_energy_and_momentum_balance:
  alias: tau
  description: Time constant at nominal flow for dynamic energy and momentum balance
    [s]
  range: float
use_linear_relation_between_m_flow_and_dp_for_any_flow_rate:
  alias: linearized
  description: = true, use linear relation between m_flow and dp for any flow rate
  ifabsent: string(true)
  range: string

```



## ThreeWayValveParameters
The following parameters are valid for the following classes threewayvalve
```yaml
Av:
  alias: Av
  description: Av (metric) flow coefficient [m2]
  range: float
Cv:
  alias: Cv
  description: Cv (US) flow coefficient [USG/min/(psi)^(1/2)]
  range: float
Kv:
  alias: Kv
  description: Kv (metric) flow coefficient [m3/h/(bar)^(1/2)]
  range: float
dp_fixed_nominal:
  alias: dpFixed_nominal
  description: Nominal pressure drop of pipes and other equipment in flow legs at
    port_1 and port_3 [Pa]
  ifabsent: string({2000,0})
  range: string
dp_valve_nominal:
  alias: dpValve_nominal
  description: Nominal pressure drop of fully open valve, used if CvData=Buildings.Fluid.Types.CvTypes.OpPoint
    [Pa]
  ifabsent: float(6000)
  range: float
fra_k:
  alias: fraK
  description: Fraction Kv(port_3->port_2)/Kv(port_1->port_2) [1]
  ifabsent: float(0.7)
  range: float
fraction_of_nominal_flow_rate_where_linearization_starts_if_y_1:
  alias: deltaM
  description: Fraction of nominal flow rate where linearization starts, if y=1 [1]
  ifabsent: float(0.02)
  range: float
m_flow_nominal:
  alias: m_flow_nominal
  description: Nominal mass flow rate [kg/s]
  ifabsent: float(0.15)
  range: float
range_of_significant_deviation_from_equal_percentage_law:
  alias: delta0
  description: Range of significant deviation from equal percentage law [1]
  ifabsent: float(0.01)
  range: float
rangeability:
  alias: R
  description: Rangeability, R=50...100 typically [1]
  ifabsent: float(50)
  range: float
rho_std:
  alias: rhoStd
  description: Inlet density for which valve coefficients are defined [kg/m3]
  range: float
use_linear_relation_between_m_flow_and_dp_for_any_flow_rate:
  alias: linearized
  description: = true, use linear relation between m_flow and dp for any flow rate
  ifabsent: string({true, true})
  range: string
valve_leakage:
  alias: l
  description: Valve leakage, l=Kv(y=0)/Kv(y=1) [1]
  ifabsent: string({0.01,0.01})
  range: string

```



## ValveParameters
The following parameters are valid for the following classes valve
```yaml
av:
  alias: Av
  description: Av (metric) flow coefficient [m2]
  range: string
cv:
  alias: Cv
  description: Cv (US) flow coefficient [USG/min/(psi)^(1/2)]
  range: float
dp_fixed_nominal:
  alias: dpFixed_nominal
  description: Pressure drop of pipe and other resistances that are in series [Pa]
  ifabsent: float(5000)
  range: float
dp_valve_nominal:
  alias: dpValve_nominal
  description: Nominal pressure drop of fully open valve, used if CvData=Buildings.Fluid.Types.CvTypes.OpPoint
    [Pa]
  ifabsent: float(10000)
  range: float
fraction_of_nominal_flow_rate_where_linearization_starts_if_y_1:
  alias: deltaM
  description: Fraction of nominal flow rate where linearization starts, if y=1 [1]
  ifabsent: float(0.02)
  range: float
k_fixed:
  alias: kFixed
  description: Flow coefficient of fixed resistance that may be in series with valve,
    k=m_flow/sqrt(dp), with unit=(kg.m)^(1/2)
  range: string
kv:
  alias: Kv
  description: Kv (metric) flow coefficient [m3/h/(bar)^(1/2)]
  range: float
m_flow_nominal:
  alias: m_flow_nominal
  description: Nominal mass flow rate [kg/s]
  ifabsent: float(0.06)
  range: float
range_of_significant_deviation_from_equal_percentage_law:
  alias: delta0
  description: Range of significant deviation from equal percentage law [1]
  ifabsent: float(0.01)
  range: float
rangeability:
  alias: R
  description: Rangeability, R=50...100 typically [1]
  ifabsent: float(50)
  range: float
use_linear_relation_between_m_flow_and_dp_for_any_flow_rate:
  alias: linearized
  description: = true, use linear relation between m_flow and dp for any flow rate
  ifabsent: string(true)
  range: string
use_m_flow_f_dp_else_dp_f_m_flow:
  alias: from_dp
  description: = true, use m_flow = f(dp) else dp = f(m_flow)
  ifabsent: string(true)
  range: string
valve_leakage:
  alias: l
  description: Valve leakage, l=Kv(y=0)/Kv(y=1) [1]
  ifabsent: float(0.0001)
  range: float

```



## WeatherParameters
The following parameters are valid for the following classes weather
```yaml
path:
  alias: filNam
  description: Name of weather data file
  ifabsent: string(None)
  range: string

```



## AhuParameters
The following parameters are valid for the following classes airhandlingunit
```yaml
dp_nominal:
  alias: dp_nominal
  description: Nominal pressure raise [Pa]
  ifabsent: string(200)
  range: string
heat_exchanger_effectiveness:
  alias: eps
  description: Heat exchanger effectiveness [1]
  ifabsent: string(0.8)
  range: string
m_flow_nominal:
  alias: m_flow_nominal
  description: Nominal mass flow rate [kg/s]
  ifabsent: string(2*100*1.2/3600)
  range: string

```



## ChillerParameters
The following parameters are valid for the following classes chiller
```yaml
eer_nominal:
  description: EER at 35 degC outdoor temperature [1]
  ifabsent: float(3.5)
  range: float
eer_outdoor_slope:
  description: EER change per kelvin of outdoor temperature [1/K]
  ifabsent: float(-0.06)
  range: float
max_electrical_power:
  description: Maximum electrical power of the compressor [W] (nominal cooling power
    / EER when absent)
  ifabsent: float(None)
  range: float
nominal_cooling_power:
  description: Nominal cooling power [W]
  ifabsent: float(10000)
  range: float

```



## DhwTankParameters
The following parameters are valid for the following classes dhwtank
```yaml
daily_draw_off_energy:
  description: Heat drawn by the hot water taps per day [kWh]
  ifabsent: float(8.0)
  range: float
loss_coefficient:
  description: Standing loss coefficient of the tank [W/K]
  ifabsent: float(2.0)
  range: float
min_temperature:
  description: Lower comfort bound of the tank temperature [K]
  ifabsent: float(318.15)
  range: float
set_temperature:
  description: Upper comfort bound of the tank temperature [K]
  ifabsent: float(328.15)
  range: float
supply_offset:
  description: Heat pump supply temperature above the tank temperature [K]
  ifabsent: float(5.0)
  range: float
volume:
  description: Water volume of the tank [m3]
  ifabsent: float(0.2)
  range: float
zone:
  description: Space receiving the standing losses of the tank (first space when absent)
  ifabsent: string(None)
  range: string

```



## BatteryParameters
The following parameters are valid for the following classes battery
```yaml
capacity:
  description: Usable energy capacity [kWh]
  ifabsent: float(10.0)
  range: float
charge_efficiency:
  description: Charging efficiency [1]
  ifabsent: float(0.95)
  range: float
discharge_efficiency:
  description: Discharging efficiency [1]
  ifabsent: float(0.95)
  range: float
initial_soc:
  description: Initial state of charge [1]
  ifabsent: float(0.5)
  range: float
max_charge_power:
  description: Maximum charging power [W]
  ifabsent: float(5000)
  range: float
max_discharge_power:
  description: Maximum discharging power [W]
  ifabsent: float(5000)
  range: float
max_soc:
  description: Upper bound of the state of charge [1]
  ifabsent: float(0.9)
  range: float
min_soc:
  description: Lower bound of the state of charge [1]
  ifabsent: float(0.1)
  range: float

```



## EvChargerParameters
The following parameters are valid for the following classes evcharger
```yaml
arrival_hour:
  description: Hour of the day the vehicle plugs in [h]
  ifabsent: float(18)
  range: float
battery_capacity:
  description: Battery capacity of the vehicle [kWh]
  ifabsent: float(60.0)
  range: float
charge_efficiency:
  description: Charging efficiency [1]
  ifabsent: float(0.9)
  range: float
departure_hour:
  description: Hour of the day the vehicle leaves [h]
  ifabsent: float(7)
  range: float
energy_per_day:
  description: Energy consumed by driving per day [kWh]
  ifabsent: float(10.0)
  range: float
initial_soc:
  description: Initial state of charge [1]
  ifabsent: float(0.5)
  range: float
max_charge_power:
  description: Maximum charging power [W]
  ifabsent: float(7400)
  range: float
target_soc:
  description: State of charge required at departure [1]
  ifabsent: float(0.8)
  range: float
weekend_present:
  description: The vehicle stays plugged in during the weekend
  ifabsent: string(true)
  range: string

```



## PhotovoltaicParameters
The following parameters are valid for the following classes photovoltaic
```yaml
area:
  description: Module area [m2]
  ifabsent: float(20.0)
  range: float
azimuth:
  description: Surface azimuth [deg], 0 for south, 90 for west
  ifabsent: float(0.0)
  range: float
efficiency:
  description: Module and inverter efficiency [1]
  ifabsent: float(0.18)
  range: float
tilt:
  description: Surface tilt [deg], 0 for a roof, 90 for a wall
  ifabsent: float(35.0)
  range: float

```



## SensorParameters
The following parameters are valid for the following classes temperaturesensor,heatmetersensor
```yaml
nominal_mass_flow_rate:
  alias: m_flow_nominal
  description: Nominal mass flow rate through the sensor, used for its dynamics and
    its small-flow regularization [kg/s]
  ifabsent: float(0.15)
  range: float

```


