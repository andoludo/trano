# 9. The water loop of the boiler house: why the gas looked 1 % off, and why it is not

**Question.** In the house before renovation, the study first reported FastModel's gas 3.3 % above Buildings, then
2.6 %, for 1.7 % more heat in the rooms. Does FastModel lose or misplace heat in its water loop (boiler, pump,
three-way valve, six radiators with thermostatic valves), or burn more gas per unit of heat?

**Answer.** No. Both water loops close their energy balance from the energy meters within 0.15 % over the month, the
jacket losses of the boilers are identical (127.3 kWh), and the two libraries burn the same gas per unit of heat
within 0.2 % (1.230 and 1.233 kWh of gas per kWh delivered by the radiators). FastModel burns 1.9 % more gas because it
delivers 1.7 % more heat. The 1 % in excess came from the study itself, which read the gas from samples of the fuel
power five minutes apart. Those samples catch or miss the two-minute bursts of the boiler when the valves open in the
morning: −0.75 % for Buildings, +0.6 % for FastModel. The study now reads both meters
(`validation/fast_model/harness.py`).

Data: [`results/water_loop_month.json`](results/water_loop_month.json) (28 days, by window),
[`results/water_loop_restart.json`](results/water_loop_restart.json) (days 8 to 10),
[`results/water_loop_restart_table.csv`](results/water_loop_restart_table.csv) (the morning restart),
[`results/water_loop_components_restart.csv`](results/water_loop_components_restart.csv),
[`results/water_loop_restart_tau0.5.json`](results/water_loop_restart_tau0.5.json); script
[`scripts/water_loop.py`](scripts/water_loop.py).

## Method

The base case is simulated with both libraries in OpenModelica, keeping the variables of the loop. The energy balance
of each loop, in kWh over a window:

* gas: the increments of the fuel meter (`EFue`, an integrator in Buildings, a state in FastModel), exact whatever
  the output interval; next to it, the integral of the *sampled* fuel power, as the study computed it before;
* into the water: efficiency (0.85) times the gas, minus the jacket loss of the boiler to its room. Buildings 13's
  `BoilerPolynomial` injects `QWat_flow = eta*QFue_flow + UAOve.Q_flow` into the water and then conducts `UAOve` out of
  it again, so its loss counts twice; FastModel's port of it does the same with a doubled `UA`. Plus the pump power,
  which Buildings adds to the water;
* out of the water: the heat of the six radiators to their rooms (convective and radiant ports);
* stored: the change of the heat in the water and metal of FastModel's radiators and boiler;
* residual: in − out − stored, the energy the loop would create or lose.

## The month (28 days of January)

| | Buildings | FastModel |
|---|---|---|
| Gas from the meter | 3390.6 kWh | 3454.2 kWh (+1.9 %) |
| Gas from the 5-minute samples (what the study read) | 3365.3 kWh (−0.75 %) | 3475.6 kWh (+0.62 %) |
| Jacket loss of the boiler | 127.3 kWh | 127.3 kWh |
| Pump heat | 4.0 kWh | (not modelled) |
| Into the water | 2758.7 kWh | 2808.8 kWh |
| Radiators to the rooms | 2755.5 kWh | 2801.9 kWh (+1.7 %) |
| Stored in water and metal | (not computed) | 2.6 kWh |
| Residual | 3.2 kWh | 4.3 kWh |
| Gas per unit of heat | 1.230 | 1.233 |

The first day is the warm-up from the initial temperatures (20 degC everywhere): both libraries burn 71 to 72 kWh of
gas and leave 2.2 kWh in the water and metal of the loop (FastModel's storage is computed, Buildings' is its residual
of that day). After it, the residuals are 1.0 kWh (Buildings) and 3.9 kWh (FastModel) on 2700 and 2750 kWh: 0.04 and
0.14 %.

## The morning restart

At 06:00 the bathroom's set point steps up, at 07:00 those of the living spaces. Day 8, every 15 minutes:

| Time | Buildings: firing, water, gas | FastModel: firing, water, gas |
|---|---|---|
| 05:45 | 0.13, 74.87 degC, 2.4 kW | 0.13, 74.87 degC, 2.4 kW |
| 06:00 | 0.13, 74.87 degC, 2.5 kW | **1.00, 73.97 degC, 18.8 kW** |
| 06:15 | 0.15, 74.85 degC, 2.8 kW | 0.16, 74.84 degC, 3.1 kW |
| 06:30 | 0.33, 74.67 degC, 6.3 kW | 0.27, 74.73 degC, 5.0 kW |
| 07:00 | 0.24, 74.76 degC, 4.5 kW | **0.81, 74.19 degC, 15.2 kW** |
| 07:15 | 0.62, 74.38 degC, 11.7 kW | 0.61, 74.39 degC, 11.5 kW |

When a thermostatic valve opens, cold water from its radiator returns to the boiler; the water dips and the
proportional controller on the boiler temperature (gain 1/K, set point 75 degC, the same in both libraries) fires up
to full power for a minute or two. Both libraries do it: the boiler water reaches its lowest at 72.0 degC in Buildings
and 72.2 degC in FastModel over these days. FastModel's set points change as a smooth step of 60 s centred on the
switch time, so its burst starts 30 s before the hour, where Buildings' starts at the event. That is why FastModel's
extra gas shows by hour of the day at 05:00 to 07:00 ([`results/study_hour_of_day.csv`](results/study_hour_of_day.csv)),
and why samples five minutes apart misread it: a sample that falls in a two-minute burst at 18.8 kW counts it for
five minutes, a sample that falls next to it counts none.

## Two leads that were not the cause

* **The compliance of the pressure nodes.** FastModel's pressure-driven circuits give the inlet of each component a
  small compliance (`tauHyd = 5 s`) whose stored mass the energy balance does not follow. With `tauHyd = 0.5 s` the
  residual over days 8 to 10 is 0.39 kWh against 0.26 kWh at 5 s, on 192 kWh: no difference
  ([`water_loop_restart_tau0.5.json`](results/water_loop_restart_tau0.5.json)).
* **A component that loses heat.** The balance of each FastModel component from its port flows and temperatures,
  sampled every 5 minutes ([`water_loop_components_restart.csv`](results/water_loop_components_restart.csv)), does not
  close: it sums to 42 W where the meters give 5 W, and its terms depend on the reference temperature of the enthalpy,
  because the samples do not resolve the flow transients of the valves. That per-component audit first suggested a
  leak in the radiators; the exact balance from the meters shows there is none.

## What it changed

* The study keeps the meters of Buildings (`EFue.y`, `EEle.y`, `EFan.y`) and reads every production from its meter
  first (`validation/fast_model/harness.py`): the gas of the house before renovation is +1.9 % for +1.7 % heat.
* Nothing in the library: FastModel's boiler, pump, valves and radiators conserve energy and burn gas per unit of heat
  as Buildings does.
