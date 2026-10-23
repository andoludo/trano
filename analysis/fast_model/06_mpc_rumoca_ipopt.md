# 6. The residential houses as MPC models: rumoca, CasADi, IPOPT

**Question.** Do the complex trano houses, with their real systems (a heat pump with a buffer, a gas boiler, a
three-way valve, six radiators with thermostatic valves, a ventilation unit, PV), translate to CasADi through rumoca
and solve as an optimal control problem with IPOPT?

**Answer.** Yes, both. The FastModelMPC model of each house exports in under a minute, IPOPT solves a 12-hour optimal
control in 23 to 55 s (`Solve_Succeeded`), every zone is held at its comfort bound without slack, and the energy
meters are recognised as the cost (electricity of the heat pump and fans, PV production; gas of the boiler).

Data: [`results/mpc_residential_after_renovation.json`](results/mpc_residential_after_renovation.json),
[`results/mpc_residential_before_renovation.json`](results/mpc_residential_before_renovation.json) (exports, roles,
the trajectories of every control and zone); script [`scripts/mpc_houses.py`](scripts/mpc_houses.py); test
`tests/test_fast_model.py::test_the_optimal_control_heats_the_residential_house` (both houses).

## Method

trano writes the house with the `FastModelMPC` library: the same physics as the closed-loop `FastModel`, with the
controllers removed and their outputs made inputs (`*_y.y`, bounded to 0..1), and one node per layer boundary in the
walls (the closed loop splits the layers as Buildings does). rumoca 0.10.3 translates it into a CasADi ODE
(trano-optimization `CasadiModel.from_modelica`); trano-optimization infers the roles of the inputs and states (the
controls, the disturbances, the comfort temperatures `TAir`, the energy meters and their carriers) and builds a
collocated optimal control problem solved by IPOPT. The scenario: 12 hourly steps on a cold, sunless night (0 degC
outside, sky at −10 degC, wind 3 m/s), every zone at or above 20 degC, energy at 0.25 per kWh, starting from the
initial state of the model (zones at 21 degC).

## Results

| | After renovation (heat pump) | Before renovation (gas boiler) |
|---|---|---|
| FastModelMPC export | 60 s: 189 states, 28 inputs, 4165 parameters | 45 s: 160 states, 27 inputs, 3544 parameters |
| Closed-loop FastModel export | 54 s: 278 states, 19 inputs | 42 s: 224 states, 19 inputs |
| Controls | heat pump, three-way valve, ventilation unit, six radiator valves (9) | boiler, three-way valve, six radiator valves (8) |
| Meters and carriers | heat pump `EEle` electricity, fans `EFan` electricity, PV `EPro` production | boiler `EFue` fuel |
| IPOPT | `Solve_Succeeded` in 55 s | `Solve_Succeeded` in 23 s |
| Comfort slack, largest | 0 K | 0 K |
| Lowest zone temperature after 3 h | 20.00 degC | 20.00 degC |
| Energy over the 12 h | heat pump 0.93 kWh, fans 0.30 kWh | gas 57.1 kWh |
| Cost (0.25 per kWh) | 0.31 | 14.27 |

What the optimiser does, from the trajectories in the JSON files: both houses let their zones fall from 21 to 20 degC
in the first hour and then hold them exactly at the bound. The renovated house first lives on the heat of its buffer
and walls (heat pump off for four hours, then rising to 0.20 of its speed), opening the three-way valve and the
radiator valves step by step; the old house has no buffer and its walls lose heat fast, so its boiler fires from the
first hour (0.19, rising to 0.32 of its power) with the three-way valve fully open on the boiler and the bedroom valves
nearly open.

The times are those of this container (4 cores) and include the CasADi construction of the problem; the test of the
repository runs both houses in about three minutes.

## What made it possible

* Every component of the library is an explicit, smooth ODE (no events, no algebraic loops), which CasADi and IPOPT
  need.
* rumoca 0.10.3 sizes arrays and evaluates subscripts only from literal parameters: the templates write out the
  sublayer counts and node indices of the walls (`nSta`, `nNod`, `lay`, `nod`, `sub`) instead of computing them.
* The thermostatic valves, the three-way valve and the production become inputs in FastModelMPC, with the same
  hydraulics as the closed loop in their set-flow form.
* The state names of the walls are shared with the closed loop, so that a closed-loop simulation can start the MPC
  model ([closed loop and MPC](07_closed_loop_and_mpc_states.md)).
