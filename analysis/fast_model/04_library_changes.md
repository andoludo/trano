# 4. What was changed in FastModel, why, and what it did

Every change lives in `trano/library/FastModel.mo` (and its element templates in
`trano/elements/library/models/fast_model/` and `trano/elements/library/library.json`), keeps the model an explicit
ODE without events (smooth `regStep`, `smoothMax`, `smoothMin`, `smoothLimit`, `sqrt(x^2 + d^2)` for `|x|`; no `when`,
`pre` or `sample`) and was checked to export to CasADi with rumoca 0.10.3 (`tests/test_fast_model.py`). The effects
are measured on BESTEST against the bands and against Buildings ([BESTEST](02_bestest.md), history table) and in the
[study](03_study_against_buildings.md). The plan they came from, with the gaps still open, is
[`docs/validation/fast_model_improvement_plan.md`](../../docs/validation/fast_model_improvement_plan.md).

| # | Change | Gap it closes, and the evidence | Effect |
|---|---|---|---|
| 1 | Smoothing of the controllers as parameters (`dU`, `dY`, `dMax`, `dT`, `dtSet`, `tau`), at their previous values | a calibration needs to widen the transitions of the thermostats and hystereses (mpctrano `FastModel.smooth_controllers`) | the 27 smoothing parameters of the heat-pump house of the tests (`tests/models/house_infiltration_air_water_heat_pump.yaml`) are tunable in the CasADi model |
| 2 | Night ventilation (`ventilationSchedule`) reaches the FastModel zone | BESTEST 650, 950 and their free-floating twins ran without their fan: the YAML field was dropped by the template | the four cases ventilate as in Buildings |
| 3 | Air density at the site pressure (the weather bus carries the atmospheric pressure) | Denver, 1600 m: FastModel used sea-level air for the infiltration and the scheduled ventilation; 940 heating +15.6 %, infiltration responses 1.12 to 1.30 | annual heating against Buildings +9.0 % → −0.4 % on average |
| 4 | `smoothLimit` reaches its bounds exactly (quadratic blends inside `[l, u]`) | it saturated at `l + dX` and `u − dX`: the 1 MW ideal systems of BESTEST leaked 1 kW of heating and 1 kW of cooling around the clock (940: 10.1 MWh of heating for a net 1.4) | heating and cooling demands right in every ideal-system case |
| 5 | Walls discretised as Buildings (`number_of_states` of the YAML, at most three sublayers per layer in closed loop, one node per layer boundary for MPC) | lumped heavy walls cooled the room faster at night and reheated it slower: 940 night heating 53 against 3 kWh, reheat peak +8 % | 940 reheat peak +8 % → +2 % |
| 6 | Single glazing: one balance per pane, no empty array in the record | BESTEST 670 did not translate (`RGap={}` is not valid Modelica for OpenModelica; a branch indexed a gap a single pane does not have) | 670 runs and passes |
| 7 | Long-wave exchange in the room around the current surface temperatures when the YAML asks (`linearize_emissive_power: false`) | FastModel linearised at 20 degC whatever the YAML said; sun-lit floors at 40 to 60 degC exchanged too little | cooling excess about half a point lower, free-floating peaks up to 0.8 K lower |
| 8 | Exact long-wave exchange of the outer surfaces with sky and ground, `sigma*eps*A*(T^4 − TEnv^4)` | the linearisation at the environment temperature understates the loss of a sun-lit surface by a quarter at 60 degC | cooling against Buildings +5.0 % → +2.4 % |
| 9 | Gas gaps of the glazing at the temperatures of their panes (`Envelope.Functions.gapConductance`: Wright's correlations of the vertical cavity and the long-wave exchange across the gap, as Buildings; the record carries the YAML data, not resistances computed in Python) | FastModel held each gap at its EN 673 conductance at 10 degC; a sun-lit glazing at 40 degC conducts a fifth more | free-floating temperatures 0.80 K → 0.38 K from Buildings on average (largest 3.52 → 1.70 K); 680FF hourly RMSE 1.17 → 0.53 K |
| 10 | Hydraulic links (sensors, junctions, the water side of the heat pump) at 100 Pa instead of 1000 Pa at their nominal flow | measured branch by branch: with the living room valve fully open, FastModel passed 36.6 g/s against 43.9 g/s in Buildings, because the sensor feeding the six radiators took a fifth of the head of a branch; the flow laws were not the cause (trano's Buildings valves and radiators are linearised too) | renovated house, set point +1 K: living room +0.75 K (Buildings 0.80, before 0.71), heating response 0.79 → 0.87 |
| 11 | Sublayers built in Modelica from the YAML layers, the counts written out by the templates (`nSta`, `nNod`, `lay`, `nod`, `sub`) | rumoca 0.10.3 sizes arrays and evaluates subscripts only from literal parameters (not from a sum, a function or a record default) | the same results within 3e-5, the discretisation in the `.mo` file rather than in Python |
| 12 | Wall states named alike in both libraries: `T` the layer boundaries (states of FastModel and FastModelMPC), `TSub` the nodes inside the layers (FastModel only) | after change 5, `T[2]` was a boundary in the MPC model and a node inside the first layer in the closed loop; mpctrano copies states by name ([closed loop and MPC](07_closed_loop_and_mpc_states.md)) | mpctrano's flexibility test passes again; the results unchanged (2.4e-5) |
| 13 | Overhangs and side fins (`Envelope.Functions.overhangSunlitFraction`, `sideFinsSunlitFraction`, ports of Buildings' `Overhang` and `SideFins`) | BESTEST 610, 630, 910 and 930 could not be described | the four cases pass, within 2.2 % of Buildings ([shading](05_shading.md)); FastModel gates in BESTEST |

The validation tooling was corrected on the way too: the study reads the heating of the ideal systems and the gas
and electricity of the productions from their energy meters rather than from samples of their powers (the samples of
the morning reheat and of the boiler restarts are caught or missed at random, see
[the boiler water loop](09_boiler_water_loop.md)); the flat Buildings model is under-determined, the study renders
Buildings with its containers as the BESTEST validation does.

## What was ruled out

* **The flow laws of the hydronic circuits** (change 10): the gap of the set point response was the resistance of the
  links, not the linearised valves.
* **The sun on the opaque surfaces** (the open gap of [open gaps](10_open_gaps.md)): the irradiance on tilted surfaces,
  the outside convection with its wind modifier, the long-wave exchange with sky and ground and the discretisation of
  the walls (tried up to nine sublayers) are those of Buildings; the response to the absorptance stays at 0.91 to 0.93.
* **Energy losses in the water loop of FastModel** (the gas of the house before renovation): the loop conserves energy
  within 0.3 % ([the boiler water loop](09_boiler_water_loop.md)).
