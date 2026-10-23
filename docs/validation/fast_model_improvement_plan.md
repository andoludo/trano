# FastModel improvement plan

What to change in the `FastModel` library so that it stays within the bands of Buildings on the zone temperatures,
the heating power and the heating demand **and** responds to a parameter change as Buildings does. The plan follows
from the [study of BESTEST 950FF, 940, 960 and the renovated house](fast_model_study.md) and from a reading of the
equations of `trano/library/FastModel.mo` against the Buildings components they stand for. Each item names the gap,
the evidence, the change, its cost in states or in smoothness (FastModel must stay an explicit, event-free ODE for
rumoca and CasADi) and the validation that closes it.

The study ranks the gaps by what they change. When it started, in Denver, where the air is thin and the BESTEST walls
are heavy, FastModel heated 7 to 16 % more than Buildings and responded 12 to 30 % more to the infiltration; in Uccle,
with the renovated house, it stayed within 1 % of the meters and 5 % of the sensitivities, except where thermostatic
valves share the flow of the circuit (set point response 0.80). Two defects were fixed during the study: the night
ventilation that FastModel dropped and the 1 kW leak of its ideal systems. Three stages of changes followed (see
[Progress](#progress)); the extended study (sweeps, glazing, sun, gains, sun-space, ventilation, radiators) then found
the gaps of the last stage and the ones still open.

| Priority | Change | Evidence in the study | Cost | Closed when | Status |
|---|---|---|---|---|---|
| 1 | Air density from the site pressure | 940: about 140 of the +186 kWh; infiltration responses 1.12 to 1.30 in Denver, 0.95 in Uccle | one expression, no state | 940 heating within 5 % of Buildings, infiltration ratios 0.9 to 1.1 | done (phase 1) |
| 2 | Discretise the layers (`number_of_states`) | 940: night heating 53 against 3 kWh, reheat peak +12 %; mass responses 1.13 to 1.45; sun-space +0.53 K | states: 4 to about 12 per heavy wall | 940 peak within 5 %, 950FF mass ratios 0.9 to 1.1, sun-space bias within 0.2 K | done (phase 2) |
| 3 | Small resistances on the hydraulic links (not the flow law: trano's Buildings valves and radiators are linearised too) | house, set point +1 K: response 0.80; with the living room valve open, 36.6 against 43.9 g/s, the valve fully open 59 % of the pumping time against 44 % | none (a parameter) | set point responses within 10 % | done (phase 3); the supply water at full load is still 0.4 K cool (priority 9) |
| 4 | Long-wave exchange around the current surface temperatures | sun-space: 0.87 K RMSE, highest temperature +0.4 K | none (moving linearisation) | sun-space RMSE below 0.5 K | done: room exchange around the surface temperatures, exact sky exchange |
| 5 | Exact smooth limits, heat pump safety times | ideal systems leaked 1 kW (fixed by rescaling); −11 W overshoot; buffer and supply peak responses 1.33 to 4.5 | one state per heat pump | no saturated controller output above 1e-6 of its range; peak ratios 0.8 to 1.2 | smooth limits done (phase 1), safety times open |
| 6 | Shading, unbalanced ventilation, wind- and stack-driven infiltration, ISO 13370 ground | BESTEST 610, 630, 910, 930 not runnable; supply air 0.3 K warm | per feature | the four shading cases inside the BESTEST bands | shading done (phase 4): the four cases inside their bands, within 2.2 % of Buildings; the rest open |
| 7 | Gas gaps of the glazing at the temperatures of their panes | free-floating zones 0.2 to 0.9 K warm on average, up to 3.5 K at their peaks; 680FF RMSE 1.17 K | none in states (an explicit function of the pane temperatures) | free-floating temperatures within 0.5 K on average | done (phase 3): 0.38 K on average, 1.70 K at most |
| 8 | Response to the solar absorptance of the outer surfaces | 0.6 to 0.9: ratios 0.91 to 0.93 on every KPI of 950FF and 940 | to be found | ratios 0.95 to 1.05 | open: the irradiance, the outside convection and sky exchange and the discretisation are ruled out |
| 9 | Heat pump at full load: capacity map, safety times | house: supply water 0.4 K cooler while the living room valve is open; set point +1 K: heating 0.87, electricity 0.76; buffer x3: peak −5 % against −1 %; EPS x0.5: peak 1.51 | none for the map; one state per heat pump for the safety times | supply within 0.2 K at full load, peak ratios 0.8 to 1.2 | open |

The [study](fast_model_study.md) is the yardstick: each change is merged with its run of
`python -m validation.fast_model` (the workflow `fast_model_study.yaml`) and the BESTEST run of FastModel, and the
tables of the study and of the [BESTEST page](bestest.md) are rendered from the new results.

## Progress

The first phases are in the library (`trano/library/FastModel.mo`, its templates and `library.json`). Every step
kept the models smooth and without events, and every FastModel model still exports to CasADi with rumoca
(`tests/test_fast_model.py`). Each step was measured on the BESTEST cases FastModel can describe (23 of 27 until the four
with shading could run, all 27 since) against the band of the standard and against the values of Buildings for the
same YAML:

| Library | Cases | KPIs inside their band | Annual heating against Buildings, mean (range) | Annual cooling against Buildings, mean (range) | Loads, mean gap | Free-floating temperatures, mean gap (largest) |
|---|---|---|---|---|---|---|
| Before (first FastModel run) | 22 | 83 of 85 | +9.0 % (+5.0 to +20.4 %) | +3.1 % (+1.5 to +4.3 %) | 5.6 % | 0.48 K (1.49 K) |
| Air density, exact smooth limits (phase 1) | 22 | 85 of 85 | −0.4 % (−3.6 to +2.6 %) | +5.5 % (+3.0 to +9.2 %) | 3.4 % | 1.04 K (4.39 K) |
| Discretised walls, single glazing (phase 2) | 23 | 87 of 89 | −0.6 % (−3.3 to +2.1 %) | +5.5 % (+1.7 to +9.3 %) | 3.1 % | 1.16 K (4.57 K) |
| Room long-wave exchange around the surface temperatures | 23 | 89 of 89 | −0.6 % (−3.2 to +1.6 %) | +5.0 % (−0.1 to +8.1 %) | 2.7 % | 0.93 K (3.82 K) |
| Exact long-wave exchange with sky and ground | 23 | 89 of 89 | +0.6 % (−1.0 to +2.5 %) | +2.4 % (−2.2 to +6.6 %) | 2.0 % | 0.80 K (3.52 K) |
| Gas gaps at the temperatures of their panes, small hydraulic links (phase 3) | 23 | 89 of 89 | +0.2 % (−0.9 to +2.5 %) | +1.4 % (−2.3 to +4.4 %) | 1.2 % | 0.38 K (1.70 K) |
| Overhangs and side fins (phase 4) | 27 | 105 of 105 | +0.2 % (−0.9 to +2.5 %) | +1.3 % (−2.3 to +4.4 %) | 1.1 % | 0.38 K (1.70 K) |

What each step did:

* **Air density at the site** (phase 1). The weather bus carries the atmospheric pressure (101325 Pa, or the pressure
  of the site's altitude when the YAML reads it from the weather file), and the zone uses the ideal-gas density at
  that pressure and its air temperature. The heating of the Denver cases fell from +9 % to −0.4 % of Buildings.
  It also showed that the excess infiltration had hidden a heat surplus: with it gone, FastModel cooled 5 % more
  than Buildings and its free-floating peaks rose by up to 4 K above those of Buildings.
* **Smooth limits that reach their bounds** (phase 1). `smoothLimit` is exactly its bound beyond a quadratic blend
  and never leaves the range; it used to saturate 0.1 % inside, which made the ideal systems leak 1 kW of heating
  and of cooling at all times. The solar angles keep the interior limit (`smoothLimitInside`), acos being singular
  at the bounds.
* **Discretised walls** (phase 2). The closed-loop model splits each layer into the states Buildings gives it, at most
  three sublayers; FastModelMPC keeps one node per layer boundary, its states being the decision variables of the
  optimal control. The reheat peak of 940 after the night set-back fell from +8 % to +2 % of Buildings. The single
  glazing of 670 runs at last: its record wrote an empty array, `{}`, that OpenModelica does not parse.
* **Long-wave exchange in the room** around the mean surface temperature when the YAML sets
  `linearize_emissive_power: false`, as every BESTEST case does (Buildings then exchanges exactly). The cooling
  excess fell by about half a point and the free-floating peaks by up to 0.8 K.
* **Exact long-wave exchange with sky and ground** for outer walls and windows,
  \(\sigma\varepsilon A (T^4 - T_{env}^4)\) instead of its linearisation at \(T_{env}\): a sun-lit outer surface
  loses a quarter more at 60 degC. The cooling excess fell from +5.0 to +2.4 % on average.
* **Gas gaps at the temperatures of their panes** (phase 3). FastModel held each gap of a glazing at its EN 673
  conductance, computed by trano's Python at 10 degC; Buildings recomputes the long-wave exchange across the gap
  (\(\sigma (T_a^4 - T_b^4)/(1/\varepsilon_a + 1/\varepsilon_b - 1)\), which grows as \(T^3\)) and the convection
  of the gas (Wright's correlations of the vertical cavity) at the temperatures of the panes. A sun-lit glazing at
  40 degC conducts a fifth more than at 10 degC, so FastModel kept too much of the sun in the zones that nothing cools.
  `Envelope.Functions.gapConductance` now computes both in Modelica from the data of the YAML (the conductivity of the
  panes, the properties of the gas, the emissivities facing each gap), an explicit and smooth function of the pane
  temperatures; `GGap` stays a parameter (the conductance at the EN 673 state, scaled by the temperatures), so that
  mpctrano still calibrates it. The free-floating temperatures came within 0.38 K of Buildings on average (0.80 K
  before) and 1.70 K at the hottest hour (3.52 K); 680FF went from 1.17 to 0.53 K hourly RMSE.
* **Small hydraulic links** (phase 3). The study blamed the flow law for the set point responses of the renovated
  house; trano's Buildings valves and radiators are linearised too (`linearized=true`), with the same conductances as
  FastModel. Measured branch by branch, the gap was elsewhere: with the living room valve fully open, FastModel passed
  36.6 g/s against 43.9 g/s in Buildings, because the components that have no resistance in Buildings (the sensor
  feeding the six radiators, the water side of the heat pump) had a linear one of 1000 Pa at their nominal flow, a
  fifth of the head of a radiator branch. At 100 Pa, the living room warms by 0.75 K with a set point +1 K (Buildings
  0.80 K, before 0.71 K) and the kitchen by 0.64 K (0.71, 0.60).
* **Layers split in Modelica** (phase 3). The construction record carries the layers of the YAML and the number of
  sublayers of each (the states Buildings gives it, at most 3; one for FastModelMPC); `PartialOpaque` splits them. The
  counts stay literal in the record: rumoca 0.10.3 sizes arrays only from literal parameters (not from a sum, nor from
  a default of the record). Same results within 3e-5. The walls name `T` the temperatures of the layer boundaries,
  the states of both FastModel and FastModelMPC, and `TSub` the nodes inside the layers (FastModel only): a state of
  one model copied into the other by its name stays at its place. Before, `T[2]` of the MPC model was a layer boundary
  and `T[2]` of the closed-loop model a node inside the first layer; mpctrano's spin-up, which copies the states by
  name, started the MPC model with the inner surfaces of its walls inside the insulation.
* **Overhangs and side fins** (phase 4). The window takes the geometry of the YAML (`overhang`, `side_fins`) and
  shades its direct irradiance by the sunlit fraction of `Buildings.HeatTransfer.Windows.Overhang` and `SideFins`,
  ported to `Envelope.Functions.overhangSunlitFraction` and `sideFinsSunlitFraction`: the same areas of the shadow
  in the plane of the window, with `smoothMax`/`smoothMin` in place of `max`/`min`, the tangents written as a sine
  over a cosine kept away from zero, and `regStep` blends where Buildings switches (the sun behind the wall, below
  the horizon). With both, the fractions combine as in Buildings (\(f_{ove} + f_{fin} - 1\), kept above 0.001).
  The diffuse irradiance is not shaded, as in Buildings. A unit test checks the fractions against their closed form
  at a few sun positions. The four shading cases run and pass every KPI, within 2.2 % of Buildings (910 peak
  cooling), and FastModel gates in BESTEST as Buildings and IDEAS do. A window without shading keeps its fraction
  at 1 through an evaluated parameter: the six-zone house still exports with rumoca (189 states, 28 inputs for
  FastModelMPC) and IPOPT solves its 12-hour optimal control in about a minute.

**Still open.** The free-floating zones of the light, well-insulated constructions still peak 1 to 1.7 K above
Buildings (600FF, 650FF, 680FF) and the 960 sun-space is 0.4 K warm on average. FastModel responds 8 % less than
Buildings to the solar absorptance of the outer surfaces (priority 8). In the renovated house, the supply water is
0.4 K cooler when the heat pump runs at full load, and the responses to the set point (heating 0.87, electricity 0.76),
to the buffer and to the peaks go through the heat pump (priority 9).
Unbalanced ventilation, wind- and stack-driven infiltration and the ISO 13370 ground are not modelled (priority 6). The [proposals](#proposals-for-the-remaining-gaps) give a formulation of each
that keeps the model explicit and smooth.

## Gaps found by reading the equations

These differences exist whatever the case; the study measures how much each of them weighs.

### 1. One node per layer boundary in the opaque constructions

`Envelope.PartialOpaque` puts one capacity at each boundary between the layers of a construction (\(n\) layers,
\(n + 1\) nodes), half of each adjacent layer lumped into the node. Buildings discretises each layer into
`nSta` states; the BESTEST cases ask for 18 states per 0.2 m of layer (`number_of_states` in the YAML, ignored by
FastModel). A 0.1 m concrete block is one lumped capacity in FastModel and about nine in Buildings. The conductance
of a layer and its total capacity are the same, so the **steady-state heat loss is identical** and a conductivity
change moves the demand alike; what differs is the **dynamics**: the lumped wall responds faster to a step of the
room or outdoor temperature and stores the diurnal solar gain differently. That weighs on the free-floating
extremes, on the morning reheat peak after a set-back and on the response to a change of the thermal mass.

Change: discretise each layer into `nSta` nodes as Buildings does, with `nSta` read from `number_of_states` (default
1 to keep today's models) and a cap for the MPC export; or, cheaper, two nodes per layer thicker than a threshold.
Cost: states (a heavy BESTEST wall goes from 4 to about 12 nodes), none in smoothness.

### 2. Air density and the site pressure

The zone uses \(\rho = 1.2\) kg/m3 for the infiltration (`rhoAir*V*ACH/3600`). Trano's Buildings zone takes the
density of the air at the atmospheric pressure of the weather file when the YAML asks for it
(`atmospheric_pressure_source: File`, as the BESTEST cases do): in Denver, 1609 m above sea level, that is about
0.98 kg/m3. The infiltration heat flow of FastModel is then about 20 % larger than that of Buildings for the same
air change rate, and so is its response to a change of the air change rate. The scheduled night ventilation is a
mass flow in both libraries and is not affected.

Change: `rhoAir` from the pressure of the weather bus (`pAtm`, ideal gas at the zone temperature or at 20 degC),
one algebraic expression. Cost: none.

### 3. Long-wave exchange linearised at 20 degC

The radiant star of the zone uses \(G_i = 4\sigma T_0^3 \varepsilon_i A_i\) at \(T_0 = 20\) degC; the BESTEST
cases ask Buildings for the non-linear exchange (`linearize_emissive_power: false`). The error grows with the
spread of the surface temperatures: a sun-lit floor at 35 degC and a window at 5 degC in winter, the free-floating
zone at 60 degC in summer. Change: linearise around the current mean surface temperature (\(T_0 = T_{rad}\) of the
previous step is not available in an ODE; \(T_0^3\) as a smooth function of the mean surface temperature is), or
keep the exact \(\sigma \varepsilon (T_i^4 - T_{rad}^4)\) with the star temperature solved implicitly. Cost: the
exact form adds an algebraic loop; the moving linearisation adds none.

### 4. Controllers without events and without safety times

A hysteresis switches over `tau` (10 s by default, a parameter since this branch) instead of at an instant, and the
heat pump has no minimum on and off times. The heat pump starts and stops at other instants than in Buildings,
which shows in the water temperatures and the hour-by-hour heat of the radiators (see the
[replay validation](fast_model.md)), less in the zones and the meters. Change: a smooth minimum on/off time
(a first-order timer gating the compressor), and `smooth_controllers` widths chosen per use (simulation: narrow;
calibration: the x5, 600 s of mpctrano). Cost: one state per heat pump.

### 5. Hydraulics linearised, one production per circuit, mixing valves only

Pressure-driven flows with resistances linearised at their nominal flows, as trano's Buildings models are
(`linearized=true` on the valves and radiators: the same conductances), and a small resistance for the components
that have none in Buildings, each inlet a pressure node of small compliance so that the flows stay explicit. That
small resistance was 1000 Pa at the nominal flow, a fifth of the head of a radiator branch behind a six-way manifold;
it is 100 Pa since phase 3 (2 % of the branch). A pressure node keeps its last value when its pump stops (the head
of a stopped pump reads 20 kPa instead of 0), which changes nothing but the reported head.

### 6. Heat pump performance map

The Vitocal 251-A08 table of Buildings is scaled to the capacity of the YAML, interpolated and smoothed in the
source temperature, quadratic in the condenser temperature; the condenser outlet is approximated from the buffer
temperature. Change: fit the condenser side on the EN 14511 table too (bilinear smooth interpolation), keep the
scaling. Cost: none in states.

### 7. Balanced ventilation and fans at constant pressure

The unit is balanced (supply = extract) while in Buildings part of the supply leaves by the infiltration paths,
so that the recovery works on a smaller extract stream; the fans run at constant pressure rise. Change: an extract
flow equal to the supply minus the infiltration pushed out, as the pressures of Buildings set it. Cost: none.

### 8. Missing features

Storage tanks on boilers, water-to-water heat pumps, chillers,
domestic hot water, batteries, EV chargers, VAV, air handling units other than system D, power sensors, moisture,
wind- and stack-driven infiltration, the ISO 13370 ground of IDEAS. The night ventilation schedule and the overhangs and side fins (BESTEST 610,
630, 910, 930) were missing and are added by this branch.

## Proposals for the remaining gaps

What rumoca and CasADi need of FastModel, learnt on the way: an explicit ODE (every algebraic variable a function of
the states, the inputs and the parameters: no algebraic loop, a pressure node of small compliance where Buildings
solves the flows of a circuit), once continuously differentiable equations without events (`regStep`, `smoothMax`,
\(\sqrt{x^2 + \delta^2}\) for \(|x|\), no `when`, `pre` or `sample`), and, for rumoca 0.10.3, array sizes and
subscripts given by literal parameters: not a sum, not the result of a function, not a default of a record (the
sublayers of a construction are written out by trano for that reason), and no loop in a function over a parameter
that stays tunable. Each proposal below keeps to that; its cost is in states and in parameters, its gate in the study
or in BESTEST.

| Gap | Proposal | Cost | Gate |
|---|---|---|---|
| Response to the outer solar absorptance (0.91) | Find where the heat goes: compare the outer surface temperatures and the conduction into the room of a sun-lit wall in both libraries (the irradiance, the outside convection with its wind modifier, the sky exchange and the sublayers, up to 9, are ruled out); the room-side long-wave exchange of the opaque surfaces (radiant star against Buildings' radiosity) is the next suspect | none until the cause is known | absorptance ratios 0.95 to 1.05 |
| Supply water at full load | The condenser outlet and the capacity from the EN 14511 table of the heat pump, interpolated as the other optics and curves of FastModel are (a polynomial fit evaluated in Modelica, smooth in the source and condenser temperatures) instead of the buffer temperature | parameters only | supply within 0.2 K of Buildings at full load, set point responses within 5 % |
| Heat pump safety times | A first-order lag on the compressor command, \(\tau \dot y = y_{on} - y\), with \(\tau\) a third of the minimum run time of the YAML: the compressor follows its hysteresis but cannot start and stop faster than the safety times allow, on average, without an event (Buildings holds each start and stop exactly, with events) | one state per heat pump | buffer and supply peak ratios 0.8 to 1.2 |
| Unbalanced ventilation | The extract of a ventilation unit equal to its supply minus the air the infiltration paths push out, as the pressures of Buildings share it: an explicit expression of the flows | none | supply air within 0.1 K |
| Wind- and stack-driven infiltration | \(\dot m = \rho A_L \sqrt{C_s \sqrt{\Delta T^2 + \delta^2} + C_w v^2}\) (LBL), explicit in the weather and the zone temperature | parameters only | infiltration responses within 10 % against IDEAS |
| Ground (ISO 13370) | A virtual layer of soil with its periodic temperature, as IDEAS: two states per floor on the ground | two states per floor | ground losses within 10 % of IDEAS |

## Plan

### Phase 1: the steady state (days)

1. **Air density at the site.** `PartialZone` takes the atmospheric pressure of the weather bus (`pAtm`, already in
   the weather data of trano) and computes \(\rho = p_{atm} / (R_{air} T_{air})\) for the infiltration and the
   scheduled ventilation of the zone, as `Buildings.ThermalZones.Detailed.MixedAir` does through its medium (an ideal
   gas at the pressure of the site); the heat capacity of the air volume takes the same density. For the MPC export the pressure is a disturbance already, so the ODE
   stays explicit. *Gate:* 940 heating within 5 % of Buildings (today +15.6 %), 940 infiltration ratio 0.9 to 1.1
   (today 1.30), the renovated house unchanged within 0.5 %.
2. **Smooth limits that reach their bounds.** Replace `smoothLimit(x, l, u, dX)` (saturating at \(l + dX\) and
   \(u - dX\)) by a limit whose smooth transitions lie inside \([l, u]\) and end exactly on the
   bounds (the `regStep` blends of `smoothMax` and `smoothMin` shifted inwards by \(dX\), clipped so that they
   cannot overshoot), in every controller and actuator, and drop the per-component rescaling added during the study. *Gate:* a unit test on the controllers
   (saturated output exactly 0 and 1, no output outside \([0, 1]\)), the ideal-system test of
   `tests/test_fast_model.py`, BESTEST cooling and heating of FastModel within their bands.

### Phase 2: the dynamics (weeks)

3. **Layers discretised as Buildings does.** `PartialOpaque` gets `nSta[nLay]` (from `number_of_states` of the YAML,
   which trano already renders for Buildings, scaled to the layer thickness as `nStaRef` does), each layer split into
   `nSta` conductances and capacities. The default keeps today's model (one node per boundary) for MPC, where the
   state count matters; the closed-loop `FastModel` takes the YAML. A middle option is a minimum of two nodes for a
   layer whose time constant exceeds a few hours. *Gate:* 940 peak heating within 5 % (today +12 %), night heating
   within 20 kWh, 950FF mass ratios 0.9 to 1.1 (today 1.13 to 1.45), sun-space bias within 0.2 K (today +0.53 K),
   the MPC export of the renovated house below 1.5 times its states.
4. **Long-wave exchange around the current temperatures.** Linearise each radiant conductance at the mean of the
   surface temperatures of the zone (\(G_i = 4\sigma\varepsilon_i A_i \bar T^3\), \(\bar T\) a smooth function of the
   states) instead of 20 degC, or keep the exact \(\sigma\varepsilon(T_i^4 - T_{rad}^4)\) with the star solved
   implicitly in the closed-loop model only. *Gate:* sun-space RMSE below 0.5 K (today 0.87 K), 960 and 600FF/900FF
   extremes within 0.5 K of Buildings.

### Phase 3: the systems (weeks)

5. **Flow sharing between radiator branches.** Done differently: the flow laws are the same as trano's Buildings
   models (linearised); the resistances of the links that Buildings does not have are ten times smaller (phase 3).
   *Gate kept:* the set point and radiator responses of the renovated house within 10 %.
6. **Heat pump safety times and condenser.** Minimum on and off times as a smooth timer state gating the compressor,
   and the condenser outlet from the EN 14511 table rather than the buffer temperature. *Gate:* the water
   temperatures of the [replay validation](fast_model.md) within 1 K (today about 2 K in closed loop), buffer and
   supply peak ratios 0.8 to 1.2.

### Phase 4: coverage

7. **Overhangs and side fins** (`Buildings.HeatTransfer.Windows.Overhang` and `SideFins` geometry, a smooth shaded
   fraction of the direct irradiance), so that BESTEST 610, 630, 910 and 930 run with FastModel. *Done:* the four
   cases inside their bands, within 2.2 % of Buildings.
8. **Unbalanced ventilation** (extract = supply minus the air pushed out by the infiltration paths), **wind- and
   stack-driven infiltration**, the **ISO 13370 ground** of IDEAS for the comparisons with IDEAS.

### Gates kept after the plan

* The study runs on every change of `trano/library/FastModel.mo` or of the FastModel templates (the workflow does so
  when the commit carries `[study]`); a test reads its committed results and fails when a zone RMSE exceeds 0.5 K
  (1 K for a free-floating BESTEST zone), a heating demand deviates by more than 10 %, or a sensitivity ratio leaves
  0.8 to 1.25, with the deviations understood listed as BESTEST does.
* FastModel gates in the BESTEST validation as Buildings and IDEAS do (`pytest -m bestest`, the workflow
  `bestest.yaml`): its 27 cases inside their bands, the direction of every change between two cases as the reference
  programs, and frozen values (`expected/FastModel.json`) that catch the drifts inside the bands.
