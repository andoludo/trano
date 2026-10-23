# FastModel improvement plan

What to change in the `FastModel` library so that it stays within the bands of Buildings on the zone temperatures,
the heating power and the heating demand **and** responds to a parameter change as Buildings does. The plan follows
from the [study of BESTEST 950FF, 940, 960 and the renovated house](fast_model_study.md) and from a reading of the
equations of `trano/library/FastModel.mo` against the Buildings components they stand for. Each item names the gap,
the evidence, the change, its cost in states or in smoothness (FastModel must stay an explicit, event-free ODE for
rumoca and CasADi) and the validation that closes it.

The study ranks the gaps by what they change. In Denver, where the air is thin and the BESTEST walls are heavy, FastModel
heats 7 to 16 % more than Buildings and responds 12 to 30 % more to the infiltration; in Uccle, with the renovated
house, it stays within 1 % of the meters and 5 % of the sensitivities, except where thermostatic valves share the
flow of the circuit (set point response 0.80). Two defects were fixed during the study: the night ventilation that
FastModel dropped and the 1 kW leak of its ideal systems.

| Priority | Change | Evidence in the study | Cost | Closed when |
|---|---|---|---|---|
| 1 | Air density from the site pressure | 940: about 140 of the +186 kWh; infiltration responses 1.12 to 1.30 in Denver, 0.95 in Uccle | one expression, no state | 940 heating within 5 % of Buildings, infiltration ratios 0.9 to 1.1 |
| 2 | Discretise the layers (`number_of_states`) | 940: night heating 53 against 3 kWh, reheat peak +12 %; mass responses 1.13 to 1.45; sun-space +0.53 K | states: 4 to about 12 per heavy wall | 940 peak within 5 %, 950FF mass ratios 0.9 to 1.1, sun-space bias within 0.2 K |
| 3 | Quadratic flow laws on the radiator branches | house, set point +1 K: response 0.80, bedrooms +10 to 20 % of heat; radiators x0.7: 1.41 | an algebraic loop in the circuit (closed loop only) | set point and radiator ratios 0.9 to 1.1, bedroom NMBE within 5 % |
| 4 | Long-wave exchange around the current surface temperatures | sun-space: 0.87 K RMSE, highest temperature +0.4 K | none (moving linearisation) | sun-space RMSE below 0.5 K |
| 5 | Exact smooth limits, heat pump safety times | ideal systems leaked 1 kW (fixed by rescaling); −11 W overshoot; buffer and supply peak responses 1.33 to 4.5 | one state per heat pump | no saturated controller output above 1e-6 of its range; peak ratios 0.8 to 1.2 |
| 6 | Shading, unbalanced ventilation, wind- and stack-driven infiltration, ISO 13370 ground | BESTEST 610, 630, 910, 930 not runnable; supply air 0.3 K warm | per feature | the four shading cases inside the BESTEST bands |

The [study](fast_model_study.md) is the yardstick: each change is merged with its run of
`python -m validation.fast_model` (the workflow `fast_model_study.yaml`) and the BESTEST run of FastModel, and the
tables of the study and of the [BESTEST page](bestest.md) are rendered from the new results.

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

Pressure-driven flows with resistances linearised at their nominal flows, a small resistance for the components
that have none in Buildings. The sharing of flow between radiator branches when thermostatic valves close differs
from the quadratic laws of Buildings. Change: the quadratic flow law of `Utilities.quadraticFlow` on the radiator
branches too (already used by the boiler), keeping the regularisation. Cost: nonlinear algebraic loop of the circuit.

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

Overhangs and side fins (BESTEST 610, 630, 910, 930), storage tanks on boilers, water-to-water heat pumps, chillers,
domestic hot water, batteries, EV chargers, VAV, air handling units other than system D, power sensors, moisture,
wind- and stack-driven infiltration, the ISO 13370 ground of IDEAS. The night ventilation schedule was missing
and is added by this branch.

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

5. **Flow sharing between radiator branches.** Pressure-driven flows with the quadratic laws of Buildings
   (`Utilities.quadraticFlow`, already used by the boiler) on the valves and the radiators, with the valve authority of
   the YAML, in the closed-loop model; the MPC model keeps its set flows. *Gate:* renovated house, set point +1 K and
   radiators x0.7 ratios 0.9 to 1.1 (today 0.80 and 1.41), the bedroom heat within 5 % NMBE in every variant.
6. **Heat pump safety times and condenser.** Minimum on and off times as a smooth timer state gating the compressor,
   and the condenser outlet from the EN 14511 table rather than the buffer temperature. *Gate:* the water
   temperatures of the [replay validation](fast_model.md) within 1 K (today about 2 K in closed loop), buffer and
   supply peak ratios 0.8 to 1.2.

### Phase 4: coverage

7. **Overhangs and side fins** (`Buildings.HeatTransfer.Windows.Overhang` and `SideFins` geometry, a smooth shaded
   fraction of the direct irradiance), so that BESTEST 610, 630, 910 and 930 run with FastModel.
8. **Unbalanced ventilation** (extract = supply minus the air pushed out by the infiltration paths), **wind- and
   stack-driven infiltration**, the **ISO 13370 ground** of IDEAS for the comparisons with IDEAS.

### Gates kept after the plan

* The study runs on every change of `trano/library/FastModel.mo` or of the FastModel templates (the workflow does so
  when the commit carries `[study]`); a test reads its committed results and fails when a zone RMSE exceeds 0.5 K
  (1 K for a free-floating BESTEST zone), a heating demand deviates by more than 10 %, or a sensitivity ratio leaves
  0.8 to 1.25, with the deviations understood listed as BESTEST does.
* FastModel joins the BESTEST validation as an informative library, with frozen values (`expected/FastModel.json`)
  that catch regressions, and becomes gating once phases 1 and 2 bring it inside the bands.
