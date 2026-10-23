# FastModel improvement plan

What to change in the `FastModel` library so that it stays within the bands of Buildings on the zone temperatures,
the heating power and the heating demand **and** responds to a parameter change as Buildings does. The plan follows
from the [study of BESTEST 950FF, 940, 960 and the renovated house](fast_model_study.md) and from a reading of the
equations of `trano/library/FastModel.mo` against the Buildings components they stand for. Each item names the gap,
the evidence, the change, its cost in states or in smoothness (FastModel must stay an explicit, event-free ODE for
rumoca and CasADi) and the validation that closes it.

<!-- the priorities and the evidence are written from the results of the study -->

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

<!-- priorities, phases and gates written from the results -->
