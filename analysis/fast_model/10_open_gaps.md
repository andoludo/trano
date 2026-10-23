# 10. What is still open, and how to close it within rumoca and CasADi

Ranked by what they change in the results of this analysis. Each proposal keeps FastModel an explicit ODE without
events and uses only what rumoca 0.10.3 accepts (array sizes and subscripts from literal parameters, no loop in a
function over a tunable parameter); the gate is the check that would close it. The same list, with more detail, is
in [`docs/validation/fast_model_improvement_plan.md`](../../docs/validation/fast_model_improvement_plan.md#proposals-for-the-remaining-gaps).

| Gap | Evidence | Proposal | Cost | Gate |
|---|---|---|---|---|
| **Heat pump at full load** | renovated house: supply water 0.4 K cooler than in Buildings while the living room valve is open; set point +1 K: heating 0.87, electricity 0.85 | the condenser outlet and the capacity from the EN 14511 table of the heat pump (a polynomial fit in the source and condenser temperatures, evaluated in Modelica) instead of the buffer temperature | parameters only | supply within 0.2 K at full load, set point responses within 5 % |
| **Heat pump safety times** | the two perturbations that *differ*: buffer x3 (FastModel's peak −5 %, Buildings' −1 %) and EPS x0.5 (peak ratio 1.51) | a first-order lag on the compressor command, `tau*der(y) = yOn − y`, `tau` a third of the minimum run time: the compressor cannot start and stop faster than the safety times allow, on average, without an event | one state per heat pump | buffer and peak ratios 0.8 to 1.2 |
| **Sun on the opaque surfaces** | solar absorptance 0.6 → 0.9: ratios 0.91 to 0.93 on every KPI of 950FF and 940 | find where the heat goes: compare the outer surface temperatures and the conduction into the room of a sun-lit wall in both libraries; the irradiance, the outside convection, the sky exchange and the discretisation are ruled out, the room-side long-wave exchange of the opaque surfaces (radiant star against Buildings' radiosity) is the next suspect | none until the cause is known | ratios 0.95 to 1.05 |
| **Sun kept in the zones nothing cools** | free-floating peaks of the light BESTEST zones 1.0 to 1.7 K above Buildings (680FF, 600FF, 650FF), sun-space 0.42 K warm on average | likely the same cause as the previous line | | free-floating peaks within 1 K |
| **Unbalanced ventilation** | supply air 0.3 K warm | the extract of a ventilation unit equal to its supply minus the air the infiltration paths push out, as the pressures of Buildings share it | none | supply air within 0.1 K |
| **Wind- and stack-driven infiltration** | IDEAS and Buildings options FastModel does not have | `m = rho*A_L*sqrt(C_s*sqrt(dT^2 + d^2) + C_w*v^2)` (LBL), explicit in the weather and the zone temperature | parameters only | infiltration responses within 10 % against IDEAS |
| **Ground (ISO 13370)** | the floors on ground of IDEAS | a virtual layer of soil with its periodic temperature, as IDEAS | two states per floor | ground losses within 10 % of IDEAS |
| **Not modelled** | | storage tanks on boilers, water-to-water heat pumps, chillers, domestic hot water, batteries, EV chargers, VAV, moisture | per feature | |

What is **not** a gap any more: the 27 BESTEST cases (shading included), the free-floating temperatures on average
(0.38 K), the set point response of the radiator branches, the MPC export of both residential houses, the state names
shared by the closed loop and the MPC model, and the gas of the boiler house per unit of heat (the same as Buildings
within 0.2 %, [the boiler water loop](09_boiler_water_loop.md)).
