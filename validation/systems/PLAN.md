# Systems validation: plan

Goal: show, with simulations, that the HVAC systems trano generates behave as real ones do (heat pumps,
gas boilers, valves and pumps, ventilation), fix what does not, extend the heat pump catalogue and the
ventilation systems, and demonstrate each family on a combined YAML simulated with Buildings, IDEAS and
FastSmoothBuildings. Everything measured lands in `validation/systems` (cases, KPIs, expectations, report)
and in `docs/validation/systems.md`, the way the BESTEST and performance campaigns are organised.

## What exists today (findings of the code review before any simulation)

| System | Buildings / IDEAS (Trano package) | FastSmoothBuildings | Gaps seen in the code |
|---|---|---|---|
| Air-water heat pump (`air_water_heat_pump`) | `Buildings.Fluid.HeatPumps.ModularReversible.AirToWaterTableData2D`, Vitocal 251-A08 (EN 14511) scaled to `nominal_heating_power`; heating curve; buffer charged in a `dTHys` band; PI on the compressor speed; safety control (min on/off); electricity = compressor + condenser pump | `FastSmoothBuildings.Hydronics.HeatPump`, same Vitocal table, `COP` variable, `EEle` meter | one performance record only; no COP on the data bus; the heating season is fixed by `TOutHeaLim` |
| Water-water heat pump (`water_water_heat_pump`) | `EquationFitReversible` with the EnergyPlus record, source at a constant `TSouSet`, runs at `TSet` whatever the demand | unsupported | no electricity, no energy meters, no control, no buffer: it cannot be assessed as it is |
| Gas boiler with storage (`default`) | `BoilerPolynomial` + `StratifiedEnhanced` tank + boiler pump, state-graph control on the tank temperatures | `FastSmoothBuildings.Hydronics.Boiler` + its control | UA loss counted twice by Buildings 13 (documented); efficiency constant 0.9 by default |
| Gas boiler without storage (`without_storage`) | `BoilerPolynomial`, P controller on the supply temperature, fires only on demand | idem | no modulation limit; condensing behaviour only through the `QuadraticLinear` curve |
| Two-way valve + emission control | `TwoWayEqualPercentage` (10 kPa valve, 5 kPa fixed, 0.06 kg/s, `linearized=true`), P/PI/on-off controls on the zone temperature | `Hydronics.Valve` + smooth controllers | static checks exist (`validation/hydronics.py`: authority, supply temperatures, radiator fit); no simulated evidence |
| Pump + collector control, three-way mixing valve | `SpeedControlled_y` pump with a PI on the valve openings; mixing valve PI on a temperature sensor | idem | the mixing circuits are covered by `test_hydronic_physics.py`; the pump electricity is not checked |
| Ventilation `system_d` | balanced fans at constant pressure, constant-effectiveness recovery (`eps` 0.8), fan power and recovered heat on the bus | `Ventilation.HeatRecoveryUnit` (fan power from dp, m, eta 0.5; fan heat on the supply) | no summer bypass, no frost protection, no post-heater, no schedule: always on at the design flow |
| AHU `default` (Guideline 36 multizone) + VAV boxes | economizer dampers, supply and return fans, G36 controller; **no heating or cooling coil in the model** | unsupported | supplies unconditioned outdoor/mixed air; two of its simulation tests are check-only ("fails for reasons under investigation") |
| AHU `test` | constant-effectiveness unit with the outdoor side at a fixed 22 degC | - | a test stub, not a system |

Only Buildings, IDEAS and FastSmoothBuildings are in scope (the reduced-order zones are informational).

## Method

* One campaign package, `validation/systems`, modelled on `validation/bestest` and `validation/performance`:
  `cases/*.yaml` (small single-zone or two-zone buildings that isolate one effect, plus the combined
  houses), `runner.py` (generate, simulate with each library, cache by the hash of the generated model,
  run in parallel), `kpi.py` (the signals and KPIs of each system family read from the result file),
  `expected.yaml` (the bands every KPI must fall in, with the physical reasoning next to each), `report.py`
  (Markdown tables and Plotly figures to `docs/validation/systems.md`), `python -m validation.systems`, and
  a `pytest -m systems` gate.
* Every case is written once in YAML and simulated with Buildings, IDEAS and FastSmoothBuildings (where the
  library supports the system; the report says where it does not).
* Simulations run with OpenModelica 1.26.9 (the CI version) with the libraries of CI (Buildings 13.0.0,
  IDEAS 3.0.0, Modelica 4.1.0). No Docker in this session: `omc` is installed from conda-forge and the
  libraries cloned from GitHub, and the runs use `TRANO_SIMULATION_RUNTIME=local`; the campaign supports
  both runtimes, so CI keeps using the container.
* Periods: a January week for the winter behaviour, a March or October week for part load, a July week for the
  summer behaviour; the combined houses run a heating-season month. Hourly output.

## Phases

### Phase 0: runtime and campaign skeleton (done first, blocks the rest)

1. Local OpenModelica runtime (installed) and a smoke test with the three libraries.
2. `validation/systems`: case registry, runner with cache and workers, KPI reader, expectations, report, CLI,
   pytest gate; unit tests on the KPI code with the small fixtures of `tests/data/kpi`.

Deliverable: `python -m validation.systems run --library Buildings heat_pump_cop_sweep` works end to end.

### Phase 1: heat pumps

Study (air-water, with Buildings, IDEAS and FastSmoothBuildings):

1. **COP against the source temperature.** `heat_pump_source_outdoor_air: false` and a sweep of
   `temperature_heatpump_source` (-10, -7, 2, 7, 12 degC) at a fixed supply set point: the measured
   QCon/PEle must follow the EN 14511 table of the Vitocal 251-A08 (A-7/W35 about 2.7, A2/W35 about 3.6,
   A7/W35 about 4.4, A12/W35 above 5) within the scaling and the part-load effects.
2. **COP against the supply temperature.** 35, 45, 55 degC set points at the same source: the COP must fall
   with the supply temperature as the table does.
3. **Electricity accounting.** `power` = compressor + condenser pump; `electric_energy` is its integral; the
   heat delivered equals the condenser heat; the seasonal COP of the winter week lies in 2.5 to 4 for radiators
   at 45 degC and above 3.5 for 35 degC.
4. **Dynamics.** Cycling (number of starts a day, minimum on and off times respected), buffer temperature
   within its band, compressor speed between `yComMin` and 1, no heating above the heating limit, the
   heating curve followed (supply set point against the outdoor temperature).
5. **Libraries.** Buildings and IDEAS share the component so they must agree closely; FastSmoothBuildings
   is a different model (no safety control, smooth table) and must agree on the energies within a stated band.
6. The water-water variant as it stands: documented as not assessable (no electricity), then replaced (below).

Fixes expected from the review: `COP` (instantaneous and from the meters) on the data bus; the water-water
variant rebuilt on the same architecture as the air-water one.

Catalogue extension (the YAML stays one `boiler` entry, selected by `variant` and a new `heat_pump_data`
parameter):

* `air_water_heat_pump`: `heat_pump_data` selects the performance record among the EN 14511 records of
  Buildings 13 (`Vitocal251A08`, the default; `SingleSplitRXM20R`) or a **generic table** given in the YAML
  (source temperatures, condenser temperatures, heating capacity and electrical power matrices) so that any
  manufacturer data sheet can be entered; `carnot` as a data-free alternative: `CarnotWithLosses` with a
  Carnot effectiveness, for early studies without a data sheet.
* `ground_source_heat_pump` (new variant): brine-water heat pump on the same buffer and control as the
  air-water one, the source side a ground loop at a constant or seasonally varying ground temperature
  (`WAMAK_WaterToWater_220kW` record for large units, the generic table otherwise). The existing
  `water_water_heat_pump` becomes this architecture with a water source at `temperature_heatpump_source`.
* FastSmoothBuildings: its `HeatPump` is bound to the Vitocal table inside the FastSmoothBuildings library
  (another repository); trano's templates will pass the parameters the library accepts, and the report lists
  what the library would need (a replaceable table package) for the other records.

### Phase 2: gas boilers

1. **Efficiency.** `heat_energy / fuel_energy` over the week against the efficiency curve (constant 0.9 by
   default): the ratio must equal the curve minus the standing losses; with `QuadraticLinear` coefficients of a
   condensing boiler it must rise at low water temperatures.
2. **Gas volume.** `gas_usage` (m3) consistent with `fuel_energy` and the heating value of the fuel record.
3. **Control.** Supply temperature reaches its set point; the boiler with storage cycles on the tank
   temperatures (count the starts, check the hysteresis); no firing without demand; the tank losses to the room.
4. **Libraries.** Buildings and IDEAS agree; FastSmoothBuildings within a band (it counts the UA loss once).

Fixes as found (candidates: the double UA loss default, a modulation minimum, the efficiency output on the bus).

### Phase 3: valves, pumps and mixing circuits

1. **Set point tracking.** A radiator with its valve and each emission control variant (P, PI/G36, on-off):
   zone temperature within a band of the set point during occupancy, the setback at night, no chattering
   (count the valve reversals).
2. **Valve characteristic.** Flow against opening for the equal-percentage valve, the effect of
   `linearized=true` (the default) against `false`, the authority at the design point.
3. **Pump.** Electricity of the pump against its nominal point, pump off when all valves are closed
   (collector control), pressure control.
4. **Mixing valve.** The supply set point held by the three-way valve under a high boiler temperature
   (already in `test_hydronic_physics.py`; reused as a case here).

### Phase 4: ventilation

Study (`system_d`, then the G36 AHU):

1. **Does it blow cold air?** Supply temperature against the outdoor and extract temperatures: at -8 degC
   outdoor and 21 degC inside, `eps` 0.8 gives about 15 degC (plus the fan heat), which is what a heat
   recovery unit without a post-heater does; the report shows the supply temperature distribution and the
   zone temperature with and without the ventilation.
2. **Energy.** The heating energy with the ventilation minus without equals the ventilation loss
   (1 - eps) m cp (Tin - Tout) integrated; the recovered heat is on the bus; the fan electricity gives a
   specific fan power in a realistic range (about 1 000 to 2 000 W per m3/s for the two fans).
3. **Summer.** Without a bypass the unit recovers heat in summer too (warmer supply air): quantified.
4. **G36 AHU.** The supply air is unconditioned (no coils): quantified; the two check-only simulations are
   investigated and fixed or documented.

Extension (new `AirHandlingUnit` variants and parameters, for Buildings/IDEAS; FastSmoothBuildings where its
library allows):

* `system_d` gains a **summer bypass** (recovery off above an outdoor temperature when the room is warmer than
  a set point), a **fan schedule or speed** (reduced flow at night) and an optional **electric post-heater**
  holding a minimum supply temperature, all off by default so that the existing models do not change;
* `system_c` (new variant): mechanical extract only, the outdoor air entering the rooms untreated at the
  outdoor temperature, one extract fan with its electricity;
* `system_a`: no fans, natural supply and extract at a given air change rate (a boundary rather than a unit),
  to compare with the mechanical systems.

### Phase 5: combined demonstrators

One YAML per family on the same house (the six-zone house of `examples/residential_after_renovation.yaml`
or the smaller `house_infiltration_*` house, whichever simulates fast enough), each simulated with
Buildings, IDEAS and FastSmoothBuildings, a heating-season month:

| YAML | Production | Distribution and emission | Ventilation |
|---|---|---|---|
| `house_air_water_heat_pump.yaml` | air-water heat pump, buffer, heating curve | pump, two mixing circuits, radiators rated at 45/35 with P valves | `system_d` with bypass and night setback |
| `house_ground_source_heat_pump.yaml` | ground-source heat pump | direct circuit, low-temperature radiators | `system_d` |
| `house_condensing_boiler.yaml` | boiler without storage, condensing curve | mixing circuit, radiators 70/50 | `system_c` extract only |
| `house_boiler_storage.yaml` | boiler with storage tank | collector pump, radiators with on-off valves | `system_a` |
| `house_heat_pump_pv.yaml` | air-water heat pump + PV | as the first | `system_d` |

For each: the KPI table per library (heating energy per m2, electricity or gas, seasonal COP or efficiency,
fan energy, comfort hours below the set point, supply temperatures) with the bands that make them realistic,
and figures of a trace week. Where a library does not support a system (FastSmoothBuildings and the VAV
AHU or the ground-source heat pump until its library gains it) the table says so.

### Phase 6: documentation, tests and CI

* `docs/validation/systems.md` rendered from the results (`python -m validation.systems report --docs`),
  linked in `mkdocs.yml`; the parameter and component references regenerated for the new variants and
  parameters; the tutorials updated where a new variant is worth showing.
* Unit tests of the campaign (KPI extraction on small fixtures, the expectations file, the case registry),
  golden models of the new variants, the `simulate` marker for the short cases, the `systems` marker for the
  campaign gate; a `systems.yaml` workflow on schedule and on demand, with the library and result caches of
  the BESTEST workflow.
* A commit per phase on `claude/trano-model-validation-yaml-2adefc`.

## Findings (coldest week of the Uccle weather unless said otherwise, `python -m validation.systems report`)

### Heat pumps

* **The air-water heat pump of Buildings and IDEAS behaves as its data sheet.** Over a sweep of constant source
  temperatures (-10, -7, 2, 7, 12 degC) at a flat 45 degC supply, the COP while running is 0.95 to 0.99 of the
  Vitocal 251-A08 table at the measured condenser outlet (49 degC) and source temperatures; the 5 % gap is the
  start-up transient of the refrigerant cycle and the condenser pump. The COP falls with the supply temperature
  (4.1 at 35 degC, 3.4 at 45 degC, 2.8 at 55 degC at a 7 degC source) and rises with the source as the table does.
* **Electricity and heat are accounted for**: the meters on the bus integrate compressor plus condenser pump; the
  seasonal COP of the coldest week is 2.83 (Buildings), 2.84 (IDEAS), 2.90 (FastSmoothBuildings) on radiators at
  45/35 degC with the heating curve, 4.3 to 4.5 in a mild March week; in July the heat pump does not run (IDEAS
  starts it once for 8 minutes when the outdoor air dips below the 16 degC heating limit).
* **The safety control holds**: no run shorter than 5 minutes, no stop shorter than 5 minutes, the compressor never
  below its minimum speed (Buildings, IDEAS); the buffer stays in its band. Without a buffer (a 60 l hydraulic
  separator) the starts rise from 14 to 22 a day at the same seasonal COP.
* **The same heat pump given as a table of the YAML reproduces the library record** (seasonal COP 2.82 against
  2.83). A Carnot model at the default effectiveness of 0.45 under-estimates the Vitocal by 23 % in the coldest
  week (2.19): 0.45 is a conservative value that keeps a data-free study on the safe side; 0.58 would match this
  unit. The ground-source heat pump on a 10 degC ground loop reaches 3.1, the water-water one on a 12 degC loop 3.5.
* **Fixed**: the water-water heat pump had no electricity, no energy meters and ran at a fixed set point whatever the
  demand; it now shares the architecture of the air-water one. The COP (instantaneous and seasonal) and the source
  temperature are on the data bus. FastSmoothBuildings ignored `heat_pump_source_outdoor_air` and
  `temperature_heatpump_source` silently; it now reports them as unsupported.
* **Sizing lesson kept in the cases**: with the linear heating curve (45 degC at -8 degC, 30 degC at 16 degC) a
  radiator rated exactly for the design loss leaves the room short on cold mornings after the night setback; the
  cases oversize the radiator by a third, as a heat pump retrofit does.

### Gas boilers

* **The boiler with storage** burns 207 kWh for 182 kWh of heat in the coldest week (efficiency 0.88: the 0.9 curve
  minus the standing losses of the boiler, counted twice by Buildings 13, and of the 200 l tank), 3.3 starts a day;
  the fuel volume follows the heating value of the fuel record, 11.67 kWh/m3 (50 MJ/kg at 0.84 kg/m3). The tank top
  reaches 85 degC for a 70 degC set point: the control charges until the tank bottom passes the set point plus 5 K.
* **The boiler without storage** modulates continuously under its proportional controller (the water holds 70.0
  degC, 96 to 104 firing hours a week, 2 starts a day), efficiency 0.875. **The condensing curve** (0.97 at 40 degC
  to 0.90 at 70 degC) on 45/35 radiators gives 0.958 while firing and 0.94 over the week.
* **Finding, not changed**: the default (storage) control charges the tank whenever the outdoor air is below its 15
  degC cut-off, with or without heat demand: 33 kWh of gas in a July week for a zone at 23 degC, the standing losses
  of a tank kept at 75 to 85 degC. A lower `threshold_outdoor_air_cutoff` or a demand condition would stop it; the
  choice is left to the user (see the decisions below).
* **Fixed in FastSmoothBuildings**: its boilers under the default and complex controls had no demand and idled at 80
  degC all year (16 kWh of gas in a July week); they now fire on demand and hold the set point of the tank control.

### Valves, pumps and mixing circuits

* The radiator valve holds the zone within 0.7 K RMSE of its set point under the proportional controller, 1.2 K under
  the Guideline 36 loop (slower, 6 movements a day) and 0.55 K under the on-off thermostat (43 to 53 movements a day);
  the valve closes 60 to 90 hours a week (the night setback), the pump never runs without demand, and the
  linearised flow law (trano's default) changes the opening (0.52 against 0.38) but not the comfort. The pumps draw
  little (0.1 to 0.15 kWh a week at 1.5 W): the preconfigured mover at 30 kPa and 0.07 kg/s.
* The mixing valve holds its supply set point within 1.4 to 1.9 K (PI) with the tank at 85 degC.
* **Finding, validator**: a radiator valve returning directly to the generator (`inlets: [VALVE]` on the boiler)
  passes the validation but does not compile (the containers connect the production to the distribution only); a
  model named like a container (`ventilation`, `emission`, ...) does not compile either. Both are left for a
  validation rule (see the decisions).
* **Fixed in FastSmoothBuildings**: the three-way valve controller of the library is proportional; the PI gain of the
  YAML (0.1) left a 7 K steady-state error and cold rooms (19.1 degC); the gain of the YAML is now only taken for a P
  controller, and the rooms follow Buildings within 0.2 K.

### Ventilation

* **Does it blow cold air?** Yes, by a few kelvin, as a real unit without a post-heater: at -1 degC outdoor and 21 degC
  inside, the balanced unit with 80 % recovery supplies 16.2 degC on average (12.8 degC at the coldest hour), the
  measured effectiveness being 0.84 with the heat of the fans. Without recovery the supply is the outdoor air (-0.7
  degC), as with the extract-only system and the natural ventilation.
* **The energy is accounted for**: the heating of the zone rises by 26 kWh with the recovery unit (the computed loss of
  the supply air deficit is 28 kWh), by 160 kWh without recovery, by 166 kWh with the extract-only system and the
  natural ventilation (0.05 kg/s x 1006 J/(kg.K) x 22 K x 168 h = 186 kWh, less the hours of setback). The fans draw
  43 W (1 030 W per m3/s for the two fans at 200 Pa, the range of real units), 9 W for the extract fan alone; the
  three libraries agree within 5 %.
* **Summer**: without a bypass the unit warms the supply above the room 97 hours of the July week; with the bypass
  (new) it opens 75 hours and the supply rises above the room 68 hours only, the remaining ones being those when the
  outdoor air itself is warmer than the room. The post-heater (new) holds the supply at 18.0 degC for 15 kWh of
  electricity a week and takes 10 kWh off the zone heating. The fan schedule (new) at half flow at night saves a
  quarter of the fan electricity.
* **The Guideline 36 unit** of trano (variant `default`, with a VAV box and its controller at their defaults) does
  not ventilate at all in the week simulated (`ahu_g36_winter`): its fans stay off (0 kg/s, 0 W), the zone heating
  equals the case without ventilation. The controller waits for an occupancy mode and zone requests that the
  single-zone configuration never raises, and the unit has no heating or cooling coil to condition the air it
  would supply. Making it a usable system (coils, a hot water source, a working operating mode) is a design
  choice left to the user; the cases keep it as a documented limitation.
