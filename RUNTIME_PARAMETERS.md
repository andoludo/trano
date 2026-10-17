# Run-time parameters of the compiled Buildings models

For calibration, a trano model is compiled once with OpenModelica and simulated many times with new
parameter values. This page tells which fields of the YAML specification can really be changed at run time
in the compiled executable, how this was established, and why the others cannot. The case is
`examples/residential_after_renovation.yaml` with Buildings 13.0.0 in OpenModelica 1.26.9 (the image trano
simulates with); the full inventory is [`parameter_inventory_buildings.csv`](parameter_inventory_buildings.csv).

## The workflow

```bash
trano compile-model examples/residential_after_renovation.yaml Buildings --end 1382400   # once
trano run-model examples/residential_after_renovation_runtime/build \
    --change "material[EPS:001].thermal_conductivity=0.033" \
    --change "spaces[LIVING:001].parameters.n50=2.5" --stop 1382400 --label trial_1
trano parameter-inventory examples/residential_after_renovation.yaml Buildings          # this study
```

or from Python (`trano.runtime`):

```python
resolver = SpecificationResolver.from_file(Path("house.yaml"), "Buildings")
compiled = build_model(project, network, SimulationOptions(end_time=16 * 86400))     # buildModel, kept
with ModelRunner(compiled) as runner:                                                # one container
    for trial in trials:                                                             # YAML-level changes
        run = runner.run(resolver.overrides(trial), label=..., stop_time=16 * 86400, strict=True)
```

- `build_model` runs `buildModel` in trano's OpenModelica container (`trano.simulate.simulate`: same
  image, same library loading, same weather handling) and keeps the build directory: executable,
  `<Model>_init.xml`, `<Model>_info.json`. A variable filter keeps the outputs and every parameter set
  from the specification in the result files.
- `ModelRunner.run` runs the executable in the container with `-overrideFile` (one `name=value` per line)
  and the experiment flags `-startTime`, `-stopTime`, `-stepSize`, `-tolerance`, `-variableFilter`.
  OpenModelica 1.26 ignores `-override=stopTime=...` ("override variable name not found in model"),
  hence the flags. The executable's log is parsed: an override reported as *not possible to override* or
  *not found* fails the run with `strict=True` and never passes as a change.
- `SpecificationResolver.overrides` turns YAML changes into run-time overrides by converting the
  specification with and without the change and comparing the parameters trano writes (see below): the
  overrides include every copy and every transformation, and a change no override can represent (a
  structural one) raises.
- OMC flags: none (`OPENMODELICA_FLAGS = ()`), as in trano's regular simulations. Flags such as
  `-d=evaluateAllParameters` or `--evaluateProtectedParameters` would evaluate parameters at translation
  and must not be added. With the defaults OpenModelica still evaluates the structural parameters
  (`Evaluate=true`, array sizes, `final` bindings) and what they depend on: those are the frozen
  parameters below.
- Smoke test: the executable run without overrides over four January days (two of warm-up) is
  bit-identical to trano's regular `simulate` of the same model (NRMSD 0 on every output); rooms at
  18-20 degC, heat pump 514 W heat for 140 W electricity on average.

## How a YAML field reaches the executable

The chain YAML field -> Modelica parameter -> run-time name is resolved by difference, not by a hand-written
table: trano converts the specification as given and with the field perturbed, and the Buildings adapter
(`trano.runtime.adapters.buildings`) lists the scalar parameters trano writes, with their values. The
parameters whose values differ are the field's fan-out with the values an override must set. The element
parameters come from `processed_parameters`, i.e. from `data_models/parameters.yaml`, the source of truth
the parameter reference is generated from; the adapter adds what the Buildings templates write as records
and arrays (constructions, glazings, zone boundaries). This snapshot matches the start values of the
executable's `<Model>_init.xml` exactly (2005 of 2023 names; the other 18 are control parameters baked into
generated classes, which this YAML does not set). Comparing the generated Modelica text as well tells a field
Buildings does not use (the model does not change) from a gap of the adapter (none found).

Python transformations found this way (status `derived` when the run-time values differ from the YAML value):

| YAML field | run-time parameter(s) a calibration sets | transformation |
|---|---|---|
| `spaces[*].parameters.n50` | `<zone>.ACH` | `n50 / n50_to_ach` (20) |
| radiator `nominal_heating_power_positive_for_heating` | `Q_flow_nominal`, `mDry`, `VWat` | `mDry = 0.0263 Q`, `VWat = 5.8e-6 Q` |
| heat pump `nominal_heating_power` | `Q_flow_nominal`, `nominal_mass_flow_radiator_loop`, `nominal_mass_flow_rate_boiler`, `V_flow[1:2]` | `1.5 Q / dT / 4200`, `V_flow = m_flow / 1000 * {0.5, 1}` |
| window `surface` | `<zone>.datConExtWin[j].wWin`, `hWin` | `sqrt(surface)` |
| PV `tilt`, `azimuth` | `til`, `azi` | degrees to radians |
| gas `density` | `glaSys.gas[i].MM` (and `a_mu`, `b_mu` if another standard gas gets closer) | ideal gas at 20 degC |
| material `thermal_conductivity`, `density`, `specific_heat_capacity`, layer `thickness` | every copy of `material[i].k/d/c/x` and `R = x/k`, `piMat`, `nStaReal` | Buildings' `Material` record, evaluated into each zone (below) |
| `occupancy`, `schedule` (`3600*{...}`) | `occupancy[i]`, `schedule[i]` | arrays scalarised |

A material fans out widely: OpenModelica copies the construction records into every zone that refers to them
as **free parameters with literal values** (`living_001.datConExtWin[1].layers.material[1].k` does not follow
`envelope1.external_wall_001.material[1].k`), including the derived `R = x/k` the conduction model actually
uses. Overriding `k` alone changes nothing; `EPS:001` conductivity reaches 12 records (1 envelope record and
11 zone copies), each with `k`, `R`, `piMat` and `nStaReal`. The equivalence check is what proves none is
missed: the start values of the recompiled model's init file are compared with the baseline's, and every free
parameter that changes must be overridden.

## Summary

298 numeric fields of the example, by group and status (`parameter_inventory_buildings.csv` has one row per
field with its Modelica paths, run-time names, init-file attributes, check results and reason):

| group | changeable | derived | frozen | no_impact | not_used | missing | fields |
|---|---|---|---|---|---|---|---|
| Ground | 2 |  |  |  | 4 |  | 6 |
| Opaque materials |  | 39 |  |  |  |  | 39 |
| Layers |  | 21 |  |  |  |  | 21 |
| Glazing | 39 | 1 |  |  | 13 |  | 53 |
| PV | 2 |  | 2 |  |  |  | 4 |
| Geometry | 35 | 10 | 20 | 3 |  |  | 68 |
| Space | 18 | 6 |  |  |  |  | 24 |
| Controls | 12 |  |  | 6 |  |  | 18 |
| Emission | 6 |  | 24 |  |  |  | 30 |
| Heat pump | 3 | 1 |  |  |  |  | 4 |
| Hydronics | 1 |  | 1 |  |  |  | 2 |
| Ventilation | 13 |  | 16 |  |  |  | 29 |
| **all** | **131** | **78** | **63** | **9** | **17** | **0** | **298** |

- **changeable** (131): the run-time parameters take the YAML value itself.
- **derived** (78): trano transforms the value; the run-time parameters to set and the transformation are in
  the `reason` column (`n50` -> `ACH`, materials -> every copy of `k`, `R`, `piMat`, `nStaReal`, window
  surface -> `wWin`, `hWin`, heat pump power -> its nominal flows). `SpecificationResolver.overrides` computes
  them from YAML changes.
- **frozen** (63): radiator nominal power and temperatures, duct and AHU pressure drops and AHU and pump
  nominal flows, PV tilt and azimuth, orientation of walls hosting windows and of windows.
- **no_impact** (9): the six heating setbacks, inert in this scenario (below), and the azimuths of the three
  adiabatic party walls (a boundary construction receives no solar radiation in Buildings).
- **not_used** (17): `perimeter` and `average_indoor_temperature` of the floors (IDEAS only; Buildings takes
  `ground_temperature`), the properties of `AIR:001` (no glazing of the example has an air gap), the
  emissivities of the gases, the density and heat capacity of the glass panes (Buildings' glass record has none).
- **missing** (0).

Calibration targets of the brief that pass all three checks on the field itself: insulation conductivities (within the admissible
range, below), `n50`, `ground_temperature`, glass `solar_transmittance`, `u_value_frame`, heating setpoints,
heat pump `nominal_heating_power` and `temperature_supply_setpoint_heat_pump`, `heat_exchanger_effectiveness`
and the duct flows. The calibration targets that are **not** run-time parameters: the radiator
`nominal_heating_power_positive_for_heating` and the AHU `m_flow_nominal` (and every pressure drop).

## The checks

A field is changeable only if it passes the three checks on the same compiled executable (16-day experiment
from the 1st of January: two days of warm-up and two January weeks with heating, setback and heat recovery
active; outputs every 15 minutes; tolerance 1e-4, trano's default):

1. **Readback**: a one-hour run with the perturbed values (`-ignoreHideResult -emit_protected`: the zone copies
   of the records are marked HideResult) reads every run-time parameter back from the result file; each equals
   the value set, the parameters bound to them in the parameter equations of `_info.json` change, and the log
   reports none as not applied. Fields sharing no run-time parameter are checked together, 25 per run.
2. **Impact**: the 16-day run with the perturbed values against the baseline on the outputs of the brief (air
   temperature of the six zones, heat pump heat and electricity, the six radiator heat flows, AHU supply
   temperature, PV power). Metric: per output, the RMS difference after the warm-up divided by the range of the
   baseline output (NRMSD); a field has an impact when the largest NRMSD reaches 1e-3. The relative change of
   the energy of each power is reported too; it is the better measure of magnitude, the NRMSD of the cycling
   heat pump being large even for small changes.
3. **Equivalence**: the field changed in the YAML, the model regenerated, recompiled and simulated without
   overrides; its outputs must equal the override run within NRMSD 1e-6, and every free parameter whose start
   value differs between the two init files must be among the overrides.

Perturbations: +20 % by default, -20 % for quantities bounded by one, +2 K for temperatures (-2 K for setbacks),
half an hour for schedules (their midnight ends kept), +0.2 rad for an azimuth at zero; for material properties
and layer thicknesses, inside the admissible range when +20 % leaves it.

**Coverage.** Readback and impact ran for every field reaching free parameters (224 fields; 225 16-day runs
with the baseline, about four minutes each). Equivalence ran on 59 fields: every high-priority field of the brief
(insulation conductivities, glazing transmittances and frame, `n50`, ground temperatures, setpoints and setbacks,
heat pump power and supply setpoint, heat recovery, duct flows), one field for every other *mechanism* (its group,
direct or derived, and the reasons some of its run-time names are not free), and floor area, room height,
schedules, tank volume and height, brick conductivity, PV efficiency and area; plus the exploratory recompile of
EPS `k` +20 % that showed the range limit. The remaining 165 fields (repeated surfaces, the other materials and
layers, the other optical properties, valves, ...) carry `equivalence_ok = inferred_true` or `inferred_false`
with the verified field of the same mechanism they were inferred from: 59 x (2.5 min compilation + 4 min run)
already took two and a half hours, the other 165 would take seven more (`trano parameter-inventory` with an
`extra_equivalence` list of every path does it).

### Equivalence results (59 recompiles and the exploratory one)

58 of the 59 are equivalent, and every equivalent case is exact: the outputs of the recompiled model and of the
override run are bit-identical (NRMSD 0.0), and no free parameter was missed. By group: Controls 13, Ventilation 13,
Space 9, Glazing 5, Opaque materials 5, Heat pump 4, Emission 2, Ground 2, Geometry 2, PV 2, Layers 1, Hydronics 1
(every row of the CSV with `equivalence_ok = true`). The table gives one field per mechanism:

| field (perturbed value) | equivalent | note |
|---|---|---|
| `material[EPS:001].thermal_conductivity` (0.0340) | yes | 12 records x (`k`, `R`, `piMat`, `nStaReal`); evaluated construction `R` unused |
| `material[BRICK_INNER:001].thermal_conductivity` (0.84) | yes | |
| `material[BRICK_FACING:001].density` (2040) | yes | |
| `constructions[EXTERNAL_WALL:001].layers[0].thickness` (0.144) | yes | |
| `glass_material[GLASS:001].solar_transmittance` ([0.664]) | yes | evaluated envelope record unused |
| `glazings[WINDOW:001].u_value_frame` (1.32) | yes | |
| `gas[ARGON:001].density` (2.04) | yes | through `MM` of the zone copies |
| `spaces[LIVING:001].parameters.n50` (3.6) | yes | `ACH` = 0.18 |
| `spaces[LIVING:001].parameters.floor_area` (48) | yes | `V` and the infiltration flow follow (bound) |
| `spaces[LIVING:001].parameters.average_room_height` (3.12) | yes | |
| `spaces[LIVING:001].occupancy.parameters.occupancy` | yes | |
| `...floor_on_grounds[0].ground_temperature` (288.65) | yes | |
| `...external_walls[0].surface` (23.09) | yes | |
| `...windows[0].surface` (7.8) | yes | `wWin`, `hWin` |
| `...emission_control.parameters.temperature_heating_setpoint` (296.15) | yes | |
| `...emission_control.parameters.temperature_heating_setback` (287.15) | yes | no impact either way |
| `...emission_control.parameters.schedule` | yes | |
| `...valve.parameters.m_flow_nominal` (0.1116) | yes | |
| `...ventilation_inlets[...].duct.parameters.nominal_mass_flow_rate` (0.016) | yes | |
| `systems[GENERATOR:001]...nominal_heating_power` (3600) | yes | with the 4 derived flows |
| `...temperature_supply_setpoint_heat_pump` (315.15) | yes | |
| `...tank_volume` (0.12), `...height_of_tank_without_insulation` (0.96) | yes | |
| `systems[THREE_WAY_VALVE:001]...m_flow_nominal` (0.4164) | yes | |
| `systems[AHU:001]...heat_exchanger_effectiveness` (0.64) | yes | |
| `solar[PV:001]...efficiency` (0.16), `...area` (25.2) | yes | |
| `...radiator.parameters.nominal_heating_power_positive_for_heating` (4680) | **no** | NRMSD 0.14; `mDry`, `VWat` evaluated, `vol[1].V` a literal |
| `material[EPS:001].thermal_conductivity` (0.0384, +20 %) | **no** | `nSta` 2 -> 1: the discretisation changes (exploratory run) |

### Impact (heat pump heat over the two weeks)

Heating setpoint +2 K: +1.5 to +5.3 % (living, kitchen, bedrooms; -0.6 % for the bathroom); heat recovery
0.8 -> 0.64: +11.9 %; EPS layer of the external wall 12 -> 14.4 cm: -7.4 %; ground temperature +2 K: -2.9 %
(kitchen), -4.4 % (living); `n50` +20 %: +3.7 % (living), between -1.9 % and +0.9 % for the other zones; mineral
wool `k` +20 %: +2.7 %; EPS `k` +6 % (range limit): +1.2 %; low-e glass transmittance -20 %: -0.9 % or +2.7 %
depending on the pane; `u_value_frame` +20 %: +0.9 %; heat pump supply setpoint +2 K: heat -1.2 %, electricity
+1.0 %; PV efficiency -20 %: PV energy -20.00 %. Every one of the 224 fields is in the `impact_metric`
column.

The heat pump cycles on and off, so its two-week energy moves by one or two percent for any small perturbation,
in either direction (the infiltration of a single upstairs room, a duct flow): the impact check detects a change
reliably (the NRMSD of an unaffected run is exactly 0), but the sign and size of changes below about 2 % are not
physical sensitivities. A calibration on this model needs a smooth objective (longer periods, zone temperatures,
or a hysteresis on the heat pump control) rather than two weeks of heat pump energy.

The **setbacks** (-2 K) have no impact at all (NRMSD exactly 0): in this well insulated house the rooms do not cool
down to 16 degC during the night in these two weeks, so the P controllers stay at zero either way. The recompiled
model does not change either (equivalence exact for each of the six): inert by design in this scenario, not ignored
by the executable; a colder period or a lower setback would make them act. The azimuths of the adiabatic party
walls are taken by the executable (readback) but do nothing: Buildings computes no solar radiation on boundary
constructions (`datConBou`).

## Freezing mechanisms found

| mechanism | parameters | evidence |
|---|---|---|
| `Evaluate=true` in Buildings | PV `til`, `azi` (`PartialPVOriented`) | calculatedParameter with a literal; the executable logs "not possible to override" |
| structural flag depending on the parameter (`Evaluate=true`, `final`) | duct `dp_nominal` (`PressureDrop.computeFlowResistance`), AHU `m_flow_nominal`, `dp_nominal`, `dpSup_nominal`, `dpRet_nominal` (fan pressure curves, heat exchanger flow resistance), pump `m_flow_nominal` (mover pressure curve) | calculatedParameter with a literal |
| use in nominal sizing and start values | radiator `T_a_nominal`, `T_b_nominal`, `TAir_nominal` (`m_flow_nominal` and the start values of `RadiatorEN442_2`; inlined as `343.15` in `UAEle`'s parameter equation) | calculatedParameter with a literal, `_info.json` |
| Python transformation into evaluated parameters | radiator nominal power: trano writes `mDry = 0.0263 Q` and `VWat = 5.8e-6 Q`, which size the water volumes and the final `mSenFac` | equivalence fails (NRMSD 0.14), `vol[1].V` changes in the recompiled init file |
| array size | `nSta` of every layer (`max(1, ceil(nStaRef x sqrt(c d / k) / 331.4))`), and the construction's `nSta[]` and `R` evaluated with it | equivalence fails when `nSta` changes (EPS `k` +20 %), exact inside the range |
| structure of trano's specification | azimuth of a wall hosting windows or of a window: trano requires each window on a wall facing the same way | the converter rejects the change |
| record evaluated, copies free | `final glass`/`final gas` modifiers of trano's glazing record make `envelope1.window_001.*` calculated; the zones use free copies | equivalence exact: harmless |

Silent cases: the executable takes an override of a free parameter the translation used for a size without any
warning; `nSeg` of the heat pump tank (not set by this YAML) is flagged changeable and is an array size. The log
check alone is not enough; the readback, impact and equivalence checks are what catch such cases.

### Admissible ranges of the material properties

The discretisation is fixed at translation, so a material property or layer thickness can change at run time only
while every layer using it keeps its number of states: `nStaReal` is proportional to `x sqrt(c d / k)`, and each
layer keeps `nSta = N` while `N - 1 < nStaReal <= N`. The range is computed per field
(`BuildingsAdapter.admissible_range`) and given in the `reason` column; for example EPS conductivity
(0.0086, 0.0342], EPS density (18.7, 74.8], `BRICK_INNER:001` density (1255, 2075], the external wall's EPS layer
thickness (0.116, 0.232]. A calibration must bound these parameters accordingly or recompile when leaving the range.

## How far isValueChangeable agrees with the run-time checks

Per field, from the `isValueChangeable` of its run-time names (fields reaching the executable):

| isValueChangeable of the run-time names | taken at run time | frozen |
|---|---|---|
| true | 155 | 0 |
| mixed | 63 | 6 |
| false | 0 | 37 |

- `true` for every name: always taken at run time (155 fields). But `true` alone does not say a change is
  equivalent: the material copies are `true` and only follow the YAML if every copy and `R` are set, and a value
  outside the admissible range changes a size the executable keeps.
- `false` for every name: always frozen (37 fields); the attribute is right, the mechanism is in the table above.
- mixed: the attribute cannot decide. For 63 fields the non-changeable names are leftovers that the model does not
  use (the glazing record of the envelope, the construction's total `R`), the change is exact; for the 6 radiator
  powers they matter and the change is not equivalent. Only the equivalence check tells.
- `causality="calculatedParameter"` with a start value means evaluated at translation, without one bound to other
  parameters and recomputed at initialisation (floor area -> `V`); both refuse overrides with a log warning.

## Changes to trano's Buildings templates that would unfreeze parameters (not implemented)

- **Layer discretisation**: write `nSta=` explicitly for each layer in the construction template (`library.json`)
  instead of letting Buildings derive it from `nStaRef`: `k`, `c`, `d` and `x` would no longer feed a structural
  parameter and the admissible ranges would disappear (the discretisation stays the compiled one).
- **Radiator power**: stop writing `mDry` and `VWat` (Buildings' own defaults are the same formulas of
  `Q_flow_nominal`); they become bindings in the model. Whether OpenModelica then keeps them changeable (`VWat`
  sizes the volumes through `V = VWat/nEle`, `mSenFac` is final) has to be checked with this inventory.
- **Glazing record**: drop the `final` of `glass=` and `gas=` in the glazing template; harmless today (the zones
  use free copies) but it makes `isValueChangeable` agree with the behaviour.
- **Duct pressure drops**: `PressureDrop` evaluates `dp_nominal` through `computeFlowResistance`; a run-time
  changeable resistance needs another component (a flow coefficient given as a parameter the structural flags do
  not depend on), a design change of the ventilation templates.
- **Wall and window orientations**: changing a wall and the windows it hosts together (a facade rotation) is a
  consistent change of the specification that maps to `datConExtWin[j].azi`; the converter could accept it.
- Not fixable in trano's templates: PV `til`/`azi`, radiator nominal temperatures, AHU and pump nominal flows and
  pressures (`Evaluate=true` and structural use inside Buildings).

## Reproduce

```bash
uv run pytest tests/test_runtime.py                 # hermetic: init file, resolution, classification, overrides
uv run pytest -m runtime                            # Docker: smoke test and the parametrised checks (hours)
trano parameter-inventory examples/residential_after_renovation.yaml Buildings
```

The inventory of this page ran in a cloud session whose network policy blocks OpenModelica's package server: the
libraries (Buildings 13.0.0, IDEAS 3.0.0, MSL 4.0.0 and 4.1.0) came from their GitHub tags, mounted with
`TRANO_MODELICA_LIBRARIES`, and `ModelicaServices.ExternalReferences.loadResource` was set to
`OpenModelica.Scripting.uriToFilename` as in OpenModelica's `+maint.om` builds (the weather file is a
`modelica://` resource resolved at run time). With network access, trano installs the `+maint.om` libraries itself.
