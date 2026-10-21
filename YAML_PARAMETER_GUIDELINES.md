# Guidelines: which YAML parameters can be changed without recompiling

These guidelines tell, field by field, which parameters of a trano YAML specification can be changed **after the
model is compiled** (OpenModelica executable, library **Buildings**), how to change them, and which ones need a
new compilation. They are written for calibration and parametric studies, where one compilation is followed by
many simulations.

They are based on the measured inventory of `examples/residential_after_renovation.yaml` (298 numeric fields,
Buildings 13.0.0, OpenModelica 1.26.9): every field that reaches the executable was read back and simulated over
two January weeks, and 59 of them were also changed in the YAML, recompiled and compared with the run-time change.
Details and evidence: [`RUNTIME_PARAMETERS.md`](RUNTIME_PARAMETERS.md); one row per field:
[`parameter_inventory_buildings.csv`](parameter_inventory_buildings.csv).

## 1. How to change a parameter

Compile once, then change **YAML fields**, not Modelica names: trano translates each YAML change into every
run-time parameter it reaches (a material reaches a dozen records, `n50` becomes an infiltration rate, ...).

```bash
trano compile-model house.yaml Buildings --end 1382400           # once: keeps house_runtime/build
trano run-model house_runtime/build \
    --change "material[EPS:001].thermal_conductivity=0.033" \
    --change "spaces[LIVING:001].parameters.n50=2.5" \
    --stop 1382400 --label trial_001
```

```python
from pathlib import Path
from trano.runtime import CompiledModel, ModelRunner
from trano.runtime.resolution import SpecificationResolver

compiled = CompiledModel.load(Path("house_runtime/build"))
resolver = SpecificationResolver.from_file(Path("house.yaml"), "Buildings")
with ModelRunner(compiled) as runner:
    for i, trial in enumerate(trials):  # trial: {"material[EPS:001].thermal_conductivity": 0.033, ...}
        overrides = resolver.overrides(trial, compiled)  # raises NotRuntimeChangeableError for a NO field
        run = runner.run(overrides, label=f"trial_{i:03d}", stop_time=16 * 86400, strict=True)
```

Rules that apply to every field:

- **Address fields by their YAML path.** List items are selected by `id` (`material[EPS:001]`,
  `systems[GENERATOR:001]`) or by index (`external_walls[0]`).
- **Pass the compiled model to `resolver.overrides(changes, compiled)`** (the CLI does): changes reaching a
  parameter fixed at compilation (a NO field, or a material property outside its range) raise
  `NotRuntimeChangeableError` instead of producing overrides that would be ignored.
- **Use `strict=True` (the CLI default).** The run fails if the executable refuses an override instead of
  silently simulating the old value; `trano run-model --no-strict` sets the free part of a refused change and
  warns.
- **Stay inside the admissible range** of material properties and layer thicknesses (section 5).
- **Anything not listed as changeable needs a recompilation**: generate the model again from the changed YAML.
- **For another YAML**, check it with `trano parameter-inventory house.yaml Buildings --select "<regex>"`: the
  verdicts below hold for the same element types and variants; a new element type or variant must be checked.

### 1.1 Check that the parameters were really applied

Every run checks itself: the executable prints, at the end of the simulation, the value it used for each overridden
parameter (`-output=<names>`, kept in `runs/<label>.log`), and trano compares them with the values set. With
`strict=True` (the default of the CLI) a run fails when an override was refused or another value was used.

```bash
trano run-model house_runtime/build --change "spaces[LIVING:001].parameters.n50=2.5" --label trial_001
# ✔ 1 overrides applied: the simulation used every value set

trano check-run house_runtime/build trial_001     # later, on a finished run: exit code 1 if not applied
```

```python
run = runner.run(overrides, label="trial_001")          # strict=True raises OverrideNotAppliedError
print(run.verification.summary())                        # or inspect .refused, .differs, .missing
verification = verify_run(compiled, "trial_001")         # from runs/trial_001.overrides.txt and .log
```

A refused override is reported twice: as refused by the executable, and as used with another value
(`set 0.5, used 0.610865` for the PV tilt). These values are the ones of the simulation, including the copies of
the construction records inside each zone, which are not stored in the result file (Buildings marks them
HideResult).

What the check cannot see: a parameter the compilation used to size arrays takes the new value and prints it,
but the model keeps its compiled structure (the tank's `nSeg`, a material outside its admissible range). Those are
the NO and range rows of this guide, and `resolver.overrides(changes, compiled)` refuses them before the run.
To confirm that a change also has an effect, compare an output with a run without overrides.

## 2. Verdicts at a glance

| verdict | meaning | what to do |
|---|---|---|
| **YES** | the run-time parameter takes the YAML value | `--change` the YAML field |
| **YES, derived** | trano transforms the value (e.g. `n50` -> infiltration rate); the transformed values are changeable | `--change` the YAML field; trano computes the run-time values |
| **YES, within range** | changeable as long as the layer discretisation of the compiled model stays valid | `--change` inside the range of section 5; recompile outside it |
| **NO** | the value is fixed in the executable | change the YAML and recompile (or use the workaround given) |
| **NOT USED** | Buildings does not use the field | nothing to change |
| **NO EFFECT here** | changeable, but it does not act in the example's scenario | changeable; may act in another scenario |

## 3. Recommended parameters by calibration goal

Which YAML fields to change for the usual calibration targets, all changeable on the compiled model. The U-values
below are ISO 6946 estimates from the layers of the example (surface resistances included); Buildings computes the
heat transfer itself, the estimates only show how far each lever goes.

### 3.1 U-value of the walls, roof and floor

The insulation layer dominates each construction, and in the example each insulation material is used by one
construction only, so it acts on that construction alone:

| construction (U today) | insulation fields | U reachable with one field | with conductivity and thickness together |
|---|---|---|---|
| external walls (0.23 W/m2K) | `material[EPS:001].thermal_conductivity`, `constructions[EXTERNAL_WALL:001].layers[0].thickness` | 0.07 to 0.24 (conductivity), 0.13 to 0.23 (thickness) | any value |
| roof (0.15 W/m2K) | `material[MINERAL_WOOL:001].thermal_conductivity`, `constructions[ROOF:001].layers[0].thickness` | 0.13 to 0.43 (conductivity), 0.14 to 0.26 (thickness) | any value |
| floor on ground (0.27 W/m2K) | `material[PUR:001].thermal_conductivity`, `constructions[GROUND_FLOOR:001].layers[1].thickness` | 0.08 to 0.29 (conductivity), 0.15 to 0.28 (thickness) | any value |

- **One field at a time** stays within the ranges of section 5, which are narrow on the side of a *worse* wall:
  EPS conductivity can rise by 7 % at most, the EPS layer can get thinner by 3 % at most.
- **To go further, change the conductivity and the thickness of the insulation together, keeping
  `thickness / sqrt(conductivity)` constant.** The layer keeps its discretisation (verified on the compiled model:
  EPS conductivity 0.045 alone is refused, 0.045 with a thickness of 0.12 * sqrt(0.045 / 0.032) = 0.142 m is
  accepted), and its thermal resistance becomes `R0 * sqrt(k0 / k)`, so any U-value can be reached. The heat
  capacity of the layer changes with its thickness, which is negligible for light insulation (20 to 35 kg/m3).

  ```bash
  trano run-model house_runtime/build \
      --change "material[EPS:001].thermal_conductivity=0.045" \
      --change "constructions[EXTERNAL_WALL:001].layers[0].thickness=0.142302"
  ```

- **Thermal bridges and workmanship** are usually calibrated as a U-value correction: use the same insulation
  fields (there is no separate thermal-bridge parameter in the Buildings model).
- **The ground** acts through `floor_on_grounds[*].ground_temperature` too (+2 K: heat pump -3 to -4 %), the
  temperature under the slab; calibrate it together with the floor insulation, the two compensate each other.
- **Do not use** the masonry or finishing materials for a U-value: they shift the heat capacity much more than the
  U-value, and the same materials are used in several constructions (`BRICK_INNER:001` in walls and partitions,
  `PLASTER:001` everywhere).

### 3.2 Windows: U-value, g-value, frame

All windows of the example use `glazings[WINDOW:001]` and the same glass materials, so these fields act on every
window of the house. For windows calibrated separately, give them separate glazings or glass materials before
compiling.

| target | fields | notes |
|---|---|---|
| U-value of the glazing (Ug) | `gas[ARGON:001].thermal_conductivity`; the emissivity of the low-e coatings, `glass_material[GLASS_LOWE_ROOM_SIDE:001].infrared_absorptivity_room_facing` and `glass_material[GLASS_LOWE_OUTSIDE_SIDE:001].infrared_absorptivity_outside_facing` (0.04); the gap widths `glazings[WINDOW:001].layers[1].thickness`, `layers[3].thickness` | argon conductivity +20 %: heat pump +2.3 %. The coating emissivity drives the radiative exchange across the gaps: 0.04 -> 0.10 multiplies it by about 2.5 (long-wave exchange between a coated and an uncoated pane). |
| U-value of the frame (Uf) | `glazings[WINDOW:001].u_value_frame` | +20 %: heat pump +0.9 %. |
| frame share (Uw and solar gains) | `windows[*].frame_fraction` per window | |
| g-value (solar gains) | `glass_material[*].solar_transmittance`, `solar_reflectance_*` | keep transmittance + reflectance <= 1. |
| window size | `windows[*].surface` | the window becomes `sqrt(surface)` wide and high; its orientation cannot change. |

### 3.3 Infiltration and ventilation

| target | fields | notes |
|---|---|---|
| infiltration (airtightness) | `spaces[*].parameters.n50`, per zone | trano sets `ACH = n50 / 20`; the most direct calibration parameter for air leakage. A space given `ach` instead takes it directly. +20 % in the living room: heat pump +3.7 %. |
| mechanical ventilation flows | `ventilation_inlets[*].duct.parameters.nominal_mass_flow_rate`, `ventilation_outlets[*]...` per duct | the flows are driven by the fans' pressure: the nominal flow of a duct changes its resistance, so the flows of all ducts redistribute. The AHU's own `m_flow_nominal` and every `dp_nominal` are fixed. |
| heat recovery | `systems[AHU:001].air_handling_unit.parameters.heat_exchanger_effectiveness` | the strongest single lever of the example (0.8 -> 0.64: heat pump +11.9 %). |
| air volume | `spaces[*].parameters.floor_area`, `average_room_height` | change the volume and with it the infiltration flow (`ACH * volume`). |

### 3.4 Heating system and controls

| target | fields | notes |
|---|---|---|
| comfort temperature | `...emission_control.parameters.temperature_heating_setpoint` per zone | +2 K: heat pump +1.5 to +5.3 %. |
| occupation and heating periods | `...emission_control.parameters.schedule`, `spaces[*].occupancy.parameters.occupancy` | shift the times; the number of entries is fixed. |
| night setback | `...temperature_heating_setback` | changeable; acts only when the rooms cool down to it. |
| heat pump size | `systems[GENERATOR:001].boiler.parameters.nominal_heating_power` | trano also sets the loop flows and the pump curve. |
| supply temperature (heating curve level) | `systems[GENERATOR:001].boiler.parameters.temperature_supply_setpoint_heat_pump` | +2 K: heat -1.2 %, electricity +1.0 %. |
| storage | `tank_volume`, `height_of_tank_without_insulation` | the number of tank segments is fixed. |
| radiator heat output | not through the YAML field: `--set building.emission1.radiator_<zone>_001.Q_flow_nominal=...` (and `vol[1].V`) | the radiator's nominal power and temperatures are fixed in the executable; section 4.10 explains how to represent another radiator. |
| radiator valve authority | `...valve.parameters.m_flow_nominal` | |
| distribution | `systems[THREE_WAY_VALVE:001].three_way_valve.parameters.m_flow_nominal` | the pump's nominal flow is fixed. |

Not changeable for the system: radiator nominal power and temperatures (through the YAML), pump nominal flow, AHU
nominal flow and pressures, duct pressure drops, tank segments, PV tilt and azimuth.

## 4. Field by field

### 4.1 `material` (opaque materials)

| field | verdict | notes |
|---|---|---|
| `thermal_conductivity` | YES, within range | reaches the envelope record and the copy in every zone using the material, with Buildings' `R = x/k`, `piMat`, `nStaReal` (e.g. `EPS:001`: 12 records). Overriding `k` alone in Modelica does nothing: the conduction uses `R`. |
| `density` | YES, within range | same copies; changes the heat capacity of the layers. |
| `specific_heat_capacity` | YES, within range | same copies. |
| `longwave_emissivity`, `shortwave_emissivity` | YES (expected) | not set in the example; they reach `absIR_a/b` and `absSol_a/b` of the first and last layers of each construction (free parameters). Check with the inventory tool before relying on them. |

The admissible ranges of every material of the example are in section 5.

Measured effect over two January weeks (heat pump heat, +20 % or up to the range limit): mineral wool
conductivity +2.7 %, EPS +1.2 % (limited to +6 % by its range), PUR -0.8 %; masonry and finishes below 1 %.

### 4.2 `gas` (glazing gaps)

| field | verdict | notes |
|---|---|---|
| `thermal_conductivity` | YES | reaches `a_k` of every glazing gap using the gas, in every zone. Argon +20 %: heat pump +2.3 %. |
| `density` | YES, derived | gives the molar mass `MM` (ideal gas at 20 degC). If the new density makes another standard gas the closest one, the viscosity coefficients `a_mu`, `b_mu` change too; trano computes them. |
| `specific_heat_capacity` | YES | reaches `a_c`. |
| `longwave_emissivity`, `shortwave_emissivity` | NOT USED | Buildings' gas record has no emissivity. |
| any field of a gas no glazing uses | NOT USED | in the example `AIR:001` is defined but no glazing has an air gap. |

### 4.3 `glass_material` (panes)

| field | verdict | notes |
|---|---|---|
| `thermal_conductivity` | YES | |
| `solar_transmittance` | YES | per pane, in every zone; low-e pane -20 %: heat pump -0.9 % or +2.7 % depending on the pane. |
| `solar_reflectance_outside_facing`, `solar_reflectance_room_facing` | YES | |
| `infrared_transmissivity` | YES | |
| `infrared_absorptivity_outside_facing`, `infrared_absorptivity_room_facing` | YES | |
| `density`, `specific_heat_capacity` | NOT USED | Buildings models panes without heat capacity. |

Keep the optical properties physical (transmittance + reflectance <= 1, transmissivity + absorptivity <= 1);
the executable does not check them.

### 4.4 `constructions`

| field | verdict | notes |
|---|---|---|
| `layers[i].thickness` | YES, within range | reaches every copy of the layer (`x`, `R`, `piMat`, `nStaReal`); range in section 5. EPS of the external wall 12 -> 14.4 cm: heat pump -7.3 %. |
| `layers[i].material` (which material) | NO | changes the record: recompile. To calibrate a layer, change the material's properties. |
| number or order of layers | NO | structural: recompile. |

A material used in several constructions changes in all of them; to change a layer of one construction only,
give it its own material in the YAML before compiling.

### 4.5 `glazings`

| field | verdict | notes |
|---|---|---|
| `u_value_frame` | YES | +20 %: heat pump +0.9 %. |
| `layers[i].thickness` (panes and gaps) | YES | no discretisation in the glazing model, so no range. |
| `layers[i].glass` / `.gas` (which material), number of layers | NO | recompile. |

### 4.6 `spaces[*].parameters`

| field | verdict | notes |
|---|---|---|
| `n50` | YES, derived | gives the infiltration `ACH = n50 / 20` (`n50_to_ach`) of the `infiltration` variant. +20 % in the living room: heat pump +3.7 %. |
| `floor_area` | YES | the air volume (`AFlo * hRoo`) and the infiltration flow follow inside the model. |
| `average_room_height` | YES | same. |
| `ach` (when given instead of `n50`) | YES (expected) | the same run-time parameter as `n50`. |
| `variant` | NO | structural. |

### 4.7 `spaces[*].occupancy`

| field | verdict | notes |
|---|---|---|
| `parameters.occupancy` (`3600*{...}` schedule) | YES | the number of entries must stay the same (it is an array size): shift times, do not add or remove entries. Keep the times inside the day (0 to 86400 s). |
| `parameters.gain`, `heat_gain_if_occupied` (`k`) | YES (expected) | not set in the example; free parameters of the occupancy component. |

### 4.8 `spaces[*].external_boundaries`

| field | verdict | notes |
|---|---|---|
| `external_walls[*].surface` | YES | gross facade area (windows included). |
| `external_walls[*].azimuth`, wall **without** window | YES | e.g. a roof or a blind gable. |
| `external_walls[*].azimuth`, wall **hosting** a window | NO | trano requires every window on a wall facing the same way; turning the wall alone is rejected. Turn the wall and its windows together and recompile. |
| `windows[*].surface` | YES, derived | gives the width and height `sqrt(surface)` of a square window (or uses the given width or height). +20 % in the living room: heat pump -2.1 % (more solar gain). |
| `windows[*].frame_fraction` | YES | |
| `windows[*].azimuth` | NO | same rule as the host wall. |
| `windows[*].width`, `height`, `overhang`, `side_fins` | YES (expected) | not in the example; they reach `wWin`, `hWin`, `ove.*`, `sidFin.*` (free). Adding or removing a shading device is structural. |
| `adiabatic_walls[*].surface` | YES | |
| `adiabatic_walls[*].azimuth` | NO EFFECT | changeable, but Buildings computes no solar radiation on boundary constructions. |
| `floor_on_grounds[*].surface` | YES | |
| `floor_on_grounds[*].ground_temperature` | YES | the boundary temperature under the slab. +2 K: heat pump -2.9 % (kitchen) to -4.4 % (living). |
| `floor_on_grounds[*].perimeter`, `average_indoor_temperature` | NOT USED | IDEAS only (ISO 13370 ground model). |
| `tilt` (`wall`, `ceiling`, `floor`), `construction`, adding or removing a boundary | NO | structural. |

### 4.9 `internal_walls`

| field | verdict | notes |
|---|---|---|
| `surface` | YES | area of the partition between two zones. |
| `construction`, `space_1`, `space_2`, tilts | NO | structural. |

### 4.10 `spaces[*].emissions` - radiator

| field | verdict | notes |
|---|---|---|
| `nominal_heating_power_positive_for_heating` | NO (partly changeable) | `--change` refuses it (mass and water volume fixed); see below for setting the heat output. |
| `water_inlet_temperature_at_nominal_condition` | NO | evaluated at compilation (nominal flow and start values of the radiator). |
| `water_outlet_temperature_at_nominal_condition` | NO | same. |
| `air_temperature_at_nominal_condition` | NO | same. |

**Radiator size.** trano writes the radiator's dry mass `mDry = 0.0263 Q` and water volume `VWat = 5.8e-6 Q` from
the nominal power. Both are fixed in the executable (they feed a structural flag of Buildings' volume,
`computeCSen`, with `Evaluate=true`). The nominal power itself, `Q_flow_nominal`, is changeable and sets the
heat-transfer coefficient, i.e. the heat output of the radiator. A changed power therefore gives the new heat output
with the thermal inertia of the compiled radiator; compared with a recompiled model the difference is about half of
the effect (NRMSD 0.14 against 0.30). The water volume of the radiator can be set too:

```bash
trano run-model house_runtime/build --strict \
  --set building.emission1.radiator_living_001.Q_flow_nominal=4680 \
  --set "building.emission1.radiator_living_001.vol[1].V=0.027144"     # 5.8e-6 * 4680
```

Only the dry-mass part of the heat capacity then stays compiled (this combination was not verified by a
recompile). Do not remove `mDry` and `VWat` from trano's template to make them follow the power: the structural
flag would then freeze the power as well.

**Nominal temperatures.** They cannot change, but a radiator rated differently can be represented through the power.
With one element (trano's default `nEle = 1`), Buildings sizes the radiator from the outlet temperature:
`Q_flow_nominal = UA * (T_b_nominal - TAir_nominal)^n`, `n = 1.24`. A radiator rated `Q_new` at outlet temperature
`T_b_new` and air temperature `TAir_new` has the same heat output as the compiled radiator (65 degC outlet, 20 degC
air) with

```
Q_flow_nominal = Q_new * ((65 - 20) / (T_b_new - TAir_new)) ^ 1.24
```

The nominal water flow derived from the rated temperatures stays compiled; in operation the valve sets the flow.

### 4.11 `spaces[*].emissions` - valve and emission control

| field | verdict | notes |
|---|---|---|
| `valve.parameters.m_flow_nominal` | YES | +20 %: heat pump -0.2 to -1.4 %. |
| `control.emission_control.parameters.temperature_heating_setpoint` | YES | +2 K: heat pump +1.5 to +5.3 % (-0.6 % for the bathroom). |
| `control.emission_control.parameters.temperature_heating_setback` | YES, NO EFFECT here | in two January weeks the well insulated rooms never cool down to the setback, so it does not act; it does in a colder period or with a lower setback. |
| `control.emission_control.parameters.schedule` (`3600*{...}`) | YES | same number of entries, times within the day. |
| `control.emission_control.variant` | NO | structural. |

### 4.12 `spaces[*].ventilation_inlets` / `ventilation_outlets` - ducts

| field | verdict | notes |
|---|---|---|
| `nominal_mass_flow_rate` | YES | the flows are pressure-driven: a larger nominal flow at the same nominal pressure drop is a smaller resistance, and the flows of all ducts redistribute. |
| `dp_nominal` | NO | Buildings' `PressureDrop` evaluates it at compilation (`computeFlowResistance`, `Evaluate=true`). To change a resistance at run time, change `nominal_mass_flow_rate` instead. |

### 4.13 `systems` - air-water heat pump (`boiler`, variant `air_water_heat_pump`)

| field | verdict | notes |
|---|---|---|
| `nominal_heating_power` | YES, derived | also sets the nominal flows of the radiator and heat pump loops (`1.5 Q / dT / 4200`) and the pump curve `V_flow`. |
| `temperature_supply_setpoint_heat_pump` | YES | the supply temperature set point (40 degC in the example). +2 K: heat -1.2 %, electricity +1.0 %. |
| `tank_volume` | YES | |
| `height_of_tank_without_insulation` | YES | |
| `number_of_volume_segments` (`nSeg`) | NO | an array size. **The executable accepts an override without any warning** but cannot change the number of segments; never pass it as a run-time parameter. |
| `use_storage_tank`, `variant` | NO | structural. |

Boiler-control parameters (`threshold_outdoor_air_cutoff`, `TSup_nominal`) and the PI parameters of the collector
and three-way valve controls are written into generated classes and are not reachable as run-time parameters by
name; recompile to change them (not set in the example).

### 4.14 `systems` - pump, three-way valve, sensors, split valve

| field | verdict | notes |
|---|---|---|
| `pump.parameters.m_flow_nominal` | NO | defines the pump's pressure curve, evaluated with the mover's structural flags. |
| `three_way_valve.parameters.m_flow_nominal` | YES | |
| split valve, temperature sensor | - | no numeric field in the example. |

### 4.15 `systems` - air handling unit (`air_handling_unit`, variant `system_d`)

| field | verdict | notes |
|---|---|---|
| `heat_exchanger_effectiveness` | YES | the strongest single lever of the example: 0.8 -> 0.64 gives heat pump +11.9 %. |
| `m_flow_nominal` | NO | defines the fans' pressure curves and the heat exchanger's resistance (structural flags). Change the duct flows instead. |
| `dp_nominal`, `supply_dp_nominal`, `return_dp_nominal` | NO | same reason. |

### 4.16 `solar` - photovoltaic

| field | verdict | notes |
|---|---|---|
| `area` | YES | PV energy changes in proportion (+20 % -> +20.00 %). |
| `efficiency` | YES | same. |
| `tilt`, `azimuth` | NO | `Evaluate=true` in Buildings' oriented PV model; the executable reports the override as refused. |

### 4.17 Always structural (recompile)

- element ids, `variant` of any element, which construction, glazing or material an element uses;
- adding or removing spaces, walls, windows, emissions, systems, layers, schedule entries;
- tilts given as `wall`, `ceiling`, `floor`; the topology (`inlets`, `outlets`, `internal_walls` pairs);
- `weather.parameters.path` (the weather file is read by the executable, but trano writes its path at generation;
  regenerate the model to use another file);
- the library (Buildings, IDEAS, ...).

## 5. Admissible ranges of material properties and layer thicknesses

Buildings divides each wall layer into a number of states `nSta = max(1, ceil(nStaRef * x * sqrt(c * d / k) / 331.4))`
that is fixed at compilation. A property or thickness can change at run time only while every layer using it keeps
its number of states: within the ranges below (lower bound excluded, upper bound included). Outside them, the run
still works but no longer matches the specification; recompile. `0` and `inf` mean the side is unbounded. A
calibration should use these as bounds. The ranges change with the compiled values: recompute them for another
YAML with the inventory tool (`reason` column).

| material | thermal_conductivity | density | specific_heat_capacity |
|---|---|---|---|
| `EPS:001` (0.032 / 20 / 1450) | (0.0086, **0.0342**] | (**18.7**, 74.8] | (**1356**, 5424] |
| `MINERAL_WOOL:001` (0.035 / 30 / 1030) | (**0.0306**, 0.123] | (8.6, **34.3**] | (294, **1177**] |
| `PUR:001` (0.023 / 35 / 1400) | (0.0064, **0.0257**] | (**31.3**, 125] | (**1253**, 5012] |
| `BRICK_FACING:001` (0.9 / 1700 / 840) | (0.237, **0.948**] | (**1614**, 6457] | (**798**, 3190] |
| `BRICK_INNER:001` (0.7 / 1600 / 840) | (0.540, 0.892] | (1255, 2075] | (659, 1090] |
| `CONCRETE:001` (1.7 / 2300 / 1000) | (0.679, 2.71] | (1441, 5762] | (626, 2505] |
| `SCREED:001` (1.4 / 2000 / 1000) | (0.590, inf) | (0, 4746] | (0, 2373] |
| `TILE:001` | (0.016, inf) | (0, 1.9e5] | (0, 6.9e4] |
| `TIMBER:001` | (0.032, inf) | (0, 2049] | (0, 6555] |
| `PLASTER:001` | (0.024, inf) | (0, 2.7e4] | (0, 2.1e4] |
| `GYPSUM_BOARD:001` | (0.017, inf) | (0, 1.36e4] | (0, 1.51e4] |
| `CAVITY_AIR_5CM:001` | (0.00025, inf) | (0, 1349] | (0, 1.1e6] |
| `JOIST_AIR_10CM:001` | (0.00099, inf) | (0, 758] | (0, 6.4e5] |

Bold: a bound within 10 % of the current value; the insulations in particular have little room, e.g. EPS
conductivity can rise by 7 % at most and mineral wool conductivity can fall by 12 % at most.

| layer (m) | range | | layer (m) | range |
|---|---|---|---|---|
| `EXTERNAL_WALL:001` EPS 0.12 | (**0.116**, 0.232] | | `ROOF:001` mineral wool 0.22 | (0.118, **0.235**] |
| `EXTERNAL_WALL:001` facing brick 0.09 | (**0.0877**, 0.175] | | `ROOF:001` timber 0.022 | (0, 0.0445] |
| `EXTERNAL_WALL:001` cavity 0.05 | (0, 1.68] | | `ROOF:001` air 0.10 | (0, 2.51] |
| `EXTERNAL_WALL:001` inner brick 0.14 | (0.0797, **0.159**] | | `ROOF:001` gypsum 0.015 | (0, 0.0582] |
| `EXTERNAL_WALL:001` plaster 0.015 | (0, 0.0685] | | `GROUND_FLOOR:001` concrete 0.12 | (0.0950, 0.190] |
| `PARTY_WALL:001` brick 0.095 | (0.0797, 0.159] | | `GROUND_FLOOR:001` PUR 0.08 | (**0.0757**, 0.151] |
| `INTERNAL_WALL:001` brick 0.09 | (**0.0797**, 0.159] | | `GROUND_FLOOR:001` screed 0.06 | (0, 0.0924] |
| `INTERNAL_WALL:001` plaster 0.01 | (0, 0.0685] | | `GROUND_FLOOR:001` tile 0.01 | (0, 0.0906] |
| `INTERMEDIATE_FLOOR:001` timber, air, gypsum | (0, 0.0445], (0, 2.51], (0, 0.0582] | | `PARTY_WALL:001` plaster 0.015 | (0, 0.0685] |

A material and the thicknesses of its layers act together (`x * sqrt(c d / k)`): when several of them change in
the same run, the combination must stay inside the range. `SpecificationResolver.overrides(changes, compiled)` and
`trano run-model --change` refuse a set of YAML changes that alters the discretisation (`NotRuntimeChangeableError`),
so a calibration can catch it.

## 6. Practical advice for calibration

1. **Pick parameters from the YES rows.** The strongest levers of the example (heat pump energy over two January
   weeks) are the heat recovery effectiveness, the setpoints, the ground temperature, the insulation thicknesses
   and conductivities, `n50` and the glazing properties.
2. **Bound material properties and thicknesses with section 5**, or recompile when the optimiser leaves the range.
3. **Use a smooth objective.** The heat pump cycles on and off: its two-week energy moves by one or two percent
   for any small change, in either direction. Zone temperatures, longer periods or daily energies are better
   targets than two weeks of heat pump energy.
4. **Keep `strict=True`** so that a refused or unused override stops the run; `trano check-run` re-checks a finished
   run from its files (section 1.1).
5. **Parameters marked NO** are fixed for the compiled model: decide them before compiling (or compile one model
   per value, e.g. per radiator rating).
6. **Another house, another variant**: run `trano parameter-inventory <yaml> Buildings` (or with `--select` for the
   fields you want to calibrate) to get the same verdicts for that model.
