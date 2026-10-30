# Plan: BOPTEST export for trano models

Goal: a trano YAML model becomes a BOPTEST test case that drops into `testcases/<name>/models/` of a BOPTEST
checkout, either as a Modelica package with its `Resources` folder and `compile_fmu.py` (for users who compile
with JModelica, Optimica or Dymola through BOPTEST's own parser), or as a ready `wrapped.fmu` compiled with
OpenModelica in trano's container, which BOPTEST's worker loads as is.

## 1. Design

### What BOPTEST actually requires

The worker (`testcase.py`) never reads Modelica. It needs an FMI 2.0 co-simulation FMU such that:

- inputs are named `<point>_u` (Real, with `unit`, `min`, `max`, description) and `<point>_activate` (Boolean);
- outputs are named `<point>_y` (Real, with `unit`, description); every overwrite point also has a `<point>_y`;
- `resources/` holds `kpis.json`, `days.json`, `config.json` and CSVs whose columns follow `data/categories.json`,
  zone columns as `LowerSetp[<zone>]`, with `time` in seconds from January 1st;
- `kpis.json` maps `<SignalType>[<zone>]` or `<SignalType>` to lists of `<point>_y` names.

The parser (`parsing/parser.py`) is only the introspection step for hand-written models. Trano holds the network
graph, so it generates the interface and the metadata directly, and the parser is supported as an alternative
compilation path by also placing the signal exchange blocks.

### Principles

1. **One source of truth: the network.** Points, zones, schedules, areas and weather all come from the `Network`,
   never from parsing the generated Modelica or an FMU.
2. **BOPTEST is a variant plus an export target, not a library.** The blocks are added through the existing
   `variant` mechanism; the interface, metadata and packaging live in a new `trano/boptest/` package; the FMU
   export reuses `trano.runtime` and the trano-simulate container.
3. **Library agnostic blocks.** `Overwrite`, `Read` and the weather reads are rendered into the embedded `Trano`
   package under `Trano.Utilities.IO.SignalExchange`, so Buildings, IDEAS, AixLib-based and FastSmoothBuildings
   models all work, and BOPTEST's parser still recognises them (it looks for the protected parameters
   `boptestOverwrite` and `boptestRead` and for an enumeration whose path contains
   `SignalExchange.SignalTypes.SignalsForKPIs`).
4. **Sane defaults, explicit overrides.** `--boptest` with no YAML section exposes the conventional points
   (setpoints and actuators as overwrites; zone temperature, CO2, gas and electric power, weather as reads).
   A `boptest:` section in the YAML narrows or extends them and sets scenario data.
5. **Verify what we ship.** Every exported FMU is loaded with fmpy and stepped the way `TestCase.advance` does
   before it is declared done.

### YAML section (schema: `trano/data_models/trano.yaml`, regenerated into `trano_final.yaml`)

```yaml
boptest:
  name: house_hydronic          # test case id; default: YAML file stem
  step: 900                      # default communication step [s]
  warmup_period: 604800          # default warmup of config.json
  time_periods:                  # days.json; "auto" picks peak/typical heating days from a baseline run
    peak_heat_day: 15
    typical_heat_day: 42
  prices:                        # prices.csv; BOPTEST units, EUR/kWh
    electricity: {constant: 0.27, day: 0.29, night: 0.25, day_start: "07:00", day_end: "22:00"}
    gas: 0.05
  emissions: {electricity: 0.167, gas: 0.181}   # kgCO2/kWh
  points:
    overwrite:
      - element: CONTROL:001     # id of an EmissionControl: its heating set point
        signal: heating_setpoint
        min: 283.15
        max: 303.15
      - element: VALVE:001       # an actuator: the control signal it receives
        signal: actuator
    read: auto                   # or a list of {element, signal, kpi, zone}
```

### New package layout

```
trano/boptest/
  __init__.py
  specification.py   # pydantic models of the boptest: section; defaults derived from a Network
  points.py          # OverwritePoint, ReadPoint, KpiSignal, BOPTEST naming (ove*/rea*/weaSta_rea*)
  interface.py       # renders the `wrapped` model and its kpis.json from the points
  resources.py       # weather.csv (from the .mos), setpoints/occupancy/internalGains (from the network),
                     # prices.csv, emissions.csv (from the section), days.json
  package.py         # config.json; zips Resources + JSON into the FMU (Data_Manager logic, no pyfmi)
  fmu.py             # buildModelFMU in the trano-simulate container; wraps runtime.build_script
  verify.py          # fmpy: causality/naming/units/min-max/resources checks and a short advance loop
  export.py          # orchestration: TestCaseBundle = folder ready for testcases/<name>/models
trano/templates/Trano.jinja2            # + Trano.Utilities.IO.SignalExchange (Overwrite, Read, SignalTypes)
trano/elements/library/models/boptest/  # `boptest` variants of emission_control, boiler_control, ahu_control,
                                        # collector_control, three_way_valve_control, v_a_v_control, data_bus,
                                        # weather, boiler, photovoltaic
```

### Where the blocks go

- **Actuator overwrites** inside the generated control models: between `emissionControl.yHea` and the control's
  `y` output, so both the direct connection to the valve and the bus member `yHea<Valve>` carry the applied
  value. Same for boiler, pump, three-way valve and VAV controls.
- **Setpoint overwrites** between the schedule switch and `emissionControl.THeaSet` (and `TCooSet`).
- **Reads** in the `boptest` variant of `DataServer`: `TRoo[i].T` as `AirZoneTemperature[<space>]`,
  `TRoo1[i].ppm` as `CO2Concentration[<space>]`, the electric power sum as `ElectricPower`; boiler `QFue.y` as
  `GasPower` in the boiler variant; heat pump `power` as `ElectricPower`; PV as negative electric power.
- **Weather reads** in the `boptest` variant of `weather`: the TMY3 reader's bus members, named
  `weaSta_reaWea<Member>` as BOPTEST does.
- **Interface** as a `wrapped` model in the generated package, next to `building`: top-level inputs and outputs
  with BOPTEST names, `building mod(...)` with the `uExt(y=...)`/`activate(y=...)` modifiers (dotted modifiers,
  which OpenModelica and Dymola accept), and `_y` outputs bound to the blocks. `trano create-model --boptest`
  writes the package with `wrapped` as the model to compile.

## 2. Phases

**Phase 0: spike, retire the integration risk first (2-3 days).**
Hand-edit the generated Buildings model of `tests/tutorials/first_model.yaml` with one overwrite, two reads and
a `wrapped` model. Export with OpenModelica in the trano container:
`setCommandLineOptions("--fmiFlags=s:cvode"); buildModelFMU(P.wrapped, version="2.0", fmuType="cs",
platforms={"static"}, includeResources=true)`. Load with fmpy, then run it in a local BOPTEST
(`docker compose up web worker provision`) with the example client loop. Questions to answer: does pyfmi 2.14 in
the worker step the OpenModelica co-simulation FMU; does the weather `.mos` travel inside the FMU
(`loadResource("modelica://P/Resources/...")` with `includeResources=true`, which needs the package written as a
directory package) or does it need embedding; does the CVODE flag change results versus trano's regular
simulation (compare with the golden run, as the runtime smoke test does). Outcome: a short memo in this file.

**Phase 1: signal exchange blocks and points model (2 days).**
Add `Trano.Utilities.IO.SignalExchange.{Overwrite, Read, SignalTypes.SignalsForKPIs}` to `Trano.jinja2`, copied
from IBPSA with the protected marker parameters. Implement `points.py` (frozen dataclasses or pydantic,
`KpiSignal` literal, zone handling, naming helpers) and `specification.py` (section models, `defaults(network)`,
`resolve(network, section)` returning the final point list). Unit tests on naming and kpis.json against the
shipped `testcases/*/models/kpis.json` shapes.

**Phase 2: `boptest` variants (3-4 days).**
One `boptest` variant per control template and for `data_bus`, `weather`, `boiler`, `photovoltaic`, selected
automatically when the export is requested (`Network` gets a `boptest: BoptestSpecification | None`; the
component lookup prefers the `boptest` variant when it exists, the default otherwise). Golden `.mo` tests per
library (`tests/golden.py`), and the existing `compile` marker test so OpenModelica checks every variant.

**Phase 3: interface and metadata (2 days).**
`interface.py` renders `wrapped` and `kpis.json`; `package.py` writes `config.json` (area as the sum of
`floor_area`, step, start time, warmup) and `library_versions.json`. `create-model --boptest` writes the package
plus `Resources/`, `kpis.json`, `config.json`, `days.json` and a `compile_fmu.py` identical in shape to BOPTEST's
so JModelica or Dymola users can use their parser path unchanged.

**Phase 4: resources (3 days).**
`resources.py`: `weather.csv` by simulating a weather-only `Trano` model once in the container and reading the
`.mat` with buildingspy (this gives the solar angles and sky temperature exactly as Buildings computes them, like
BOPTEST's `Data_Generator.generate_weather`); `setpoints.csv` from the `EmissionControl` schedule, `THeaSet` and
`THeaSetBack` per space, `UpperSetp` from `TCooSet` or a default; `occupancy.csv` and `internalGains.csv` from
the occupancy element; `prices.csv` and `emissions.csv` from the section with BOPTEST's constant, day/night and
highly dynamic columns; `days.json` from the section or, with `auto`, from a year-long baseline run using the
peak and typical day search of `find_days.py`. Column validation against a vendored copy of `categories.json`.

**Phase 5: FMU export and packaging (2-3 days).**
`fmu.py` adds `build_fmu_script` next to `runtime.build_script`, reusing `load_statements` and `container`.
`package.py` appends `Resources/*.csv` and the three JSON files under `resources/` of the FMU. `verify.py`
reproduces the worker's checks: FMI version, co-simulation, every `_u` has unit, min and max, every output has
a unit (CO2 excepted), inputs/outputs pair up, the three JSON files load, CSV columns are allowed, and a
twenty-step advance loop with and without `activate` changes the applied value. CLI:
`trano export-boptest model.yaml Buildings --output testcases/house --fmu --verify`.

**Phase 6: schema, CLI, docs, tests (2 days).**
`boptest` section in the LinkML schema and `tests/test_schema.py`; `--boptest` on `create-model`;
`export-boptest` command; tutorial "Exporting a BOPTEST test case" and a reference page for the section;
`boptest` pytest marker for the end-to-end test against a BOPTEST checkout (compose up, select, advance, kpi),
run nightly, not in `make tests`.

**Phase 7 (follow-up, separate PR): light runtime.**
`trano/cosim/StepSimulator` on fmpy (initialize with warmup, advance with held inputs and clipping, results,
forecasts from the same resources DataFrame, KPIs from `trano/kpi.py`), so the exported FMU is also usable
without BOPTEST's service. Optional `boptest-submit` command wrapping the presigned POST of `BoptestSubmit`.

## 3. Risks and acceptance

| Risk | Mitigation |
|---|---|
| OpenModelica co-simulation FMU behaves differently under pyfmi than JModelica's | Phase 0 runs it inside the real worker; keep `--fmiFlags=s:cvode`; compare with the regular simulation |
| Weather file not found inside the FMU | Directory package + `includeResources=true`; fallback: BOPTEST's own `.mos` convention of copying the file into `Resources` and referencing it relatively |
| Buildings external C functions missing in the shared object | `platforms={"static"}`; the verify step loads the FMU on a clean machine image |
| Worker rejects an input without min/max or an output without unit | `verify.py` fails the export before packaging |
| Zone ids in `kpis.json` and CSV columns disagree | one `zone_id(space)` helper used by both `points.py` and `resources.py` |
| Variant explosion across six libraries | start with Buildings and IDEAS; the blocks are library agnostic, only the templates differ |

Acceptance: `trano export-boptest tests/tutorials/three_zones_hydronic_heating.yaml Buildings --fmu --verify`
produces a folder that, copied to `testcases/trano_three_zones/models`, lets `docker compose up` provision it and
`examples/python/testcase1.py`-style client select it, advance two days with a setpoint overwrite active, read
non-empty core KPIs and stop, with no change to BOPTEST.
