# BESTEST validation suite

End-to-end validation of Trano-generated [Buildings library](https://github.com/lbl-srg/modelica-buildings)
models against the **ASHRAE Standard 140 / IEA SHC Task 22 BESTEST** envelope
benchmark (Class I, single-zone cases).

## What this gives you

For each enabled case the suite produces:

- annual heating energy [kWh/m²]
- annual cooling energy [kWh/m²]
- peak heating power [W] + timestamp
- peak cooling power [W] + timestamp
- free-float zone temperature min/max [°C] + timestamps
- hourly zone temperature traces for two reporting days

…then compares each KPI against a reference band loaded from
[`spec/reference.csv`](spec/reference.csv) (placeholder values today; replace
with ASHRAE 140‑2017 Annex B numbers when available) and writes
`_reports/report.md`, `_reports/report.json`, plus one matplotlib trumpet plot
per KPI under `_reports/plots/`.

## Cases

| Case  | Envelope    | Window         | HVAC       | Iteration 1 status |
|-------|-------------|----------------|------------|--------------------|
| 600FF | lightweight | south 12 m²    | none       | **runs**           |
| 900FF | heavyweight | south 12 m²    | none       | **runs**           |
| 600   | lightweight | south 12 m²    | heater-only† | xfail              |
| 620   | lightweight | E 6 + W 6 m²   | heater-only† | xfail              |
| 640   | lightweight | south 12 m²    | heater-only with night setback† | xfail |
| 650   | lightweight | south 12 m²    | none (cooling+night-vent unsupported) | xfail |
| 900   | heavyweight | south 12 m²    | heater-only† | xfail              |
| 920   | heavyweight | E 6 + W 6 m²   | heater-only† | xfail              |
| 940   | heavyweight | south 12 m²    | heater-only with night setback† | xfail |
| 950   | heavyweight | south 12 m²    | none (cooling+night-vent unsupported) | xfail |

† Iteration 1 limitation — see "Known gaps" below.

Cases 610, 630, 910, 930 are **not authored** because they require window
overhangs and side fins that Trano's `Window` schema doesn't expose yet.

## How to run

Prerequisites: a reachable Docker daemon (the suite uses Trano's existing
OpenModelica-in-Docker harness).

```bash
# Pytest (parametrized, warn-only, marker-gated):
uv run pytest -m bestest -v

# Standalone CLI:
uv run python -m validation.bestest list
uv run python -m validation.bestest run-case 600FF
uv run python -m validation.bestest run-all
uv run python -m validation.bestest report     # re-render from cache
```

The default `pytest` run (no `-m`) skips `slow` tests, so the BESTEST suite
does not run inadvertently. Without Docker the pytest module skips cleanly.

## Caching

Results live in `.cache/bestest/<case_id>/`:

- `case.yaml` — merged YAML actually fed to `convert_network`
- `<name>.building_res.mat` — raw OpenModelica result (left in place by Trano's `simulate()`)
- `kpis.json` — serialised `KPIResults`
- `hash.txt` — SHA‑256 over `_base.yaml`, `_heavyweight.yaml` (when applicable),
  `case_<id>.yaml`, `harness.py`, and `spec/parameters.py`. Mismatch ⇒ rerun.

Both `.cache/bestest/` and `validation/bestest/_reports/` are gitignored.

## Known gaps (iteration 1 → 2)

These limitations are documented in the case YAML headers and in
`tests/test_bestest.py` (xfail markers). Address in iteration 2 to enable
the rest of the cases.

1. **Cooling control polarity.** Trano's `EmissionControl` variants only emit
   `yHea` (high when zone is *cold*). Wiring a negative-power "ideal cooler"
   `Radiator(variant: ideal)` to `yHea` yields inverted polarity (cools the
   cold zone). Fix: add a `cooling` variant to
   `trano/elements/library/models/default/emission_control.yaml` whose body
   exposes `yCoo` (high when zone is *hot*).
2. **Single emission per space.** `Space.find_emission()` raises
   `NotImplementedError` when a zone has more than one emission element
   (`trano/elements/space.py:198`). This blocks heater + cooler coexistence.
   Fix: relax that constraint to allow multiple emissions and connect each
   to its own control signal.
3. **Time-varying air change rate.** ASHRAE 140 cases 650/950 require night
   ventilation 10.4 ACH between 18:00 and 07:00. Buildings space template
   accepts only a constant `ACH`; the `infiltration` variant of
   `trano/elements/library/models/buildings/space.yaml` may expose a
   `RealInput` for time-varying ACH but needs investigation.
4. **Window overhangs / side fins.** Cases 610/630/910/930 require window
   shading geometry. Add `overhang` and `side_fin` fields to Trano's
   `Window` schema in `trano/data_models/trano.yaml` and propagate to
   `trano/elements/envelope.py` and the Buildings space template.

## Switching from warn-only to fail

When you trust the bands and Trano's HVAC support, change `warnings.warn(...)`
to `assert not out_of_band, ...` near the bottom of `tests/test_bestest.py`.

## Weather

All cases use the Buildings-bundled Denver TMY3 weather file
(`modelica://Buildings/Resources/weatherdata/USA_CO_Denver.Intl.AP.725650_TMY3.mos`).
ASHRAE 140 originally specifies `DRYCOLD.TMY`; Denver TMY3 is the modern
substitute commonly used when the legacy file is not available.

## Layout

```
validation/bestest/
├── README.md                     ← you are here
├── __main__.py                   ← Typer CLI
├── harness.py                    ← build_yaml, case_hash, run_case, run_all
├── kpi.py                        ← extract_kpis, integrate, find_peak, hourly_resample
├── report.py                     ← compare_to_reference, write_report, trumpet plots
├── cases/
│   ├── _base.yaml                ← lightweight envelope (600 series)
│   ├── _heavyweight.yaml         ← heavyweight envelope override (900 series)
│   └── case_<id>.yaml            ← per-case spaces + emissions
├── spec/
│   ├── parameters.py             ← CASES dict + Pydantic models
│   └── reference.csv             ← KPI bands (ASHRAE 140 Annex B)
└── _reports/                     ← gitignored — created on first `run-all`
```
