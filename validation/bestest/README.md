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

## Compatibility matrix

Every case below uses the same envelope geometry (48 m² floor area, 2.7 m
height), 200 W continuous internal gain split 60 %
radiative / 40 % convective per §5.2.1.7, Denver TMY3 weather, and double
glazing per §5.2.4 (3.175 mm panes, k = 1.06 W/(m·K), τ = 0.86156, 13 mm air gap).
Infiltration is left at the Buildings MixedAir default (gap, see below) —
the spec value is 0.5 ACH nominal (altitude-adjusted 0.41 at Denver's 1609 m).

| Case | Spec definition | Iteration 1 status | What's missing for full compliance |
|------|-----------------|--------------------|-----------------------------------|
| 600FF | LW envelope, S window 12 m², no HVAC | **COMPATIBLE — runs** | — |
| 900FF | HW envelope, S window 12 m², no HVAC | **COMPATIBLE — runs** | — |
| 600 | LW + heat<20 °C / cool>27 °C, ideal capacity | **PARTIAL** (heater-only; xfail) | cooling control (gap 1) |
| 620 | 600 with windows E/W 6 m² each | **PARTIAL** | cooling control |
| 640 | 600 with heating setback to 10 °C 23:00–07:00 | **PARTIAL** | cooling control |
| 900 | 600 with HW envelope | **PARTIAL** | cooling control |
| 920 | 900 with windows E/W | **PARTIAL** | cooling control |
| 940 | 900 with setback | **PARTIAL** | cooling control |
| 650 | cool>27 °C 07–18 h, night vent 13.14 ACH 18–07 h | **INCOMPATIBLE** (no HVAC; xfail) | cooling control + time-varying ACH (gaps 1, 3) |
| 950 | 650 with HW envelope | **INCOMPATIBLE** | cooling control + time-varying ACH |
| 610, 630, 910, 930 | shading: 1 m overhang (±fins) | **NOT EXPRESSIBLE** — not authored | overhang/fin fields in `Window` (gap 4) |

"PARTIAL" cases simulate end-to-end and the heating-side KPIs are meaningful;
cooling-side KPIs report 0 because no cooler element is wired (intentional —
see gap 1). They are marked `pytest.xfail` so the suite stays green.

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
3. **Time-varying air change rate.** ASHRAE 140 cases 650/950 require a
   night ventilation fan delivering 1703.16 m³/h (≈ 13.14 ACH on a 129.6 m³
   zone) between 18:00 and 07:00. The Buildings space template accepts only
   a constant `ACH`; the `infiltration` variant of
   `trano/elements/library/models/buildings/space.yaml` may expose a
   `RealInput` for time-varying ACH but needs investigation.
4. **Window overhangs / side fins.** Cases 610/630/910/930 require window
   shading geometry. Add `overhang` and `side_fin` fields to Trano's
   `Window` schema in `trano/data_models/trano.yaml` and propagate to
   `trano/elements/envelope.py` and the Buildings space template.
5. **Opaque surface solar absorptance.** ASHRAE 140 specifies exterior and
   interior solar absorptance 0.6 on opaque surfaces. Trano's
   `OpaqueConstructions.Generic` glazing-system template (see
   `trano/elements/library/library.json`) doesn't emit `absSol_a/absSol_b`,
   so Buildings' default value (0.5) applies and the simulated solar gains
   on opaque surfaces are systematically below spec.
6. **Window frame.** ASHRAE 140 windows have no frame, but Trano emits
   `UFra = 1.4` (per the glazing template). The Buildings window component
   may apply a default frame fraction; if so, the modelled window area is
   slightly below the spec's 12 m² (6 m²) of pure glazing. Confirm during
   execution and document the deviation, or extend the template to emit
   `fFra = 0`.
7. **Constant infiltration ACH.** ASHRAE 140 §5.2.1.6 specifies 0.5 ACH
   (0.41 altitude-adjusted at Denver). `Buildings.ThermalZones.Detailed.MixedAir`
   has no `ACH` parameter — infiltration is supplied via a separate
   `Buildings.Fluid.Sources.MassFlowSource_T` connected to the air port.
   Trano's space schema currently has no way to attach that source, so the
   YAMLs cannot set infiltration and MixedAir's internal default is used.
   Fix: expose an infiltration field on the space and emit a paired
   mass-flow source in the Buildings space template.

## Switching from warn-only to fail

When you trust the bands and Trano's HVAC support, change `warnings.warn(...)`
to `assert not out_of_band, ...` near the bottom of `tests/test_bestest.py`.

## Weather

All cases use the Buildings-bundled Denver TMY3 weather file
(`modelica://Buildings/Resources/weatherdata/USA_CO_Denver.Intl.AP.725650_TMY3.mos`).
ASHRAE 140 originally specifies `DRYCOLD.TMY`; Denver TMY3 is the modern
substitute commonly used when the legacy file is not available.

## Relationship to `tests/models/bestest/case600FF.yaml`

The pre-existing fixture `tests/models/bestest/case600FF.yaml` (and its
golden Modelica file under `tests/data/`) is used by
`tests/test_template.py` as a code-generation snapshot test, not as part of
this validation suite. It is intentionally **not** modified here, even
though its internal-gain split has the same inversion that this suite
fixes — touching it would also force regenerating the golden `.mo` file.

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
