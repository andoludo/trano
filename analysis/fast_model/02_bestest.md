# 2. BESTEST (ASHRAE 140) with FastModel

**Question.** Does FastModel, the smooth library that rumoca translates to CasADi, pass every requirement of the
BESTEST validation that Buildings and IDEAS pass, and how far is it from Buildings on the same YAML?

**Answer.** Yes: the 27 cases, 105 of 105 KPIs inside their band, and FastModel is now a **gating** library
(`pytest -m bestest` fails when one of its cases leaves its band, as for Buildings and IDEAS). Against Buildings, the
loads differ by 1.1 % on average and 4.4 % at most, the temperatures by 0.38 K on average and 1.70 K at most.

Data: [`results/bestest_kpis.csv`](results/bestest_kpis.csv) (one row per case and KPI),
[`results/bestest_summary.json`](results/bestest_summary.json),
[`results/bestest_history.csv`](results/bestest_history.csv); script
[`scripts/bestest_comparison.py`](scripts/bestest_comparison.py) (`--history`). The full tables with the bands are on
the [BESTEST page](../../docs/validation/bestest.md#fastmodel).

## Method

Each case is a trano YAML file (`validation/bestest/cases/`) simulated for a year by OpenModelica with each library.
The annual loads must fall inside the acceptance limits of the standard; the peaks and the free-floating temperatures
inside the spread of the reference programs widened by 5 % of the largest peak or 1 K
(`validation/bestest/report.acceptance_band`). The FastModel results were produced locally and confirmed by the
BESTEST workflow on the branch (run 39949636202: `pytest -m bestest -k FastModel`, 27 passed, the same values); they
are frozen in `validation/bestest/expected/FastModel.json` so that a drift inside the bands fails too.

## Results now

| | All 27 cases | The four shading cases (610, 630, 910, 930) |
|---|---|---|
| KPIs inside their band | 105 of 105 | 16 of 16 |
| Annual heating against Buildings, mean (range) | +0.2 % (−0.9 to +2.5 %) | +0.4 % (−0.2 to +0.9 %) |
| Annual cooling against Buildings, mean (range) | +1.3 % (−2.3 to +4.4 %) | +0.9 % (+0.2 to +1.4 %) |
| Loads (annual and peaks), mean gap (largest) | 1.1 % (4.4 %) | 0.7 % (2.2 %, 910 peak cooling) |
| Temperatures, mean gap (largest) | 0.38 K (1.70 K) | none in these cases |

The largest gaps to Buildings, all inside their bands:

| Case | KPI | FastModel | Buildings | Gap |
|---|---|---|---|---|
| 980 | annual cooling | 3.567 MWh | 3.415 MWh | +4.4 % |
| 960 | peak cooling | 1.424 kW | 1.476 kW | −3.5 % |
| 980 | peak cooling | 3.404 kW | 3.296 kW | +3.3 % |
| 680 | annual cooling | 6.316 MWh | 6.136 MWh | +2.9 % |
| 680FF | highest temperature | 70.62 degC | 68.91 degC | +1.70 K |
| 600FF | highest temperature | 64.38 degC | 63.32 degC | +1.06 K |
| 650FF | highest temperature | 63.17 degC | 62.14 degC | +1.04 K |
| 960 | highest temperature (sun-space) | 49.05 degC | 48.04 degC | +1.01 K |

The pattern is the one of the [study](03_study_against_buildings.md): FastModel keeps a little more of the sun in
the zones that nothing cools (the light free-floating zones, the sun-space) and cools the sunny cases a little more.

## Along the history of the branch

Every version of `expected/FastModel.json` in git, set against the Buildings values frozen at the same commit
(`bestest_comparison.py --history`); each line is a library change described in
[the library changes](04_library_changes.md):

| Commit | Library | Cases | Inside | Heating, mean | Cooling, mean | Loads, mean gap | Temperatures, mean (largest) |
|---|---|---|---|---|---|---|---|
| 42a466e | first FastModel run | 22 | 83 / 85 | +9.0 % | +3.1 % | 5.6 % | 0.48 K (1.49 K) |
| 3b04e14 | air density at the site, exact smooth limits | 22 | 85 / 85 | −0.4 % | +5.5 % | 3.4 % | 1.04 K (4.39 K) |
| b7f503c | walls discretised as Buildings, single glazing | 23 | 87 / 89 | −0.6 % | +5.5 % | 3.1 % | 1.16 K (4.57 K) |
| ef062cb | room long-wave exchange around the surface temperatures | 23 | 89 / 89 | −0.6 % | +5.0 % | 2.7 % | 0.93 K (3.82 K) |
| 7658816 | exact long-wave exchange with sky and ground | 23 | 89 / 89 | +0.6 % | +2.4 % | 2.0 % | 0.80 K (3.52 K) |
| b0e90cd | gas gaps at the temperatures of their panes, small hydraulic links | 23 | 89 / 89 | +0.2 % | +1.4 % | 1.2 % | 0.38 K (1.70 K) |
| adf7f00 | overhangs and side fins; FastModel gating | 27 | 105 / 105 | +0.2 % | +1.3 % | 1.1 % | 0.38 K (1.70 K) |

Two steps made a number worse before the next one fixed it, and both were diagnosed rather than tuned: removing the
excess infiltration (air density) uncovered a heat surplus in the sunny zones (cooling +5.5 %, free-floating peaks up
to 4.4 K high), which the long-wave exchange of the room and of the sky (+2.4 %) and the gas gaps at the temperatures
of their panes (+1.4 %) then explained.

## Reproduce

```bash
python -m analysis.fast_model.scripts.bestest_comparison --history   # reads committed results only
python -m validation.bestest run --library FastModel --workers 3      # the year-long runs (Docker, ~10 min)
pytest -m bestest -k FastModel                                        # the gate
```
