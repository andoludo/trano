# FastModel: analysis

`FastModel` is trano's reduced building library: the same YAML as Buildings and IDEAS, components written to be
smooth and explicit so that rumoca translates them into a CasADi ODE for optimal control (`FastModelMPC`) and
calibration (mpctrano), and so that the closed loop (`FastModel`) runs in OpenModelica too. This folder gathers the
analysis that brought it to its present state: whether it passes BESTEST, whether it reproduces Buildings and
responds to the YAML as Buildings does, whether complex houses export and solve as MPC problems, what was changed in
the library and why, and what is still open.

## In short

* **BESTEST**: 27 of 27 cases, 105 of 105 KPIs inside their band, shading included; FastModel gates as Buildings and
  IDEAS do. Against Buildings: loads 1.1 % apart on average (4.4 % at most), temperatures 0.38 K (1.70 K). It started
  at 83 of 85 KPIs of 22 cases with the heating 9 % above Buildings.
* **Against Buildings, closed loop, same YAML**: zones within 0.02 to 0.10 K hour by hour (0.46 K in an unheated
  sun-space), heating within 1.7 % in every base case. Of 45 changes of the YAML, 36 move both libraries alike,
  4 closely, 2 differ (both peaks of the heat pump) and 3 move neither.
* **MPC**: both residential houses (heat pump with buffer and ventilation; gas boiler) export with rumoca in under
  a minute (189 and 160 states) and IPOPT solves their 12-hour optimal control in 23 to 55 s, every zone at its bound.
  The closed loop and the MPC model share their state names, so mpctrano can start one from the other.
* **Boiler house**: rooms within 0.02 to 0.09 K, heating +1.7 %, gas +1.9 % (gas per unit of heat within 0.2 %);
  both water loops conserve energy within 0.15 %.
* **Open**: the heat pump at full load and its safety times, the response to the solar absorptance of the opaque
  surfaces (0.91 to 0.93), the zones that nothing cools keeping a little too much sun.

## Reports

| | Report | Question |
|---|---|---|
| 1 | [Requirements](01_requirements.md) | where each requirement stands, and where its check runs |
| 2 | [BESTEST](02_bestest.md) | does FastModel pass ASHRAE 140, and how far is it from Buildings, now and along the history |
| 3 | [Study against Buildings](03_study_against_buildings.md) | same results and same sensitivities as Buildings on complex cases |
| 4 | [Library changes](04_library_changes.md) | every change of `FastModel.mo`, the gap and evidence behind it, its effect |
| 5 | [Shading](05_shading.md) | overhangs and side fins, smooth enough for CasADi |
| 6 | [MPC: rumoca, CasADi, IPOPT](06_mpc_rumoca_ipopt.md) | do the residential houses export and solve as optimal control problems |
| 7 | [Closed loop and MPC states](07_closed_loop_and_mpc_states.md) | can mpctrano start the MPC model from the closed loop |
| 8 | [House before renovation](08_house_before_renovation.md) | the gas-boiler house, closed loop and MPC |
| 9 | [Boiler water loop](09_boiler_water_loop.md) | does FastModel lose heat or burn more gas per unit of heat |
| 10 | [Open gaps](10_open_gaps.md) | what is left, and how to close it within rumoca and CasADi |

## Scripts and results

Run from the root of the repository, with trano's environment (`uv sync --group dev`):

| Script | Needs | Writes |
|---|---|---|
| `python -m analysis.fast_model.scripts.bestest_comparison --history` | the committed BESTEST results, git | `bestest_kpis.csv`, `bestest_summary.json`, `bestest_history.csv` |
| `python -m analysis.fast_model.scripts.study_responses` | the committed study results | `study_kpis.csv`, `study_fidelity.csv`, `study_responses.csv`, `study_verdicts.csv`, `study_summary.json`, `study_hour_of_day.csv` |
| `python -m analysis.fast_model.scripts.shading_check` | trano-optimization (rumoca) | `shading_check.json` |
| `python -m analysis.fast_model.scripts.mpc_houses` | trano-optimization (rumoca, IPOPT) | `mpc_residential_after_renovation.json`, `mpc_residential_before_renovation.json` |
| `python -m analysis.fast_model.scripts.water_loop restart` | Docker, OpenModelica libraries | `water_loop_restart.json`, `water_loop_restart_table.csv`, `water_loop_components_restart.csv` |
| `python -m analysis.fast_model.scripts.water_loop month` | Docker, OpenModelica libraries | `water_loop_month.json` |
| `python -m analysis.fast_model.scripts.water_loop restart --tag tau0.5 --library FastModel --patch 'tauHyd(unit="s") = 5 ' 'tauHyd(unit="s") = 0.5 '` | Docker, OpenModelica libraries | `water_loop_restart_tau0.5.json`, `water_loop_components_restart_tau0.5.csv` |

* **trano-optimization** is not a dependency of trano: install it next to trano (`uv pip install -e
  ../trano-optimization`, with rumoca >= 0.10.3).
* **Simulations** run OpenModelica in Docker (image `openmodelica/openmodelica:v1.26.9-ompython`) with the libraries
  in `TRANO_MODELICA_LIBRARIES` (Buildings 13, IDEAS 3, MSL 4), as the validation does; the runs are cached in
  `.cache/analysis/` by the hash of the model.
* The detailed validation results these scripts read live in `validation/bestest/results`,
  `validation/bestest/expected` and `validation/fast_model/results`, produced by the workflows `bestest.yaml` and
  `fast_model_study.yaml`; the pages built from them are
  [`docs/validation/bestest.md`](../../docs/validation/bestest.md),
  [`docs/validation/fast_model_study.md`](../../docs/validation/fast_model_study.md) and
  [`docs/validation/fast_model_improvement_plan.md`](../../docs/validation/fast_model_improvement_plan.md).

The timings in the results (exports, IPOPT, OpenModelica) are those of the container the analysis ran in (4 cores);
they vary from machine to machine, the other numbers do not.
