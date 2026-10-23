# 1. The requirements and where each one stands

The goal set for FastModel: *a smooth library, compatible with rumoca and CasADi/IPOPT, compatible with complex trano
house descriptions with a heat pump, that passes every BESTEST requirement, and can still be used in an MPC*; and, for
every parameter of the YAML, *the same or a comparable impact as in Buildings*.

| Requirement | Status | Evidence |
|---|---|---|
| Smooth, event-free library | met | every component is an explicit ODE with `regStep`, `smoothMax`, `smoothMin`, `smoothLimit` and `sqrt(x^2 + d^2)`; no `when`, `pre` or `sample` in the building model; the smoothing of the controllers is exposed as parameters ([library changes](04_library_changes.md)) |
| rumoca → CasADi | met | every FastModel and FastModelMPC model of the tests exports with rumoca 0.10.3, the two residential houses and the shaded BESTEST case 630 included ([MPC](06_mpc_rumoca_ipopt.md), [shading](05_shading.md)) |
| IPOPT optimal control | met | both residential houses solve a 12-hour optimal control (`Solve_Succeeded`, no comfort slack) with their real systems as controls and their meters as cost |
| Complex house with a heat pump | met | the renovated house: air-to-water heat pump with a buffer, three-way valve, six radiators with thermostatic valves, balanced ventilation with heat recovery, PV, six zones; closed loop within 0.06 to 0.10 K of Buildings, heating +1.5 %, heat-pump electricity −0.2 % |
| …and with a gas boiler | met | the house before renovation: rooms within 0.02 to 0.09 K, heating +1.7 %, gas +1.9 %, the same gas per unit of heat within 0.2 % ([house before renovation](08_house_before_renovation.md)) |
| Every BESTEST requirement | met | 27 of 27 cases, 105 of 105 KPIs inside their band; FastModel gates in `pytest -m bestest` as Buildings and IDEAS do ([BESTEST](02_bestest.md)) |
| Usable in an MPC (mpctrano) | met | the closed loop and the MPC model share their state names, so a closed-loop spin-up starts the MPC model; mpctrano's calibration, flexibility, project and model tests pass against the branch ([closed loop and MPC](07_closed_loop_and_mpc_states.md)) |
| Same or comparable impact of a YAML change as in Buildings | met for 40 of the 42 perturbations that move the results | 36 alike, 4 close, 2 differ (both peaks of the heat pump), 3 move neither library ([study](03_study_against_buildings.md)) |

What is not met, and the proposal for each, is in [open gaps](10_open_gaps.md): the heat pump at full load and its
safety times (the two perturbations that differ), the response to the solar absorptance of the opaque surfaces (0.91
to 0.93, inside *alike* but systematically low), and the sun kept in the zones that nothing cools.

## Where the checks run

| Check | Where | Runs in CI |
|---|---|---|
| rumoca exports, the probes of the gap conductance, the shading and the walls, IPOPT on both houses | `tests/test_fast_model.py` | no: trano-optimization (rumoca) is not a dependency of trano; run locally |
| OpenModelica compiles the FastModel examples (both houses included) | `tests/test_fast_model.py -m compile` | yes, `compile.yaml` |
| OpenModelica simulates a FastModel example | `tests/test_fast_model.py -m simulate` | no (CI simulates the AixLib cases only); run locally |
| BESTEST of FastModel against the bands and the frozen values | `pytest -m bestest -k FastModel`, `tests/test_bestest_results.py` | the workflow `bestest.yaml` (dispatch) and the fast suite |
| The study against Buildings | `validation/fast_model`, `tests/test_fast_model_study.py` | the workflow `fast_model_study.yaml` (`[study]` in a commit message) |
| mpctrano | `mpctrano/tests` | in mpctrano, against a released trano |
