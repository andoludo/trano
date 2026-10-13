# Prior art: lightweight thermal models and optimization toolchains (Phase 0b)

*Search and tests done on 2026-10-13. This document feeds Phase 2.*

**Question.** Is there an existing library or toolchain that trano should reuse instead of building its own,
for lightweight building and district thermal models used in optimization?

**Requirement**, as clarified with the maintainer:

- one model generated from the trano YAML;
- that model must support **simulation**, **dynamic optimization with IPOPT**, **state and parameter estimation**
  and, ultimately, **MPC**;
- the whole pipeline must be **free and fast**;
- CasADi and rumoca are welcome but not required.

**Method.**

- Desk research: GitHub, PyPI, Modelica Association, OpenModelica docs, the rumoca repo and its issues,
  CasADi, and papers.
- Hands-on tests in a scratch environment:
  - a probe of the Modelica features rumoca supports;
  - an attempt to compile MSL and IBPSA with rumoca;
  - one benchmark solved by six pipelines on trano's own generated model.

## Recommendation

**Keep building on trano's own `mpc` generator. Borrow components from other libraries. Do not reuse or fork any
Modelica library as-is.**

**Standard pipeline**: trano YAML → flat explicit-ODE Modelica (`building_mpc`) → rumoca → CasADi → **do-mpc**.
On top of the one CasADi model, do-mpc provides:

- simulation (CVODES);
- moving-horizon estimation of states and parameters;
- nonlinear MPC on IPOPT.

**Batch identification**: a direct CasADi `Opti` problem on IPOPT.

**OpenModelica**:

- **reference plant**: simulates the detailed Buildings/IDEAS output of the same YAML;
- **independent cross-check of the OCP**: its built-in IPOPT optimizer runs on `building_mpc` unchanged.

**Why:**

- Of the libraries tested, none exports to an optimizer. rumoca's CasADi target accepts only flat, explicit ODEs.
  Anything built from connectors (MSL `HeatTransfer`, IBPSA, AixLib, Buildings, FastBuildings) produces
  algebraic variables, and the export rejects them. IBPSA does not even resolve in rumoca 0.10.2.
- trano's generator already emits exactly the subset that the free optimizers handle.
- In the benchmark, the rumoca + CasADi (+ do-mpc) pipelines were the only ones that did all four tasks:
  - simulation in 0.11 s for 7 days;
  - 48 h OCP;
  - warm MPC re-solves in 0.16–0.29 s;
  - estimation recovered the true capacity scales within 0.7 %.

## 1. Toolchain benchmark on trano's model

**Model.** `house.building_mpc`, generated from `tests/models/three_zones_mpc.yaml` with R3C2 zones:
6 states, 10 inputs, 32 parameters.

**Tasks** (identical for every pipeline):

- **Simulate**: 7 days with 1.5 kW per zone.
- **OCP**: 48 h economic problem at 15 min steps (192 intervals), with:
  - a time-of-use price;
  - `20 °C ≤ Ti ≤ 24 °C`;
  - `0 ≤ QHea ≤ design power`.
- **MPC**: 24 receding-horizon re-solves of the OCP.
- **Estimate**: indoor and envelope capacity scales plus the unknown initial states, from 3 days of noisy `Ti`
  (σ = 0.1 K). The true scales are 1.3 and 0.8.

**Setup.** All runs used one CPU of the session container, with IPOPT and MUMPS.

| # | Pipeline | Model input | Simulate 7 d | OCP 48 h (first solve) | Warm MPC solve | Estimation (true: 1.3 / 0.8) | OCP objective | Verdict |
|---|----------|-------------|--------------|------------------------|----------------|------------------------------|---------------|---------|
| P1 | rumoca 0.10.2 → CasADi 3.8 `Opti` (RK4 multiple shooting) → IPOPT | trano Modelica | 0.11 s (CVODES) | 5.0 s (build + SX expansion; 148 iterations) | **0.16 s** | batch, 4.3 s → **1.2996 / 0.8027** | 19.593 € | ✅ all four tasks |
| P2 | rumoca → CasADi → **do-mpc 5.1.2** (collocation, degree 2) → IPOPT | trano Modelica | 0.57 s | 1.3 s setup + 2.5 s | 0.29 s | **MHE**, 1-day window, 0.069 s per step → **1.303 / 0.805** | 19.352 € | ✅ all four tasks, ready-made MHE + MPC |
| P3 | **OpenModelica 1.26.9** `optimize()` (Radau IIA, 3 points) → IPOPT 3.14 | trano Modelica, unchanged | 0.02 s (+0.9 s compile) | **0.42 s** (24 iterations; +0.9 s compile) | 0.43 s per run (new process, no warm start) | ❌ parameters cannot be decision variables; a constant-state workaround ended in IPOPT "Restoration Failed" | 19.599 € | ✅ simulate and OCP; ❌ estimation; MPC only through scripting |
| P4 | Python translation of the flat equations → **Pyomo 6.10** `dae` (Radau, 3 points) → IPOPT 3.13 | generated Python | 1.3 s | 5.6 s | 0.76 s (rewrites the `.nl` file at every solve) | batch, 1.3 s → **1.2996 / 0.8027** | 19.593 € | ✅ all four tasks, slower loop |
| P5 | Python translation → **GEKKO 1.3.2** with `remote=False` | generated Python | 11.2 s | ❌ the local binary has no IPOPT ("solver 3 not supported") and falls back to APOPT: 24 steps take 78 s, 192 steps were still running after 10 min | – | not run | – | ❌ IPOPT only on the public BYU server, so model data would leave the machine |
| P6 | trano Modelica → pymoca 0.9.3 → **rtc-tools 2.8.1** (collocation) → IPOPT | trano Modelica (`input Real` instead of the `Trano.MPC.RealInput` connector) | not tested | 5.2 s on first run (pymoca compile), 1.55 s cached | 1.55 s (rebuilds the problem every run) | ❌ no estimator built in | 19.628 € | ◐ OCP and MPC; DAEs allowed |
| P7 | ModelingToolkit.jl + ModelPredictiveControl.jl (Julia, MIT) | generated Julia, or Base Modelica | – | – | – | – | – | not run: the Julia download hosts are blocked in this environment; desk research only (section 2) |

**Cross-checks:**

- The final `space_001_Ti` of the 7-day simulation agrees to within 0.04 K across all pipelines
  (290.49–290.53 K).
- The OCP objectives agree within discretisation error. The shooting and Radau schemes match to six digits;
  do-mpc's degree-2 collocation and rtc-tools' scheme differ by about 1 %.
- The Python translation used for P4 and P5 is a regex over `der(x) = expr;`, which only works because the
  `mpc` contract forbids every construct that would need a real compiler. trano could therefore emit CasADi,
  Pyomo or JuMP code directly, without going through rumoca.

## 2. Toolchain landscape

These entries come from desk research, except where a row of section 1 is cited.

| Toolchain | License, latest release | IPOPT | Simulate | Estimate | MPC | Modelica in | Notes |
|-----------|-------------------------|-------|----------|----------|-----|-------------|-------|
| [rumoca](https://github.com/cognipilot/rumoca) | Apache-2.0; 0.10.2 (2026-10-04) | via CasADi or JAX export | native, but **no time-varying inputs yet** ("roadmap §8") | `objective_gradient` only | – | native compiler; 64 % of MSL 4.1 examples simulate | Alpha. `casadi-ode` accepts **explicit ODEs only** (section 4). On the main branch, that target's `target.toml` enables explicit algebraic assignments (not built or tested here). No IPOPT or MPC roadmap. |
| [do-mpc](https://github.com/do-mpc/do-mpc) | LGPL-3; 5.1.2 (2026-09) | ✅ | ✅ CVODES/IDAS, DAE | ✅ EKF, MHE with parameters | ✅ NMPC, robust | via a CasADi model | **Best single Python package for the four tasks** (P2) |
| [CasADi](https://github.com/casadi/casadi) / [rockit](https://gitlab.kuleuven.be/meco-software/rockit) | LGPL-3; 3.8.1 | ✅ | ✅ | do it yourself | do it yourself | FMU import (FMI 2/3) | Base layer of P1 and P2 |
| [acados](https://github.com/acados/acados) | BSD-2 | SQP-RTI by default | ✅ | MHE as an OCP | ✅ fastest real-time NMPC | – | Use only if real-time limits require it |
| [OpenModelica](https://github.com/OpenModelica/OpenModelica) | OSMC-PL / AGPL; runtime BSD; 1.27.1 | ✅ built-in dynamic optimization | ✅ full Modelica | ❌ data reconciliation covers static models only | scripting only (OMPython) | native | P3. Can export Base Modelica |
| [rtc-tools](https://github.com/rtc-tools/rtc-tools) + [mesido](https://github.com/Multi-Energy-Systems-Optimization/mesido) | LGPL-3; 2.8.1, mesido 0.1.22 | ✅ (HiGHS for MILP) | ✅ | ❌ | receding horizon by scripting | via pymoca 0.9 | P6. mesido covers district-heating MILP in Python (pycml); its legacy Modelica package targets MSL 3 |
| [pymoca](https://github.com/pymoca/pymoca) | BSD-3; 0.12.0 | via CasADi | – | – | – | subset; DAE allowed | 0.12 is incompatible with CasADi 3.8 (`MX.__div__`); it works with CasADi 3.6. In my probe it handled connectors, arrays and `if`, but not MSL `HeatTransfer` or user functions |
| [Pyomo](https://github.com/Pyomo/pyomo) `dae`, `contrib.mpc`; IDAES | BSD-3; 6.10.1 | ✅ | via collocation | ✅ batch | ✅ | – | P4 |
| [GEKKO](https://github.com/BYU-PRISM/GEKKO) | MIT; 1.3.2 | **remote server only** | ✅ | MHE | MPC | – | P5. `remote=True` is the default |
| [ModelingToolkit.jl](https://github.com/SciML/ModelingToolkit.jl) + [ModelPredictiveControl.jl](https://github.com/JuliaControl/ModelPredictiveControl.jl) | MIT; MTK 11.45, MPC.jl 2.14 | ✅ through JuMP / InfiniteOpt | ✅ | ✅ KF, EKF, UKF, MHE | ✅ linear, nonlinear, economic | [BaseModelica.jl](https://github.com/SciML/BaseModelica.jl) (no records or functions) | Most complete free estimation and MPC stack; costs a Julia dependency and JIT latency. **Not tested here** |
| [AgentLib-MPC](https://github.com/RWTH-EBC/AgentLib-MPC) | BSD-3; 0.6.4 (main branch 2026-08) | ✅ (CasADi) | FMU / CasADi | MHE | NMPC, ADMM, ML models | – | Building-focused multi-agent MPC; a template for later phases |
| Modelon OCT / JModelica.org; Dyad / JuliaSim Control; MPCPy | commercial; source-available; dead (Python 2, JModelica) | | | | | | Excluded as not free or not maintained |

## 3. Model libraries

The column "Covers trano YAML" scores the concepts in `trano/data_models/trano.yaml` as Env / Occ / Hyd / Prod /
Stor / EV / PV / Vent / DH:

- Env: envelope and zones
- Occ: occupancy
- Hyd: emission and hydronics (radiators, valves, pumps)
- Prod: production (boilers, heat pumps, chillers)
- Stor: storage (DHW tank, battery)
- EV: EV charger
- PV: photovoltaics
- Vent: ventilation (AHU, VAV)
- DH: district heating

Scores: ✓ covered, ◐ partial, · not covered.

"Tested" in the rumoca column means I ran it with rumoca 0.10.2.

| Candidate | License | Last activity | Physics scope | rumoca / optimizer compatibility | Covers trano YAML (Env/Occ/Hyd/Prod/Stor/EV/PV/Vent/DH) | Use |
|-----------|---------|---------------|---------------|----------------------------------|---------------------------------------------------------|-----|
| **trano `mpc` library** (baseline) | trano | this repo | R1C1, R3C2, R4C3, 5R2C zones; inline heat pump, boiler, chiller, DHW tank, battery, EV, PV | ✅ tested: 132 rumoca tests pass; P1–P4 and P6 | ✓ ✓ ◐ ✓ ✓ ✓ ✓ ◐ · | **extend** |
| [IBPSA](https://github.com/ibpsa/modelica-ibpsa) `ThermalZones.ReducedOrder.RC`, `ISO13790.Zone5R1C` | BSD-3 style | 2026-10-09 | VDI 6007 1–4 element RC; ISO 13790 5R1C; solar gains | ❌ tested: does not resolve (`ModelicaServices.Machine.eps`); uses connectors, `MixingVolume` with a fluid medium, the weather bus and tables | ✓ · · · · · · · · | **borrow** equations and validation cases |
| [Buildings](https://github.com/lbl-srg/modelica-buildings) incl. `DHC` | BSD-3 style | v14.0.0, 2026-10 | everything, including district heating and cooling | ❌ (fluid media, events, tables, external C) | ✓ ✓ ✓ ✓ ✓ ◐ ✓ ✓ ✓ | high-fidelity **plant** (already used by trano) |
| [AixLib](https://github.com/RWTH-EBC/AixLib) | BSD-3 | v3.0.1, 2026-06 | VDI 6007 multizone, HVAC, district | ❌ (fluid, tables, events) | ✓ ✓ ✓ ✓ ✓ · ◐ ✓ ✓ | plant (already used by trano) |
| [IDEAS](https://github.com/open-ideas/IDEAS) / LIDEAS | BSD-3 style | v4.0.0, 2025-09 | detailed envelope; LIDEAS linearises to state-space models for MPC (needs Dymola) | ❌ | ✓ ✓ ✓ ✓ ◐ · ✓ ✓ · | plant (already used by trano); linearisation idea |
| [FastBuildings](https://github.com/open-ideas/FastBuildings) | Modelica License 2 | 2022-12 | about 40 grey-box single-zone RC variants | ◐ only MSL `HeatTransfer` plus `replaceable`, `inner`/`outer` and legacy IDEAS imports; connectors give algebraic variables, so `casadi-ode` rejects it (probe) | ✓ · ◐ · · · · · · | **borrow** model structures for identification |
| [TEASER](https://github.com/RWTH-EBC/TEASER) | MIT | v1.3.1, 2025-12 | Python archetypes → VDI 6007 R/C parameters | n/a (Python) | ✓ (parameters) | **borrow** the parameter calculation, cross-check `trano.mpc.estimation` |
| [OCHRE](https://github.com/NREL/OCHRE) (NREL) | BSD-3 | 0.9.2, 2025-12 | Python residential RC envelope, HVAC, water heater, EV, PV, battery | n/a (Python, no optimizer) | ✓ ✓ ◐ ✓ ✓ ✓ ✓ ◐ · | **borrow** equipment physics and defaults |
| [HiSim](https://github.com/FZJ-IEK3-VSA/HiSim) (FZJ) | MIT | 1.2.4, 2026-10 | 5R1C, heat pumps, battery, EV, PV, storage | n/a (Python) | ✓ ✓ ◐ ✓ ✓ ✓ ✓ · · | **borrow** heat pump and battery maps |
| [mesido](https://github.com/Multi-Energy-Systems-Optimization/mesido) | LGPL-3 | 0.1.22, 2026-10 | district heating MILP: pipes, buffers, ATES, heat pumps | rtc-tools (pycml in Python; legacy Modelica targets MSL 3) | · · · ✓ ✓ · · · ✓ | **borrow** the district formulation (later phase) |
| [MoSDH](https://github.com/MoSDH/MoSDH) | BSD-3 | 2024-03 | solar district heating, storage, networks on `FluidHeatFlow` | ❌ for `casadi-ode` (connectors, events) | · · ✓ ✓ ✓ · · · ✓ | district reference model |
| [pycity_scheduling](https://gitlab.git.nrw/rwth-acs/public/simulation/pycity_scheduling) | MIT | 1.4.4, 2026-07 | Pyomo district scheduling: building, heat pump, CHP, battery, EV, PV | n/a (Pyomo) | ◐ · · ✓ ✓ ✓ ✓ · ◐ | district optimization formulation |
| [GMT (URBANopt)](https://github.com/urbanopt/geojson-modelica-translator), [uesgraphs](https://github.com/RWTH-EBC/uesgraphs) | BSD / MIT | 2026-07 | generators: GeoJSON or graph → Buildings / AixLib district models | ❌ (detailed targets) | district generators | closest analogues of trano's design |
| [MPCPy](https://github.com/lbl-srg/MPCPy), BRCM (ETH, MATLAB), [RC_BuildingSimulator](https://github.com/architecture-building-systems/RC_BuildingSimulator) | BSD / ? / MIT | 2014–2023 | RC models for MPC | dead (Python 2 / JModelica, MATLAB, unmaintained) | ✓ only | methodology references |

BOPTEST, DESTEST, Energym, Sinergym and CityLearn are **benchmarks**, not model sources. Use BOPTEST later to
validate trano's MPC against a common reference.

## 4. What rumoca 0.10.2 accepts (feature probe)

| Construct | Compiles | `to_casadi()` |
|-----------|----------|---------------|
| flat explicit ODE (`der(x) = f(x, u, p)`) | ✅ | ✅ |
| `max(·)` / `min(·)` | ✅ | ✅ |
| any algebraic variable (including an `output`, or an alias from `extends` + `replaceable`) | ✅ | ❌ `residual_equations` |
| `connect()` and MSL `Thermal.HeatTransfer` components | ✅ | ❌ `residual_equations` |
| `for` loop over an array state | ✅ | ❌ `dynamic_derivative_subscripts` |
| `if` expression, `when` | ✅ | ❌ `events` |
| user `function` | ✅ | ❌ `PureCall` |
| `Modelica.Constants`, `CombiTimeTable`, IBPSA | ❌ unresolved `ModelicaServices.Machine.*` | – |

This is exactly trano's `mpc` contract (see `docs/tutorials/rc_model_predictive_control.md`). Phase 2 must keep
the generated `building_mpc` inside it.

## 5. Inputs to Phase 2

1. **Keep the flat explicit-ODE contract** of `building_mpc`, and keep the rumoca translation test suite as its
   guard. Revisit when rumoca releases explicit algebraic assignments for `casadi-ode`.
2. **Add a do-mpc adapter**: `MPCModelInterface` + rumoca export → `do_mpc.model.Model`. Map:
   - controls to `_u`;
   - disturbances to `_tvp`;
   - selected parameters to `_p`, so the MHE can estimate them.

   Ship it as an optional extra and test simulate, MHE and MPC as in P2.
3. **Hedge the rumoca dependency.** rumoca is alpha and maintained by a single organisation. Since trano owns the
   equations, add a direct CasADi emitter (or Pyomo, or Julia) from the same templates. It removes one compiler
   from the critical path, and P4 shows how small the translation is.
4. **Borrow, don't fork**:
   - TEASER / VDI 6007 parameterisation and the IBPSA validation cases for the RC parameters;
   - OCHRE / HiSim maps for heat pump, water heater, battery and EV;
   - mesido / pycity_scheduling formulations when district heating is added.
5. **Validate** the MPC models against trano's Buildings/IDEAS output, simulated by OpenModelica, as the plant.
   Optionally cross-check OCPs with OpenModelica's native optimizer, which runs on the unchanged Modelica file.
6. **Re-evaluate the Julia stack** (ModelingToolkit + ModelPredictiveControl.jl) once its package hosts are
   reachable. It is the only free alternative with KF/UKF/MHE and economic MPC in one package.
