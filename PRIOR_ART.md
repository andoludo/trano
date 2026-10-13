# Prior art: lightweight thermal models and optimization toolchains (Phase 0b)

*Search and tests done on 2026-10-13. This document feeds Phase 2.*

**Question.** Is there an existing library or toolchain that trano should reuse instead of building its own,
for lightweight building and district thermal models used in optimization?

**Requirement**, as clarified with the maintainer:

- one model generated from the trano YAML;
- that model must support **simulation**, **dynamic optimization with IPOPT**, **state and parameter estimation**
  and, ultimately, **MPC**;
- the whole pipeline must be **free and fast**;
- the models must be **connectable components**, wired the way trano wires Buildings/IDEAS models, while staying
  light, fast and accurate;
- CasADi and rumoca are welcome but not required.

**Method.**

- Desk research: GitHub, PyPI, Modelica Association, OpenModelica docs, the rumoca repo and its issues,
  CasADi, and papers.
- Hands-on tests in a scratch environment:
  - a probe of the Modelica features rumoca supports;
  - an attempt to compile MSL and IBPSA with rumoca;
  - one benchmark solved by six pipelines on trano's own generated model;
  - the same house rebuilt from connectable components (own connector library and MSL `HeatTransfer`), run through
    every pipeline that accepts connectors (section 1b).

## Recommendation

**Write a small connectable component library inside trano and borrow physics and parameters from other projects.
Do not reuse or fork an existing Modelica library: none of them is connectable, light and optimizer-ready at the
same time.**

- **IBPSA, AixLib, Buildings and IDEAS** are connectable but not light: fluid media, events, tables and external C.
  IBPSA still does not resolve in rumoca, even on main.
- **MSL `Thermal.HeatTransfer`** is connectable and light. It works with rumoca main and OpenModelica, but:
  - pymoca 0.12 fails on its `RealInput` connector;
  - loading MSL makes rumoca compile 35 times slower (5.3 s instead of 0.15 s).
- **FastBuildings** is MSL `HeatTransfer` plus legacy IDEAS imports, so it inherits the MSL issues plus broken
  imports.

**What the library looks like.** About 100 lines of Modelica (`Light` in section 1b):

- trano's own `HeatPort` connector;
- capacitor, resistor, conductor, boundaries, heat flow and solar components;
- zone templates (R1C1 to R4C3, 5R2C) assembled from them;
- system components (heat pump COP, emitter, tank, battery, EV, PV) with smooth equations.

trano's generator connects these components exactly as it connects Buildings components today.

**Pipeline**: trano YAML → connected components (Modelica) → **rumoca** → CasADi explicit ODE → **do-mpc** or
CasADi `Opti`. On that one CasADi model this gives:

- simulation (CVODES);
- moving-horizon estimation of states and parameters, or batch identification;
- nonlinear MPC on IPOPT.

**The rumoca step needs a release that is not out yet.** rumoca main (unreleased 0.10.3, commit `33c2114`, built
from source here) exports connected models. The released 0.10.2 does not. Until 0.10.3 is released, the same
pipeline works with **pymoca 0.12 plus symbolic elimination** of the algebraic variables (about 30 lines, verified
in section 1b).

**OpenModelica runs the same `.mo` file unchanged**:

- simulation;
- its native IPOPT optimizer, as a cross-check;
- the detailed Buildings/IDEAS output of the same YAML, as the reference plant.

**Why:**

- **Connectors cost no accuracy and no speed on this route.** With rumoca main, the connected house matches the
  flat `building_mpc` to 9 digits:
  - OCP objective 19.592617089 €;
  - the same estimated capacities;
  - the same 7-day simulation result.

  The export takes 0.15 s, and warm MPC solves take 0.15–0.17 s (flat: 0.16 s). This holds for both `Light` and
  the MSL version.
- **Accuracy comes from the RC structure and its parameters, not from the modelling style**: TEASER / VDI 6007,
  identification from data. Connectors only make the models composable.
- **Connected models are fast only if the algebraic variables are eliminated before optimization**, as rumoca main
  and the pymoca route do. rtc-tools keeps the 36 algebraic variables at every collocation point: same objective,
  but 8–9 s per solve instead of 1.55 s for the flat model.
- In the flat-model benchmark (section 1), the CasADi (+ do-mpc) pipelines were the only ones that did all four
  tasks:
  - simulation in 0.11 s for 7 days;
  - 48 h OCP;
  - warm MPC re-solves in 0.16–0.30 s;
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
| P2 | rumoca → CasADi → **do-mpc 5.1.2** (collocation, degree 2) → IPOPT | trano Modelica | 0.54 s | 1.6 s setup + 2.6 s | 0.30 s | **MHE**, 1-day window, 0.076 s per step → **1.303 / 0.805** | 19.593 € | ✅ all four tasks, ready-made MHE + MPC. ⚠️ set `terminal_bounds` too: `bounds` do not constrain the last state, and without them the plan was 1 K too cold at the end of the horizon (19.352 €) |
| P3 | **OpenModelica 1.26.9** `optimize()` (Radau IIA, 3 points) → IPOPT 3.14 | trano Modelica, unchanged | 0.02 s (+0.9 s compile) | **0.42 s** (24 iterations; +0.9 s compile) | 0.43 s per run (new process, no warm start) | ❌ parameters cannot be decision variables; a constant-state workaround ended in IPOPT "Restoration Failed" | 19.599 € | ✅ simulate and OCP; ❌ estimation; MPC only through scripting |
| P4 | Python translation of the flat equations → **Pyomo 6.10** `dae` (Radau, 3 points) → IPOPT 3.13 | generated Python | 1.3 s | 5.6 s | 0.76 s (rewrites the `.nl` file at every solve) | batch, 1.3 s → **1.2996 / 0.8027** | 19.593 € | ✅ all four tasks, slower loop |
| P5 | Python translation → **GEKKO 1.3.2** with `remote=False` | generated Python | 11.2 s | ❌ the local binary has no IPOPT ("solver 3 not supported") and falls back to APOPT: 24 steps take 78 s, 192 steps were still running after 10 min | – | not run | – | ❌ IPOPT only on the public BYU server, so model data would leave the machine |
| P6 | trano Modelica → pymoca 0.9.3 → **rtc-tools 2.8.1** (collocation) → IPOPT | trano Modelica (`input Real` instead of the `Trano.MPC.RealInput` connector) | not tested | 5.2 s on first run (pymoca compile), 1.55 s cached | 1.55 s (rebuilds the problem every run) | ❌ no estimator built in | 19.628 € | ◐ OCP and MPC; DAEs allowed |
| P7 | ModelingToolkit.jl + ModelPredictiveControl.jl (Julia, MIT) | generated Julia, or Base Modelica | – | – | – | – | – | not run: the Julia download hosts are blocked in this environment; desk research only (section 2) |

**Cross-checks:**

- The final `space_001_Ti` of the 7-day simulation agrees to within 0.04 K across all pipelines
  (290.49–290.53 K).
- The OCP objectives agree within discretisation error: 19.593–19.599 €, and rtc-tools 0.2 % higher.
- I replayed the CasADi RK4 and do-mpc plans through CVODES: both keep `Ti ≥ 293.150 K`.
- The Python translation used for P4 and P5 is a regex over `der(x) = expr;`, which only works because the
  `mpc` contract forbids every construct that would need a real compiler. trano could therefore emit CasADi,
  Pyomo or JuMP code directly, without going through rumoca.

## 1b. Connector-based models: which pipeline accepts them

**Model.** The same three-zone R3C2 house, with the same parameter values, assembled from connected components
like trano does:

- [`Light`](prior_art/Light.mo): about 100 lines, its own `HeatPort` connector, no MSL. It has 54 `connect()` equations.
- `HouseMSL`: the same house built from `Modelica.Thermal.HeatTransfer` (MSL 4.0) components.

**Tasks.** Identical to section 1.

| Pipeline | `Light` (own connectors) | `HouseMSL` (MSL `HeatTransfer`) | Structure the solver sees | OCP objective | Speed | Estimation and MPC |
|----------|--------------------------|---------------------------------|---------------------------|---------------|-------|--------------------|
| rumoca 0.10.2 → CasADi | compiles in 0.07 s; ❌ `to_casadi()` | ❌ MSL constants do not resolve | 6 states + **115 algebraic** (no alias elimination) | – | – | – |
| **rumoca main** (unreleased 0.10.3, `33c2114`, built from source) → CasADi | ✅ export 0.15 s | ✅ export 5.3 s (loads MSL) | 6 states; the 115 / 149 algebraic variables are eliminated into the ODE (MX function, `.expand()` gives SX for do-mpc) | **19.592617089 €**, identical to the flat model | 7 days 0.3 s; OCP 4.9 s; **warm MPC 0.15–0.17 s** | ✅ batch estimation 1.2996 / 0.8027, identical to the flat model (10–15 s: the MX function is not expanded) |
| **pymoca 0.12 → CasADi, algebraics eliminated symbolically → explicit ODE** | ✅ | ❌ `KeyError` on the MSL `RealInput` connector (both binding styles) | 6 states (36 algebraic eliminated) | **19.592617089 €**, identical to the flat model | flatten + eliminate 0.25 s; 7 days 0.12 s; OCP 5.6 s; warm MPC 0.34 s | ✅ batch estimation 1.2996 / 0.8027, identical to the flat model; ✅ do-mpc NMPC on the same function: 19.5925 €, warm solve 0.81 s (CasADi 3.6, elimination not optimised yet) |
| OpenModelica `optimize()` | ✅ | ✅ | index reduction + alias elimination | 19.5988 € (Light) / 19.5988 € (MSL), same as the flat model | OCP 0.59 s / 0.66 s | ❌ estimation; MPC by scripting |
| rtc-tools 2.8.1 (pymoca 0.9.3) | ✅ | not tested (same pymoca connector code) | DAE kept: 36 algebraic at every collocation point | 19.628 €, same as the flat model | **8–9 s per solve** (flat: 1.55 s) | ❌ estimation |
| ModelingToolkit.jl (acausal, `ModelingToolkitStandardLibrary.Thermal`) | – | – | `structural_simplify` (alias elimination, tearing) | – | – | not tested (Julia hosts blocked); on paper the native acausal option |

**pymoca gap.** It does not generate the flow equation of a top-level connector: an unconnected `HeatPort` declared
in the top model leaves the system one equation short. Connect components directly; trano's generator does this
anyway.

**Rules for the trano component library**, so that every pipeline above accepts it:

1. **Connectors.** Define its own connectors (`connector HeatPort Real T; flow Real Q_flow; end HeatPort;`). Use
   plain `input Real` for signals; no short-class connectors like MSL's `RealInput = input Real`.
2. **Nodes and algebraics.**
   - Every node that a `connect()` set joins has a capacity.
   - A massless node (for example the 5R1C surface node) joining three or more resistors is a linear algebraic loop.
     rumoca main rejects it (`residual_equations`, tested). pymoca + elimination solves it exactly (tested against
     the hand solution). Write the closed-form node temperature inside the component instead, and the model works
     with every route.
   - Truly nonlinear algebraic loops keep a DAE: do-mpc supports algebraic states (`_z`), at the rtc-tools speed
     cost.
3. **No discontinuities.** No events, `if`, `when`, tables or external functions. Use smooth expressions, and pass
   weather and schedules in as inputs.
4. **Explicit parameters.** Parameters are literal values on the components, so that they stay symbolic for
   identification.

## 2. Toolchain landscape

These entries come from desk research, except where a row of section 1 is cited.

| Toolchain | License, latest release | IPOPT | Simulate | Estimate | MPC | Modelica in | Notes |
|-----------|-------------------------|-------|----------|----------|-----|-------------|-------|
| [rumoca](https://github.com/cognipilot/rumoca) | Apache-2.0; 0.10.2 (2026-10-04) | via CasADi or JAX export | native, but **no time-varying inputs yet** ("roadmap §8") | `objective_gradient` only | – | native compiler; 64 % of MSL 4.1 examples simulate | Alpha. In 0.10.2, `casadi-ode` accepts **explicit ODEs only**. **Main (0.10.3-dev) exports connector models, including MSL `HeatTransfer`, with algebraic variables eliminated** (sections 1b and 4). No IPOPT or MPC roadmap. |
| [do-mpc](https://github.com/do-mpc/do-mpc) | LGPL-3; 5.1.2 (2026-09) | ✅ | ✅ CVODES/IDAS, DAE | ✅ EKF, MHE with parameters | ✅ NMPC, robust | via a CasADi model | **Best single Python package for the four tasks** (P2). Needs an SX model (`.expand()`); state bounds need `terminal_bounds` as well |
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
| [FastBuildings](https://github.com/open-ideas/FastBuildings) | Modelica License 2 | 2022-12 | about 40 grey-box single-zone RC variants | ◐ MSL `HeatTransfer` plus `replaceable`, `inner`/`outer` and legacy IDEAS imports. The connectors are accepted by rumoca main, not by 0.10.2; the IDEAS imports and `inner`/`outer` are not tested | ✓ · ◐ · · · · · · | **borrow** model structures for identification |
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

## 4. What rumoca accepts (feature probe)

| Construct | 0.10.2: compiles | 0.10.2: `to_casadi()` | main (0.10.3-dev): `to_casadi()` |
|-----------|------------------|-----------------------|----------------------------------|
| flat explicit ODE (`der(x) = f(x, u, p)`) | ✅ | ✅ | ✅ |
| `max(·)` / `min(·)` | ✅ | ✅ | not re-tested |
| algebraic variable, `output` | ✅ | ❌ `residual_equations` | ✅ |
| `extends` + `replaceable` (aliases) | ✅ | ❌ `residual_equations` | ✅ |
| `connect()`: own connectors, or MSL `Thermal.HeatTransfer` | ✅ (own connectors only) | ❌ `residual_equations` | ✅ |
| `for` loop over an array state | ✅ | ❌ `dynamic_derivative_subscripts` | ✅ |
| linear algebraic loop (massless node joining three resistors) | ✅ | ❌ `residual_equations` | ❌ `residual_equations` |
| `if` expression, `when` | ✅ | ❌ `events` | ❌ `events` |
| user `function` | ✅ | ❌ `PureCall` | not re-tested |
| `Modelica.Constants`, MSL in general | ❌ unresolved `ModelicaServices.Machine.*` | – | ✅ resolves (MSL `HeatTransfer` house compiles) |
| IBPSA (`ReducedOrder`, `ISO13790`) | ❌ resolve failed | – | ❌ resolve failed |

**Implications:**

- trano's flat `building_mpc` contract matches 0.10.2.
- A connected library needs rumoca ≥ 0.10.3 with capacitive nodes only, or the pymoca route.
- Events stay out in both versions: use smooth expressions.

## 5. Inputs to Phase 2

1. **Create the trano component library**: `Trano.MPC` components with their own `HeatPort` and the rules of
   section 1b. Generate the MPC model by connecting components, as trano does for Buildings/IDEAS, rather than by
   writing equations into one flat class. Start from the `Light` prototype (zone templates R1C1 to R4C3, 5R2C) and
   add system components: heat pump with COP, emitter, DHW tank, battery, EV, PV.
2. **Front end.**
   - Move to rumoca ≥ 0.10.3 once it is released. Main is verified here on commit `33c2114`; you could pin that
     commit as a git dependency until then.
   - Fallback: pymoca 0.12 + symbolic elimination (needs CasADi < 3.7).
   - Keep the rumoca translation tests for every generated model. Add a test that the connected model and the flat
     `building_mpc` produce the same right-hand side.
3. **Add a do-mpc adapter**: `MPCModelInterface` + the CasADi model → `do_mpc.model.Model`. Map:
   - controls to `_u`;
   - disturbances to `_tvp`;
   - selected parameters to `_p`, so the MHE can estimate them.

   Ship it as an optional extra and test simulate, MHE and MPC as in P2.
4. **Test every generated model in OpenModelica as well**: simulation, plus an optional OCP cross-check with its
   native optimizer. Validate against trano's Buildings/IDEAS output as the plant.
5. **Borrow, don't fork**:
   - TEASER / VDI 6007 parameterisation and the IBPSA validation cases for the RC parameters;
   - OCHRE / HiSim maps for heat pump, water heater, battery and EV;
   - mesido / pycity_scheduling formulations when district heating is added.
6. **Re-evaluate the Julia stack** (ModelingToolkit, acausal components + ModelPredictiveControl.jl) once its
   package hosts are reachable. It is the only free alternative with native acausal modelling, KF/UKF/MHE and
   economic MPC.
