# RC models for model predictive control (CasADi + IPOPT)

Detailed Modelica libraries (Buildings, IDEAS, AixLib) are well suited to simulation but not to
optimisation: they rely on events, tables, media models and large algebraic loops that optimal
control solvers cannot handle. Trano can also generate **simple resistance-capacitance (RC) models**
from the same building description. They are written in a Modelica subset that translates
directly into a symbolic [CasADi](https://web.casadi.org/) ODE, so they can be used for MPC with
[IPOPT](https://coin-or.github.io/Ipopt/).

The MPC features need optional dependencies:

```bash
pip install 'trano[mpc]'   # casadi + rumoca (Modelica -> CasADi compiler)
```

## Available zone models

| Model      | States       | Structure                                                                         | Reference                                  |
|------------|--------------|-----------------------------------------------------------------------------------|--------------------------------------------|
| `R1C1`     | Ti           | one lumped capacity, indoor-outdoor and indoor-ground resistances                  | Bacher & Madsen (2011), *Ti* model          |
| `R3C2`     | Ti, Te       | indoor air + envelope/mass; windows and ventilation connect indoor air to outdoor | Bacher & Madsen (2011), *TiTe*; Harb et al. (2016) |
| `R4C3`     | Ti, Te, Th   | `R3C2` + heat emitter capacity (radiator or floor heating lag)                    | Bacher & Madsen (2011), *TiTeTh*            |
| `ISO13790` | Ti, Tm       | ISO 13790 5R1C network with a capacitive air node (5R2C)                           | ISO 13790:2008, annex C                     |

All the models share the same inputs:

* `TOut` outdoor air temperature [K] and `HGlo` global horizontal irradiance [W/m²] (building level),
* `<zone>_QInt` internal gains [W] (disturbance) and `<zone>_QHea` heating power [W] (control),

and the ground temperature `TGro` is a parameter. Adjacent zones are coupled through the
conductance of the internal walls: `H_<zone a>_<zone b>*(Ti_b - Ti_a)`.

For example, the `R3C2` zone equations are:

```modelica
der(Ti) = ((Te - Ti)/Rie + (TOut - Ti)/Ria + gA*HGlo + QInt + QHea)/Ci;
der(Te) = ((Ti - Te)/Rie + (TOut - Te)/Rea + (TGro - Te)/Reg + aE*HGlo)/Ce;
```

In the ISO 13790 model the massless surface node of the standard is eliminated analytically, so
the model stays an explicit ODE while matching the standard heat balances.

### Why these models are CasADi/IPOPT compatible

The generated Modelica code only uses:

* flat, scalar models (no connectors, no sub-components, no arrays),
* explicit state equations `der(x) = f(x, u, p)` without algebraic variables,
* parameters bound to literal values (they remain symbolic in CasADi, which enables parameter
  identification),
* smooth expressions: no events, `if`, `min`/`max`, tables or external functions,
* no dependency on any other Modelica library (not even the Modelica Standard Library).

The resulting dynamics are linear in the states and inputs, so the MPC problem is a convex
quadratic program that IPOPT solves in a few iterations.

## From a Trano building to an RC model

The RC parameters are derived from the geometry and the constructions of the Trano YAML file:
one RC zone per space, ISO 6946 surface resistances, opaque elements split in two halves around
the mass node, windows and ventilation directly between indoor and outdoor air, and half of the
internal walls capacity assigned to each adjacent zone. All the assumptions are gathered in
`EstimationSettings`.

```python
from trano.mpc import EstimationSettings, RCModelType, rc_building_from_yaml

building = rc_building_from_yaml(
    "three_zones_ideal_heaters.yaml",
    model_type=RCModelType.r3c2,
    settings=EstimationSettings(air_change_rate=0.4),
)
modelica_source = building.to_modelica("TranoRC")  # stand-alone Modelica package
```

The same can be done from the command line:

```bash
trano create-rc-model three_zones_ideal_heaters.yaml --model-type R4C3
```

The generated package `TranoRC` contains the library of single-zone models (`TranoRC.Zones.*`)
and the flat multi-zone model of the building (`TranoRC.Building`). It can be simulated with any
Modelica tool (OpenModelica, Dymola) and translated to CasADi:

```python
model = building.to_casadi()  # Modelica -> CasADi through rumoca
model.state_names  # ('space_001_Ti', 'space_001_Te', ...)
model.control_names  # ('space_001_QHea', 'space_002_QHea', 'space_003_QHea')
model.disturbance_names  # ('TOut', 'HGlo', 'space_001_QInt', ...)

f = model.continuous_dynamics()  # casadi.Function xdot = f(x, u, d, p)
F = model.discrete_dynamics(time_step=900)  # RK4 with stable sub-steps: x+ = F(x, u, d, p)
```

!!! note
    The derived parameters are physically consistent initial guesses. For a real building, calibrate
    them on measurements: `F` keeps the parameters `p` symbolic, so a least-squares identification
    problem can be written with `casadi.Opti` and solved with IPOPT.

## A simple economic MPC

`ModelPredictiveController` minimises the heating cost while keeping the indoor temperatures
inside a comfort band, with soft constraints so that the problem is always feasible:

```text
min   Σk price[k]·ΣQHea[k]·Δt  +  wc·Σ(sLow + sHigh)  +  wq·Σ(sLow² + sHigh²)  +  ws·Σ(ΔQHea/QMax)²
s.t.  x[k+1] = F(x[k], QHea[k], d[k], p)
      TLow[k] - sLow[k] ≤ Ti[k+1] ≤ THigh[k] + sHigh[k],   sLow, sHigh ≥ 0
      0 ≤ QHea[k] ≤ QMax
```

The problem is built once as a parametric NLP (multiple shooting); at each step only the initial
state and the forecasts change, and IPOPT is warm started with the shifted previous solution.

```python
import numpy as np

from trano.mpc.controller import Forecast, MPCSettings, ModelPredictiveController, run_closed_loop

K = 273.15
hour = np.arange(72) % 24
occupied = (hour >= 7) & (hour < 22)
forecast = Forecast(
    outdoor_temperature=K + 2 + 5 * np.sin(2 * np.pi * (hour - 9) / 24),
    solar_irradiance=np.clip(600 * np.sin(np.pi * (hour - 7) / 10), 0, None) * (hour <= 17),
    internal_gains=np.where(occupied, 300.0, 100.0),  # W, same for every zone
    lower_temperature=np.where(occupied, K + 20, K + 16),
    upper_temperature=np.where(occupied, K + 24, K + 26),
    price=np.where((hour >= 17) & (hour < 21), 0.40, 0.25),  # per kWh
)

controller = ModelPredictiveController(
    model,
    MPCSettings(horizon=24, time_step=3600, max_heating_power=6000),
)
solution = controller.solve(forecast.slice(0, 24))  # open loop
print(solution.status, solution.energy_cost)
print(solution.heating_power)  # (24, 3) W

result = run_closed_loop(controller, forecast, n_steps=48)  # receding horizon
print(result.energy_cost, result.comfort_violation)
```

The controller lowers the temperature at night, pre-heats the zones before the occupancy and
before the expensive price period, and keeps the comfort band during occupancy. Each IPOPT solve
takes about 0.1 s for the three zones house.

To evaluate the controller under model mismatch, simulate a different plant:

```python
import dataclasses

plant = dataclasses.replace(model, default_parameters=model.parameter_values(space_001_Ci=3e6))
result = run_closed_loop(controller, forecast, n_steps=48, plant=plant)
```

## References

* P. Bacher, H. Madsen (2011). Identifying suitable models for the heat dynamics of buildings.
  *Energy and Buildings*, 43(7), 1511-1522.
* H. Harb, N. Boyanov, L. Hernandez, R. Streblow, D. Müller (2016). Development and validation of
  grey-box models for forecasting the thermal response of occupied buildings.
  *Energy and Buildings*, 117, 199-207.
* ISO 13790:2008. Energy performance of buildings - Calculation of energy use for space heating and
  cooling (simple hourly method, annex C).
* J. A. E. Andersson et al. (2019). CasADi: a software framework for nonlinear optimization and
  optimal control. *Mathematical Programming Computation*, 11, 1-36.
