# Multilayer zones and hydronic circuits for MPC (`mpc` library)

The [RC models](rc_model_predictive_control.md) of the `mpc` library are small and fast, but they lump a zone
into one to three capacities and leave the heating system out. With `--envelope multilayer`, the `mpc` library
instead assembles, from the same YAML file, a model **close to the Buildings library** that **still translates
into a CasADi ODE** for simulation, estimation and model predictive control with IPOPT:

* every zone has the physics of `Buildings.ThermalZones.Detailed.MixedAir`: layered walls, floors and roofs,
  windows with the angular optics of their glazing, interior and exterior convection, long-wave exchange with
  the sky and between the room surfaces, the transmitted solar radiation distributed on the room surfaces,
  infiltration, walls and floors shared between zones;
* the hydronic circuits of the YAML are modelled: radiators (EN 442), radiator valves, mixing and diverting
  three-way valves, junctions, pumps, sensors, gas boilers and air-to-water heat pumps with their buffer;
* balanced ventilation units with heat recovery (trano's `system_d`) and PV arrays.

```bash
trano create-model house.yaml mpc --envelope multilayer   # writes house.mo and house.mpc.json
trano simulate-model house.yaml mpc --envelope multilayer # runnable, like the other libraries
```

or in Python:

```python
from pathlib import Path

from trano.data_models.conversion import convert_network
from trano.elements.library.library import Library
from trano.mpc.modelica import network_interface

library = Library.from_configuration("mpc").model_copy(update={"envelope": "multilayer"})
network = convert_network("house", Path("house.yaml"), library=library)
modelica_model = network.model()
interface = network_interface(network)  # states, inputs, parameters and zones of house.building_mpc
```

## How the model is built

`house.building_mpc` is made of instances of the components of `Trano.MPC.Components`, a small Modelica library
written for optimal control and joined with `connect()` statements, as a Modelica library is used. Every
component keeps a few rules, so that the assembled model translates into an explicit ODE
(`xdot = f(x, u, p)`) with [rumoca](https://github.com/cognipilot/rumoca) 0.10.3 or newer:

* every node joined by connections has a heat capacity, or is solved in closed form inside its component
  (the radiant star node of a room, the mixing of water flows);
* the equations are smooth: no events, tables, `if` or `min`/`max` (power laws and table interpolations are
  regularised);
* the parameters are literal values, so that they stay symbolic in CasADi (identification).

| Package | Components |
|---|---|
| `Thermal` | `HeatCapacitor`, `ThermalResistor`, `ThermalConductor`, `StarNode` (radiant star of a room) |
| `Envelope` | `InteriorConvection`, `ExteriorConvection` (wind), `SkyRadiation`, `WindowSolar`, `WindowExteriorConvection`, `SolarDistribution` |
| `Hydronics` | `Radiator` (EN 442), `FlowValve`, `Valve` (equal percentage), `ThreeWayValve` (mixing), `DivertingValve`, `Splitter`, `Mixer`, `Pump`, `TemperatureSensor`, `Boiler`, `HeatPump`, `PrescribedSupply`, `Reference` |
| `Ventilation` | `HeatRecoveryUnit` (balanced ventilation with a constant-effectiveness heat recovery) |
| `Electrical` | `Photovoltaic` |
| `Sources` | prescribed and fixed temperatures, heat flows |

### Water circuits

The circuits follow the system graph of the YAML, in the flow direction. Flows are **set**, not computed from
pressures: each radiator valve sets the flow of its branch, a mixing valve the share of supply water at its
outlet, mass balances give the rest. This is how trano's Buildings models behave when the pump holds the
pressure difference, and it keeps the model explicit; the flow of a branch is then the natural decision of an
MPC (the valve opening that realises it is a local control). A split valve, or any element with several inlets
or outlets, becomes an ideal mixer or splitter; one `Reference` (the expansion vessel) per closed circuit drops
the redundant mass balance.

Radiators are `RadiatorEN442_2` of Buildings: the conductance of each element is solved from the nominal
conditions as Buildings does at initialisation, the heat is split between the zone air and its radiant star.
Boilers are `BoilerPolynomial` (efficiency curve, water and dry mass, losses). Heat pumps are the
`ModularReversible.AirToWaterTableData2D` of trano's `air_water_heat_pump` production: the interpolation of the
EN 14511 tables of the Vitocal 251-A08 in the source temperature, smoothed at the table knots, scaled to the
nominal capacity of the YAML, with the buffer tank. Boilers, heat pumps, fans and PV arrays carry an energy meter
state (`EFue`, `EEle`, `EFan`, `EPro` in kWh), so that an optimal control problem integrates their consumption
and production exactly.

### Ventilation and PV

A ventilation unit (`system_d`) supplies each zone with the nominal flow of its supply duct, scaled by its speed,
at `TOut + eps*(TExtract - TOut)` plus the heat of its fans, the extract air being the zone air mixed by flow (as
trano's Buildings unit: constant-pressure fans and a constant-effectiveness heat exchanger). A PV array produces
`A*fAct*eta*eta_DCAC*(HDir + HDif)` on the irradiance of its orientation (Buildings `PVSimpleOriented`); the
optimal control counts its production against the electricity of each step (see below).

### Inputs and states

| Input | Role |
|---|---|
| `TOut`, `TBlaSky`, `winSpe`, `winDir` | weather (disturbances) |
| `HDir_<orientation>`, `HDif_<orientation>`, `cosInc_<orientation>` | direct and Perez diffuse irradiance, cosine of the incidence angle on each orientation of the facades (disturbances) |
| `<zone>_QInt` | internal gains (disturbance) |
| `<zone>_QHea` | heat of an ideal emitter (control), for zones without a water circuit |
| `<valve>_m_flow` | flow of a radiator branch [kg/s] (control, between 0 and the nominal flow of the valve) |
| `<three-way valve>_y` | share of supply water (control) |
| `<boiler>_y` | boiler part load ratio, or heat pump compressor speed (control) |
| `<air handling unit>_y` | air flow of a ventilation unit relative to its design flow (control; its bounds hold it at the design flow, as Buildings runs it, unless you relax them) |

The interface (`house.mpc.json`) lists them in the order of the CasADi vectors, with their role, unit, bounds and
the zone or system they act on; the zone air temperatures are the states `<zone>_air.T`.

## Optimal control with IPOPT

`trano.mpc.optimal_control.OptimalControl` sets up and solves the optimal control problem of any model of the
`mpc` library: Radau collocation of the ODE (stable for the stiff water nodes whatever the step), IPOPT, a warm
start from a simulation of the model (CVODES), control bounds from the interface, soft comfort bounds on the zone
air temperatures and the energy cost read from the meters of the systems. Electricity is netted over each step:
the PV production first covers the heat pump and the fans, the rest is sold at `feed_in_price` (smoothly, so that
IPOPT sees a differentiable cost); the boiler fuel has its own `fuel_price`.

```python
import numpy as np
import rumoca

from trano.mpc.optimal_control import KELVIN, OptimalControl, OptimalControlSettings

export = rumoca.Session().loads(modelica_model, model=interface.model).to_casadi()
ocp = OptimalControl(interface, export.rhs, OptimalControlSettings(time_step=3600))

hours = np.arange(24)
comfort = np.where((hours >= 7) & (hours < 22), KELVIN + 20, KELVIN + 16)
disturbances = ...  # (len(interface.disturbances), 24): forecasts of the disturbance inputs
result = ocp.solve(np.array(interface.initial_state), disturbances, comfort, price=np.full(24, 0.10))
result.controls  # branch flows, valve positions, boiler part load ratio, hour by hour
```

On the single-zone hydronic test house the solution lets the zone float down to the night set point, fires the
boiler to reach 20 °C exactly at 7:00 and holds the bound, with the branch flow and the mixing modulated
together; the trajectory of the model under these controls (CVODES) stays within 0.2 K of the collocated one.
IPOPT uses the SPRAL linear solver when CasADi ships it (MUMPS otherwise, and as a fallback): 24 hourly steps take
about 4 s for that house and about 40 s for the six-zone heat pump house (111 states).

The same discretization gives **state estimation** and the **closed loop**:

```python
from trano.mpc.optimal_control import MovingHorizonEstimation, receding_horizon

# the states over a past window closest to the measured zone temperatures (walls, water and buffers follow)
estimator = MovingHorizonEstimation(interface, export.rhs)
estimate = estimator.estimate(prior_state, applied_controls, past_disturbances, measured_air_temperatures)

# MPC: solve, apply the first controls to the plant, repeat (the plant defaults to the model itself)
loop = receding_horizon(ocp, estimate.current, disturbances, comfort, price, horizon=12)
```

## How close to Buildings

`python -m validation.mpc` simulates the Buildings model of a YAML file and **replays** what its systems did
(the heat of ideal emitters, or the branch flows, valve positions and boiler or heat pump controls) through the
runnable `mpc` model, then compares the zones and the water temperatures. See the
[validation page](../validation/mpc_library.md) for the results.

## Limits

* Ventilation units are balanced (supply = extract): the small imbalance of Buildings, set by the pressures of
  the zones and of their infiltration paths, is not modelled.
* Boilers with a storage tank, water-to-water heat pumps, chillers, domestic hot water, batteries and EV
  chargers are not modelled yet: a model that leaves a system of the YAML out raises an
  `UnmodelledSystemWarning` naming it.
* Flows are set by the valves; the pressure-driven sharing of flow between branches is not modelled.
