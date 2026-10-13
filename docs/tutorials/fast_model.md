# FastModel: models close to Buildings for MPC

The [RC models](rc_model_predictive_control.md) of the `mpc` library are small and fast, but they lump a zone
into one to three capacities and leave the heating system out. The `FastModel` library instead turns the same YAML
file into a model **close to the Buildings library** that **still translates into a CasADi ODE** (with
[rumoca](https://github.com/cognipilot/rumoca)) for simulation, state estimation and model predictive control
with IPOPT:

* every zone has the physics of `Buildings.ThermalZones.Detailed.MixedAir`: layered walls, floors and roofs,
  windows with the angular optics of their glazing, interior and exterior convection, long-wave exchange with the
  sky and between the room surfaces, the transmitted solar radiation distributed on the room surfaces,
  infiltration, walls and floors shared between zones;
* the hydronic circuits of the YAML are modelled: radiators (EN 442), valves, mixing three-way valves, split
  valves, pumps, sensors, gas boilers and air-to-water heat pumps with their buffer;
* balanced ventilation units with heat recovery (trano's `system_d`) and PV arrays.

```bash
trano create-model house.yaml FastModel    # writes house.mo
trano simulate-model house.yaml FastModel  # runnable, like the other libraries
```

## A library like Buildings and IDEAS

`FastModel` is used exactly as trano uses Buildings and IDEAS: a Modelica library and one template per element of
the YAML, without code of its own.

* `trano/library/FastModel.mo` holds the components (the physics), embedded in every generated package;
* `trano/elements/library/models/fast_model/*.yaml` hold the templates that declare a component for each element
  of the YAML, with its ports, as the templates of the other libraries;
* the `FastModel` entry of `library.json` writes the constructions and glazings as records of a `Data` package,
  takes the parameter names of Buildings, and the package template `templates/fast_model.jinja2`.

The walls, windows and floors are elements of their own connected to the bus of their zone, as in IDEAS: each
`OuterWall`, `Window`, `BoundaryWall` (floor on ground or over outdoor air), `AdiabaticWall` or `InternalWall`
connects its `propsBus_a` to a `propsBus[i]` of its `Zone`. The bus carries the convective and radiative
exchanges with the room, the properties of the surface that the zone needs (area, emissivity, solar absorptance
and transmittance), the solar radiation a window transmits, and the weather.

| Package | Components |
|---|---|
| `BoundaryConditions` | `Weather` (solar position and Perez sky of the site), `TiltedSurface` (irradiance on a surface) |
| `Envelope` | `OuterWall`, `Window`, `BoundaryWall`, `AdiabaticWall`, `InternalWall` |
| `Zones` | `Zone`, `VentilatedZone` |
| `Hydronics` | `Radiator` (EN 442), `Valve`, `ThreeWayValve`, `Junction`, `Pump`, `TemperatureSensor`, `Boiler`, `HeatPump` |
| `Ventilation` | `HeatRecoveryUnit`, `Duct` |
| `Electrical` | `Photovoltaic` |
| `HVAC` | `IdealHeatingCooling` |
| `Data` | `Construction`, `Glazing` (records written by trano) |

The generated package holds two models:

* `building_mpc`, the building: an explicit ODE whose inputs are the weather data, the internal gains and the
  controls of the systems. It contains no table, no event and no component of another library;
* `building`, the runnable model: `building_mpc` fed with the weather file (the Buildings reader, which also gives
  `building_mpc` the location of the site), the occupancy schedules, and the controls replayed from the external
  data of the network (each held at 1 without data).

### Rules for an explicit ODE

Every component keeps a few rules, so that rumoca translates the assembled model into `xdot = f(t, x, u, p)`:

* every node joined by connections has a heat capacity, or is solved in closed form inside its component (the
  radiant star of a room, the mixing of water flows);
* the equations are smooth: power laws and table interpolations are regularised, conditions are inside functions;
* flows are **set**, not computed from pressures: each valve sets the flow of its branch, a mixing valve the share
  of supply water, mass balances the rest; the production (boiler, heat pump) or the ventilation unit closes the
  balance of its circuit, one per circuit;
* a port with several connections is an array, one element per connection, sized by the template from the
  connections trano made: the inlets of a component mix, its outlets leave at its outlet temperature;
* parameters computed by functions (the conductance of a radiator solved as Buildings does at initialisation, the
  polynomial fits of the window optics, the scaling of the heat pump tables) take values settled at translation.

### Inputs of `building_mpc`

| Input | Role |
|---|---|
| `weather_inputs.TDryBul`, `.TBlaSky`, `.winSpe`, `.winDir`, `.HDirNor`, `.HDifHor`, `.HGloHor` | weather data, as in a weather file or a forecast (disturbances) |
| `<occupancy>.gai.QCon`, `.QRad` | convective and radiant internal gains per floor area [W/m2] (disturbances) |
| `<valve>_y.y` | flow of a radiator branch relative to its nominal flow (control, 0 to 1) |
| `<three-way valve>_y.y` | share of supply water (control) |
| `<boiler>_y.y` | boiler part load ratio, or heat pump compressor speed (control) |
| `<ventilation unit>_y.y` | air flow relative to the design flow (control held at 1 by its bounds, as Buildings runs it) |
| `<emitter>_Q.Q_flow` | heat of an ideal emitter [W] (control, between its bounds) |

The solar position and the irradiance on each surface are computed inside the model from the time (the clock time
of the year) and the site, so that a forecast only needs the weather channels. The meters of the systems are
states in kWh: `EFue` (boiler fuel), `EEle` (heat pump electricity), `EFan` (fans), `EPro` (PV production).

## Simulation, estimation and optimal control: `trano.optimization`

The CasADi side lives in `trano.optimization`, a package that depends on rumoca, CasADi, NumPy and pydantic only
(no other module of trano), so that it can be moved to a package of its own. It reads everything from the Modelica
model: the names of the states, inputs and parameters in the order of the CasADi vectors, their units, bounds and
descriptions. The roles of the variables follow simple conventions (`Roles.infer`): an input with a `min` and a
`max` is a control, any other input a disturbance; the states named `TAir` are the comfort temperatures; the
states in kWh are meters.

```python
from pathlib import Path

import numpy as np

from trano.data_models.conversion import convert_network
from trano.elements.library.library import Library
from trano.optimization import KELVIN, CasadiModel, OptimalControl, OptimalControlSettings

network = convert_network("house", Path("house.yaml"), library=Library.from_configuration("FastModel"))
model = CasadiModel.from_modelica(network.model(), "house.building_mpc")
ocp = OptimalControl(model, settings=OptimalControlSettings(time_step=3600))

hours = np.arange(24)
comfort = np.where((hours >= 7) & (hours < 22), KELVIN + 20, KELVIN + 16)
disturbances = ...  # (len(ocp.roles.disturbances), 24): forecasts of the weather and gains
result = ocp.solve(model.description.initial_state, disturbances, comfort, price=np.full(24, 0.10),
                   start_time=t0)  # the model time of the first step: the solar position depends on it
result.controls  # branch flows, valve positions, boiler part load ratio, hour by hour
```

`OptimalControl` collocates the ODE on Radau points (stable for the stiff water nodes whatever the step), solves
with IPOPT (SPRAL when CasADi ships it, MUMPS otherwise), warm-starts from a simulation of the model (CVODES), and
costs the energy read from the meters: the PV production first covers the electricity of the step, the rest is sold
at `feed_in_price`; the boiler fuel has its own `fuel_price`. The same discretization gives the moving horizon
estimation and the closed loop:

```python
from trano.optimization import MovingHorizonEstimation, receding_horizon

estimator = MovingHorizonEstimation(model)
estimate = estimator.estimate(prior_state, applied_controls, past_disturbances, measured_air_temperatures)
loop = receding_horizon(ocp, estimate.current, disturbances, comfort, price, horizon=12, start_time=t0)
```

The parameters computed from others (a conductance from a thickness and a conductivity) are parameters of their
own in the CasADi function, at their values: an identification adjusts the effective ones (conductances,
capacities).

## How close to Buildings

`python -m validation.mpc` simulates the Buildings model of a YAML file and **replays** what its systems did
(the heat of ideal emitters, or the branch flows, valve positions and boiler or heat pump controls) through the
runnable `FastModel` model, then compares the zones, the water temperatures and the meters. See the
[validation page](../validation/fast_model.md) for the results.

## Limits

* One production (boiler or heat pump) per hydraulic circuit, and mixing three-way valves (the bypass from the
  return): diverting valves are not modelled yet.
* Ventilation units are balanced (supply = extract) and of the `system_d` kind.
* Boilers with a storage tank, water-to-water heat pumps, chillers, domestic hot water, batteries, EV chargers and
  VAV systems are not modelled: generating a model with them stops with a message naming them.
* Flows are set by the valves; the pressure-driven sharing of flow between branches is not modelled.
