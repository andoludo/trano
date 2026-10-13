# FastModel

`FastModel` is the trano library for **optimal control**: from the same YAML file as Buildings and IDEAS, it
generates a model whose physics follow `Buildings.ThermalZones.Detailed.MixedAir` and the Buildings hydronic and
ventilation components, written so that [rumoca](https://github.com/cognipilot/rumoca) translates it into a
CasADi ODE \(\dot x = f(t, x, u, p)\). The same model simulates in OpenModelica and in CasADi, where
`trano.optimization` estimates its state and solves optimal control problems with IPOPT.

```bash
trano create-model house.yaml FastModel    # writes house.mo: building_mpc and its runnable wrapper building
trano simulate-model house.yaml FastModel  # simulates building with OpenModelica
```

This page describes the components of the library and their equations, what the library does not model, and how
close and how fast it is, ending with the most complex YAML file it simulates compared with Buildings and IDEAS.
The [FastModel tutorial](../tutorials/fast_model.md) shows how to use the model in CasADi.

## How a model is built

`FastModel` is used as trano uses Buildings and IDEAS: a Modelica library, `trano/library/FastModel.mo`, and one
template per element of the YAML file, `trano/elements/library/models/fast_model/*.yaml`. The library is embedded
in every generated package; the constructions and glazings of the YAML are written as records of its `Data`
package. The generated package holds two models:

* **`building_mpc`**, the building: an explicit ODE whose inputs are the weather data, the internal gains and the
  controls of the systems. It holds no table, no event and no component of another library: this is the model
  rumoca translates;
* **`building`**, the runnable model: `building_mpc` fed with the weather file (the Buildings reader, which also
  gives `building_mpc` the location of the site), the occupancy schedules, and the controls replayed from the
  external data of the network (each held at 1 without data).

Walls, windows and floors are elements of their own connected to the bus of their zone, as in IDEAS. Each
controllable system declares its control as a top-level input of `building_mpc` named `<element>_y` (bounded to
0..1) or `<element>_Q` (a heat flow rate, bounded by the nominal power): the controllers of the YAML (thermostatic
valves, boiler and collector controls) are not rendered, their decisions are the inputs.

| YAML element | FastModel component | Buildings counterpart |
|---|---|---|
| `spaces` | `Zones.Zone`, `Zones.VentilatedZone` (with supply and extract air) | `ThermalZones.Detailed.MixedAir` |
| `spaces` variant `infiltration` | the same, with the air change rate `ach` | `Trano.ThermalZones.MixedAirInf` |
| `external_walls`, roofs, doors | `Envelope.OuterWall` | `datConExt` of `MixedAir` |
| `windows` + `glazings` | `Envelope.Window` | `datConExtWin` of `MixedAir` |
| `floor_on_grounds` | `Envelope.BoundaryWall` (at the ground temperature, or the outdoor air) | `datConBou` + fixed temperature |
| `adiabatic_walls` | `Envelope.AdiabaticWall` | `datConBou`, open outer port |
| `internal_walls` | `Envelope.InternalWall` | `surBou` |
| `weather` | `BoundaryConditions.Weather` | `BoundaryConditions.WeatherData.ReaderTMY3` |
| `occupancy` | `Interfaces.OccupancyInputs` (gains per floor area) | trano occupancy schedule |
| `radiator` | `Hydronics.Radiator` | `Fluid.HeatExchangers.Radiators.RadiatorEN442_2` |
| `radiator` variants `ideal`, `idealbus` | `HVAC.IdealHeatingCooling` | trano ideal heater |
| `valve` | `Hydronics.Valve` | `Fluid.Actuators.Valves.TwoWayEqualPercentage` |
| `three_way_valve` | `Hydronics.ThreeWayValve` (mixing) | `Fluid.Actuators.Valves.ThreeWayEqualPercentageLinear` |
| `split_valve` | `Hydronics.Junction`, `Hydronics.Junction2` | `Fluid.FixedResistances.Junction` |
| `pump` | `Hydronics.Pump` | `Fluid.Movers.Preconfigured.SpeedControlled_y` |
| `temperature_sensor` | `Hydronics.TemperatureSensor` | `Fluid.Sensors.TemperatureTwoPort` |
| `boiler` (`without_storage`) | `Hydronics.Boiler` | `Fluid.Boilers.BoilerPolynomial` |
| `boiler` variant `air_water_heat_pump` | `Hydronics.HeatPump` | `Fluid.HeatPumps.ModularReversible.AirToWaterTableData2D` + buffer |
| `air_handling_unit` variant `system_d`, `duct` | `Ventilation.HeatRecoveryUnit`, `Ventilation.Duct` | trano system D (`ConstantEffectiveness` + 2 fans) |
| `solar.photovoltaic` | `Electrical.Photovoltaic` | `Electrical.AC.OnePhase.Sources.PVSimpleOriented` |
| `ideal_heating_cooling` | `HVAC.IdealHeatingCooling` | trano ideal heating and cooling |

### Rules for an explicit ODE

Every component keeps a few rules, so that the assembled model translates into \(\dot x = f(t, x, u, p)\):

* every node joined by connections has a heat capacity, or is solved in closed form inside its component (the
  radiant star of a room, the mixing of water flows);
* the equations are smooth (once continuously differentiable): power laws, maxima and table interpolations are
  regularised, conditions are inside functions or `noEvent`;
* flows are **set**, not computed from pressures: each valve sets the flow of its branch, a mixing valve the share
  of supply water, mass balances the rest; the production (boiler, heat pump) or the ventilation unit closes the
  balance of its circuit, one per circuit;
* a port with several connections is an array, one element per connection, sized by the template: the inlets of a
  component mix, its outlets leave at its outlet temperature;
* parameters computed by functions (the conductance of a radiator, the polynomial fits of the window optics, the
  scaling of the heat pump tables) take values settled at translation.

## Components and equations

The notation: \(T\) temperatures [K], \(Q\) heat flow rates [W], \(\dot m\) mass flow rates [kg/s], \(A\) areas,
\(c_p\) the specific heat capacity of water (4184 J/(kg.K)) or of air (1006 J/(kg.K)), \(\rho\) densities
(1000 and 1.2 kg/m3), \(\sigma\) the Stefan-Boltzmann constant.

### Smooth functions

The library replaces the non-differentiable functions of Buildings by the regularised ones of
`Buildings.Utilities.Math.Functions`:

* `regStep(x, y1, y2, ε)`: a cubic step from \(y_2\) (\(x < -\varepsilon\)) to \(y_1\) (\(x > \varepsilon\));
  `smoothMax`, `smoothMin` and `smoothLimit` are built on it;
* `positivePart(x, δ)` \(= \tfrac12\left(x + \sqrt{x^2 + \delta^2}\right)\), a smooth \(\max(x, 0)\);
* the natural convection law \(Q = A\,c\,|\Delta T|^{4/3}\) is written
  \(Q = A\,c(\Delta T)\,\Delta T\,(\Delta T^2 + \Delta T_0^2)^{1/6}\), with \(\Delta T_0 = 0.1\) K and the coefficient
  \(c\) blended smoothly between its values for a surface warmer or colder than the air.

### Weather and solar irradiance

`BoundaryConditions.Weather` takes the seven channels of a weather file or a forecast (`TDryBul`, `TBlaSky`,
`winSpe`, `winDir`, `HDirNor`, `HDifHor`, `HGloHor`) and computes the solar position from the model time, as
`Buildings.BoundaryConditions.SolarGeometry`:

\[
\delta = \arcsin\!\left(-\sin 23.45^\circ \cos\frac{2\pi (t/86400 + 10)}{365.25}\right), \qquad
t_{sol} = t - t_{zone} + \frac{\lambda\,43200}{\pi} + E(t), \qquad
\omega = \frac{2\pi}{24}\left(\frac{t_{sol}}{3600} - 12\right)
\]

\[
\cos\theta_z = \cos\varphi \cos\delta \cos\omega + \sin\varphi \sin\delta
\]

with \(E\) the equation of time, \(\varphi\) the latitude and \(\lambda\) the longitude, and the Perez
coefficients \(F_1\), \(F_2\) of the diffuse sky from the sky clearness and brightness (the eight clearness bins
joined by smooth steps). `BoundaryConditions.TiltedSurface` gives the irradiance on a surface of tilt \(\beta\)
and azimuth \(\gamma\):

\[
H_{dir} = \max(\cos\theta\, H_{dir,nor}, 0), \qquad
H_{dif} = H_{dif,hor}\left[(1 - F_1)\frac{1 + \cos\beta}{2} + F_1 \frac{\max(0, \cos\theta)}{\max(\cos 85^\circ, \cos\theta_z)} + F_2 \sin\beta\right] + H_{glo,hor}\,\rho_g\frac{1 - \cos\beta}{2}
\]

where \(\theta\) is the incidence angle on the surface and \(\rho_g = 0.2\) the ground reflectance. The site
(latitude, longitude, time zone) comes from the weather file in `building`; in `building_mpc` it is a parameter.

### Opaque surfaces

`Envelope.OuterWall`, `BoundaryWall`, `AdiabaticWall` and `InternalWall` conduct heat through the layers of their
construction, with a node at each layer boundary (\(n\) layers, \(n + 1\) nodes). The capacity of a node is half
of each adjacent layer, the conductance of a layer \(k/x\):

\[
C_j = A\left(\tfrac12 x_{j-1}\rho_{j-1} c_{j-1} + \tfrac12 x_j \rho_j c_j\right), \qquad G_j = \frac{A\,k_j}{x_j}
\]

\[
C_1 \dot T_1 = Q_{sol} - Q_{ext} - G_1 (T_1 - T_2), \qquad
C_j \dot T_j = G_{j-1}(T_{j-1} - T_j) - G_j (T_j - T_{j+1}), \qquad
C_{n+1} \dot T_{n+1} = G_n (T_n - T_{n+1}) - Q_{int} + Q_{rad}
\]

* **Outer surface** (`OuterWall`): absorbed sun \(Q_{sol} = \alpha_{sol} A (H_{dir} + H_{dif})\); exchange with the
  outside \(Q_{ext}\) as `Buildings.HeatTransfer.Convection.Exterior` (`TemperatureWind`) and
  `SkyRadiationExchange`: natural convection, forced convection
  \(h_F = 2.537\, W R \sqrt{P v / A}\) for a square surface (\(P = 4\sqrt A\)), with \(W\) = 1 windward, 0.5
  leeward and 0.75 without wind (smooth in the wind direction and speed) and \(R = 1.67\) for a rough surface, and
  the long-wave exchange with sky and ground linearised around their mean radiant temperature
  \(T_{env} = \left(F_{sky} T_{sky}^4 + (1 - F_{sky}) T_{out}^4\right)^{1/4}\), \(F_{sky} = (\pi - \beta)/\pi\):
  \(Q_{lw} = 4\sigma\varepsilon A T_{env}^3 (T_1 - T_{env})\).
* **Floor on ground** (`BoundaryWall`): the outer surface is held at the ground temperature of the YAML (10 degC by
  default), as trano does for Buildings' `datConBou`, or at the outdoor air temperature (`outdoor_air` variant).
* **Party wall** (`AdiabaticWall`): no heat crosses the outer surface.
* **Wall or floor between zones** (`InternalWall`): each face exchanges with its own zone; side b faces the other
  way (tilt \(\pi - \beta\), a floor seen from below is a ceiling).
* **Inner surface**: natural convection to the room air \(Q_{int}\) with the coefficients of Buildings
  (1.3 for a wall; 0.76 or 1.51 for a ceiling or a floor depending on whether it is warmer than the air), or a
  fixed coefficient; long-wave and solar exchange with the room \(Q_{rad}\) (see the zone).

### Windows

`Envelope.Window` models each pane of its glazing as a node, the gas gaps as conductances, and the frame as a
conductance between the room air and the outside, after `Buildings.HeatTransfer.Windows`:

\[
C_i \dot T_i = Q_{abs,i} + G_{i-1}(T_{i-1} - T_i) - G_i (T_i - T_{i+1}) \quad (\text{outer pane: } - Q_{ext},\ \text{inner pane: } - Q_{int} + Q_{rad})
\]

* \(G_i = A_{gla} / (R_{pane,i}/2 + R_{gap,i} + R_{pane,i+1}/2)\), the glass area \(A_{gla} = (1 - f_{fra})A\);
* the optics of the glazing are computed by trano from its layers with the algorithm of Buildings
  (`Buildings.HeatTransfer.Windows.Functions`, after Finlayson et al.: angular properties of each pane from its
  normal-incidence values, panes combined by the net radiation method): direct transmittance \(\tau\) and
  absorptance \(a_i\) of each pane at the incidence angles 0, 10, ..., 90 degrees, and their hemispherical values;
  the gap resistances follow EN 673. The library fits a quartic in \(\cos\theta\) through the ten angles (least squares, at
  translation), so that the optics are smooth in the sun position:
  \(Q_{abs,i} = A_{gla}\left(H_{dir}\, a_i(\cos\theta) + H_{dif}\, a_i^{hem}\right)\);
* transmitted radiation, distributed in the room by the zone:
  \(Q_{tra,dir} = A_{gla} H_{dir} \tau(\cos\theta)\), \(Q_{tra,dif} = A_{gla} H_{dif} \tau^{hem}\);
* outside: \(Q_{ext} = A_{gla}(4 + 4v)(T_1 - T_{out}) + Q_{lw}\) (the wind-dependent film coefficient of the
  Buildings window model and the sky exchange above);
* frame: \(Q_{fra} = \dfrac{f_{fra} A}{1/h_e + 1/U_{fra} + 1/h_i}\,(T_{air} - T_{out})\), taken from the room air.

### Zones

`Zones.Zone` balances the air of the room, exchanges long-wave radiation between its surfaces through a radiant
star, and distributes the transmitted solar radiation, after `Buildings.ThermalZones.Detailed.MixedAir`:

\[
m_{sen}\,\rho c_p V\,\dot T_{air} = \sum_i Q_{con,i} + Q_{con,emitters} + A_{flo}\, q_{con} + \rho c_p V \frac{n_{ach}}{3600}(T_{out} - T_{air}) + \sum_k \dot m_k c_p (T_{sup,k} - T_{air})
\]

with \(Q_{con,i}\) the convective heat of each surface, the internal gains \(q_{con}\) per floor area, the
infiltration \(n_{ach}\) (air changes per hour, `infiltration` variant) and the supply air of a ventilation unit
(`VentilatedZone`; the extract air leaves at \(T_{air}\)). The long-wave exchange is linearised at
\(T_0 = 20\) degC: each surface \(i\) has the conductance \(G_i = 4\sigma T_0^3 \varepsilon_i A_i\) to the radiant
star, whose temperature balances the radiant gains:

\[
T_{rad} = \frac{\sum_i G_i T_i + Q_{rad,emitters} + A_{flo}\, q_{rad}}{\sum_i G_i}, \qquad
Q_{rad,i} = G_i (T_{rad} - T_i) + k_{dir,i} Q_{tra,dir} + k_{dif,i} Q_{tra,dif}
\]

The transmitted diffuse radiation is absorbed in proportion to \(\alpha_i A_i\)
(\(k_{dif,i} = \alpha_i A_i / \sum_j (\alpha_j + \tau_j) A_j\)); the direct radiation hits the floors first, and
what they reflect is spread over the other surfaces, as `SolarRadiationExchange` of Buildings. A radiant emitter
sees the mean surface temperature \(\sum_i G_i T_i / \sum_i G_i\).

### Water circuits

The water ports carry a mass flow rate and the temperature of the water crossing them. A component mixes its inlets
(regularised by \(\dot m_{small} = 10^{-6}\) kg/s so that a closed branch keeps a defined temperature) and its outlets
leave at its outlet temperature:

\[
T_{in} = \frac{\sum_k \dot m_k T_k + \dot m_{small}\, \overline{T_k}}{\sum_k \dot m_k + \dot m_{small}}
\]

| Component | Equations |
|---|---|
| `Valve` | \(\dot m = y\,\dot m_{nom}\): the valve sets the flow of its branch (control `<valve>_y`) |
| `ThreeWayValve` (mixing) | \(\dot m_1 = y\,\dot m\), \(\dot m_3 = (1 - y)\,\dot m\), \(T_2 = y T_1 + (1 - y) T_3\) (control `<valve>_y`) |
| `Junction`, `Junction2` | mass balance; the outlets leave at the mixed temperature of the inlets |
| `Pump` | pass-through; electrical power \(P = \Delta p_{nom}\,\dot m / (\rho\,\eta)\) |
| `TemperatureSensor` | pass-through; \(T\) is the water temperature (no sensor dynamics) |

**Radiator** (`RadiatorEN442_2` of Buildings): \(n_{ele} = 5\) water and metal elements in series, each emitting
by the EN 442 power law, a share \(f_{rad}\) radiant:

\[
\frac{C}{n_{ele}}\dot T_i = \dot m c_p (T_{i-1} - T_i) - Q_{con,i} - Q_{rad,i}, \qquad
Q_{con,i} = (1 - f_{rad})\, UA_{ele}\, \Delta T_i (\Delta T_i^2 + \Delta T_0^2)^{(n-1)/2}
\]

with \(\Delta T_i = T_i - T_{air}\) (radiant part: \(T_i - T_{rad}\)), \(n = 1.24\), \(C = \rho V_{wat} c_p + 500\,
m_{dry}\) and the water volume and dry mass of Buildings (5.8e-6 m3/W and 0.0263 kg/W of nominal power). As
Buildings at initialisation, \(UA_{ele}\) is solved (by bisection, at translation) so that the radiator emits its
nominal power at its nominal water and room temperatures.

**Boiler** (`BoilerPolynomial`), the reference of its circuit: the part load ratio \(y\) is the control,

\[
Q_{fuel} = y\,\frac{Q_{nom}}{\eta_{nom}}, \qquad
C\,\dot T = \dot m c_p (T_{in} - T) + \eta(y, T)\, Q_{fuel} + UA\,(T_{amb} - T), \qquad \dot E_{fuel} = Q_{fuel}
\]

with the efficiency curves of Buildings (constant, polynomial in \(y\), or quadratic-linear in \(y\) and \(T\)), its
water volume, dry mass and heat loss to the boiler room.

**Air-to-water heat pump** (`AirToWaterTableData2D` with the Vitocal 251-A08 data of Buildings, scaled to the
capacity of the YAML) charging a mixed buffer, the reference of its circuit: the compressor speed \(y\) is the
control,

\[
T_{con,out} = T + \Delta T_{con}\, y, \qquad
Q_{con} = y\, s\, \tilde Q(T_{con,out}, T_{out}), \qquad P_{el} = y\, s\, \tilde P(T_{con,out}, T_{out})
\]

\[
C\,\dot T = \dot m c_p (T_{in} - T) + Q_{con} + UA\,(T_{amb} - T), \qquad \dot E_{el} = P_{el}
\]

with \(s = Q_{nom} / \tilde Q(T_{set}, T_{out,des})\) the scaling of the tables, \(\tilde Q\) and \(\tilde P\) the
EN 14511 tables made differentiable (quadratic in the condenser temperature through the rows at 35, 45 and
55 degC, linear in the source temperature with the kinks smoothed over 1 K), \(C\) the buffer and the condenser
water, and \(UA\) the insulation of the buffer.

### Ventilation

`Ventilation.Duct` carries \(\dot m = y\,\dot m_{nom}\), the speed \(y\) of the fans of its unit travelling along
the ducts and through the zones. `Ventilation.HeatRecoveryUnit` (trano's system D: constant-effectiveness heat
exchanger and two constant-pressure fans), the reference of its air loop, with the fan speed \(y\) as its control:

\[
T_{sup} = T_{out} + \varepsilon\,(T_{ext} - T_{out}) + \frac{P_{sup} + \varepsilon P_{ret}}{\dot m_{nom} c_p}, \qquad
P_{fan} = y\,(P_{sup} + P_{ret}), \qquad P = \frac{\Delta p_{nom}\,\dot m_{nom}}{\rho\,\eta_{fan}}
\]

where \(T_{ext}\) is the mixed extract air of the zones and the fans heat the air (the supply fan after the exchanger,
the return fan before it).

### Electricity and ideal systems

* `Electrical.Photovoltaic` (`PVSimpleOriented`): \(P = A\, f_{act}\, \eta\, \eta_{DC/AC}\, (H_{dir} + H_{dif})\) on
  the plane of the modules, metered as `EPro`.
* `HVAC.IdealHeatingCooling`: the heat flow rate \(Q\) is an input `<element>_Q`, split between the room air and
  the radiant star by its radiant share.

### Inputs, outputs and roles

| Input of `building_mpc` | Role |
|---|---|
| `weather_inputs.TDryBul`, `.TBlaSky`, `.winSpe`, `.winDir`, `.HDirNor`, `.HDifHor`, `.HGloHor` | weather (disturbances) |
| `<occupancy>.gai.QCon`, `.QRad` | internal gains per floor area [W/m2] (disturbances) |
| `<valve>_y.y`, `<three-way valve>_y.y`, `<production>_y.y`, `<ventilation unit>_y.y` | controls, 0 to 1 |
| `<emitter>_Q.Q_flow` | heat of an ideal emitter [W] (control between its bounds) |

The meters are states in kWh: `EFue` (fuel), `EEle` (heat pump electricity), `EFan` (fans), `EPro` (PV
production); the comfort temperatures are the `TAir` states of the zones. `trano.optimization.Roles.infer` reads
these roles from the model: an input with a `min` and a `max` is a control, any other input a disturbance.

## Limitations

**Not modelled: generating a model stops with a message naming the element.** Boilers with a storage tank,
water-to-water heat pumps, chillers, domestic hot water tanks, batteries, EV chargers, VAV boxes and their
controls, air handling units other than the balanced unit with heat recovery (`system_d`), and power sensors.

**Simplifications:**

* **The controls are inputs.** The controllers of the YAML (thermostatic valves, boiler and heat pump controls,
  collector and three-way valve controls) are not modelled: the optimal control decides, or the runnable model
  replays the controls of the external data. Without data, `building` holds every control at 1: it is not a
  closed-loop simulation of the building like the Buildings and IDEAS models.
* **Flows are set, not pressure-driven.** Each valve sets the flow of its branch, the pressure-driven sharing of
  flow between branches (pump curves, valve authorities, balancing) is not computed. One production per hydraulic
  circuit, and mixing three-way valves only: diverting valves are not modelled.
* **Ventilation** units are balanced (supply = extract) and run at constant pressure rise; infiltration is a
  constant air change rate (no wind- or stack-driven air flows).
* **The envelope** follows Buildings, not IDEAS: the floor on ground sees a fixed ground temperature (IDEAS models
  the ground with ISO 13370), and overhangs and side fins of the windows are not modelled. The long-wave exchange
  in a room is linearised at 20 degC, the interior convection uses the natural convection laws of Buildings.
* **No moisture**: sensible heat only, with constant air properties; the `co2` occupancy variant only brings its
  heat gains.
* **The heat pump** is the Vitocal 251-A08 of Buildings scaled to the capacity of the YAML, heating only, without
  the safety controls (minimum on and off times) of Buildings.
* **Tools**: the runnable model needs OpenModelica (or Dymola) and Buildings for the weather reader; the CasADi
  export of the connected `building_mpc` needs rumoca 0.10.3 or later.

## Benchmark

### Accuracy

The [validation](../validation/fast_model.md) simulates the Buildings (or IDEAS) model of a YAML file with its own
controls and **replays** them through the runnable FastModel model: the heat of the ideal emitters, or the flow of
each radiator branch, the position of each three-way valve, the boiler part load ratio or the heat pump speed and
the fan speed. Everything else is simulated by FastModel. Largest zone RMSE per case:

| Case | Zones | Systems | Zone air temperature RMSE |
|---|---|---|---|
| BESTEST 600FF / 900FF (a year) | 1 | free floating | 0.39 K / 0.33 K |
| BESTEST 600 / 900 (a year) | 1 | ideal heating and cooling | 0.65 K / 0.41 K |
| Three zones (January) | 3 | ideal heaters, walls between zones | 0.18–0.49 K |
| Heat pump house (January) | 6 | heat pump, buffer, radiators | 0.15–0.33 K |
| Residential house before renovation (January) | 6 | gas boiler, radiators | 0.11–0.19 K |
| Residential house after renovation (January) | 6 | heat pump, buffer, three-way valve, radiators, ventilation with heat recovery, PV | 0.12–0.17 K |

The radiators deliver the heat of Buildings within 0.6–4.4 % CV(RMSE); the meters follow within a few percent
(fuel +0.0 %, heat pump electricity −1.6 and −1.8 %, fans −0.6 %, PV 0.0 %).

### Speed

The [showcase](#showcase-a-renovated-house-with-every-system) below times the most complex YAML file FastModel
simulates in OpenModelica and in CasADi, against Buildings and IDEAS.

## Showcase: a renovated house with every system

[`examples/residential_after_renovation.yaml`](https://github.com/andoludo/trano/blob/master/examples/residential_after_renovation.yaml)
is the most complex YAML file `FastModel` simulates: the Belgian semi-detached house of the
[residential reference cases](../validation/residential_reference_cases.md) after a deep renovation, with every
kind of system the library models.

| | |
|---|---|
| Zones | 6 (living room, kitchen, three bedrooms, bathroom), 110.6 m2, on two storeys |
| Envelope | 15 external walls and roofs, 10 windows (triple glazing), 2 slabs on ground, 3 party walls, 10 walls and floors between zones |
| Heating | air-to-water heat pump of 3 kW with a 100 l buffer, pump, mixing three-way valve, split valve, 6 radiators with their valves |
| Ventilation | balanced unit with 80 % heat recovery, 12 supply and extract ducts |
| Electricity | 4.2 kWp PV array |
| `building_mpc` | 189 states, 28 inputs (9 controls, 7 weather channels, 12 internal gains), 2807 parameters |

The house is simulated for the four weeks of January in Uccle by Buildings and by IDEAS, each with its own
controls (thermostatic valves, heating curve and safety controls of the heat pump, three-way valve control).
FastModel then **replays the controls of each library**: the flow of each radiator branch, the position of the
three-way valve, the compressor speed and the fan speed. The heat pump, the buffer, the water circuit, the
radiators, the ventilation unit, the PV and the six zones are simulated by FastModel.

```bash
python -m validation.mpc compare residential_after --models FastModel --references Buildings IDEAS
python -m validation.mpc.benchmark residential_after
```

### Zone temperatures

| Zone | Mean Buildings [degC] | FastModel − Buildings: RMSE / bias [K] | Mean IDEAS [degC] | FastModel − IDEAS: RMSE / bias [K] |
|---|---|---|---|---|
| Living room | 20.16 | 0.17 / −0.13 | 20.14 | 0.36 / +0.25 |
| Kitchen | 20.08 | 0.14 / −0.09 | 20.08 | 0.36 / +0.25 |
| Bedroom 1 | 18.14 | 0.13 / −0.08 | 18.11 | 0.32 / +0.14 |
| Bedroom 2 | 17.90 | 0.12 / −0.08 | 17.88 | 0.31 / +0.16 |
| Bedroom 3 | 18.02 | 0.12 / −0.06 | 17.99 | 0.33 / +0.18 |
| Bathroom | 19.10 | 0.13 / −0.06 | 19.05 | 0.39 / +0.23 |

{!validation/figures/fast_model_residential_after_zones.html!}

FastModel follows Buildings within 0.12–0.17 K: its envelope and zone are those of Buildings. Against IDEAS it
stays within 0.31–0.39 K, 0.14–0.25 K too warm on average: the two libraries describe the same house with
different envelope models, above all the floor on ground (IDEAS models the ground below the slab with ISO 13370,
Buildings and FastModel hold it at 10 degC) and the heat exchanges inside the rooms. The controls replayed from
IDEAS bring the heat the IDEAS house needed, which the FastModel house loses a little more slowly.

### Systems

| | Buildings | FastModel replaying Buildings | IDEAS | FastModel replaying IDEAS |
|---|---|---|---|---|
| Heat pump electricity [kWh] | 130.6 | 128.3 (−1.8 %) | 134.8 | 133.5 (−1.0 %) |
| Fan electricity [kWh] | 17.2 | 17.0 (−0.6 %) | 17.2 | 16.8 (−2.1 %) |
| PV production [kWh] | 139.3 | 139.3 (0.0 %) | 139.3 | 139.3 (0.0 %) |
| Radiator heat, CV(RMSE) hourly | | 1.8–4.3 % | | 1.6–4.1 % |
| Heat pump supply water, RMSE | | 0.72 K | | 0.80 K |
| Supply air of the ventilation unit, RMSE | | 0.30 K | | 0.27 K |

{!validation/figures/fast_model_residential_after_energy.html!}

The radiators deliver the heat of either library within a few percent hour by hour, and the meters agree within
2 %. The water temperatures differ most (supply 0.7–0.8 K, return 1.7 K) at the start and stop of the heat pump:
the Buildings heat pump has a condenser and a pump of its own, the FastModel one heats a mixed buffer.

### Speed

Four weeks of January, OpenModelica 1.26.9 (DASSL, tolerance 1e-6), on the same 4-core machine:

| Model | Translation and compilation [s] | Simulation [s] |
|---|---|---|
| Buildings | 182 | 886 |
| IDEAS | 167 | 373 |
| FastModel (`building`, replaying the controls) | 24 | 99 |
| FastModel in CasADi (`building_mpc`, CVODES, inputs held over 5-minute steps) | 30 (rumoca) | 9 |

FastModel simulates the four weeks 9 times faster than Buildings and 4 times faster than IDEAS in OpenModelica,
and about 100 times faster than Buildings once translated to CasADi. The Buildings and IDEAS runs record the signals
the replay needs every minute, the FastModel run every 15 minutes.

The CasADi model is the same model as the OpenModelica one: simulated with the inputs of the OpenModelica run, its
zone temperatures stay within 0.014 K of it over the four weeks.

**Optimal control.** A day-ahead problem of the house (hourly steps, the 9 controls, comfort at 20 degC from 7 to
22 h and 17 degC otherwise, electricity bought at 0.30 and sold at 0.05 per kWh) is solved by IPOPT from the middle
of January:

| Radau collocation points per hour (`CollocationSettings.degree`) | NLP variables | IPOPT iterations | Solve time [s] |
|---|---|---|---|
| 1 | about 9 600 | 100 | 39 |
| 2 | about 14 200 | 97 | 117 |
| 3 (default) | about 18 700 | 165 | 380–475 |

The model itself costs little (about 2 s of evaluations in the slowest solve): the time goes into the
factorisations of the KKT system, whose Hessian blocks couple the 189 states at every collocation point. For a
model of this size, fewer collocation points per step, or a shorter horizon, keep the solve within a minute.

{!validation/figures/fast_model_residential_after_speed.html!}
