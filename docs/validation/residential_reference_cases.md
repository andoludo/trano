---
description: "A Belgian semi-detached house before and after renovation simulated with the IDEAS and Buildings libraries from the same Trano YAML files."
---
# Residential reference cases: IDEAS and Buildings

Two YAML files describe the same Belgian semi-detached house before and after a deep renovation:

- [`examples/residential_before_renovation.yaml`](https://github.com/andoludo/trano/blob/master/examples/residential_before_renovation.yaml)
- [`examples/residential_after_renovation.yaml`](https://github.com/andoludo/trano/blob/master/examples/residential_after_renovation.yaml)

Each file generates the model of either library, with the same geometry, zoning, use and weather:

```bash
trano create-model examples/residential_before_renovation.yaml IDEAS
trano create-model examples/residential_before_renovation.yaml Buildings
```

Both files are written by `examples/residential_cases.py`, which also prints the sizing below:
change the house there, not in the YAML files.

## The house

Semi-detached two-storey house in Uccle, 7.4 m x 8.4 m inside, street facade north, garden facade
south, gable west and party wall east. Heated floor area 110.6 m2 in six zones.

| Zone | Storey | Floor area [m2] | Height [m] | Exposed surfaces | Set point [degC], schedule |
|---|---|---|---|---|---|
| Living (with the hall) | ground | 40.0 | 2.6 | south, west, north (hall), party wall, slab | 21, 7-23 h |
| Kitchen | ground | 15.0 | 2.6 | north, west, slab | 21, 7-23 h |
| Bedroom 1 | first | 18.5 | 2.5 | south, west, attic floor | 18, 6-23 h |
| Bedroom 2 | first | 18.5 | 2.5 | north, west, attic floor | 18, 6-23 h |
| Bedroom 3 | first | 9.6 | 2.5 | north, party wall, attic floor | 18, 6-23 h |
| Bathroom | first | 9.0 | 2.5 | south, party wall, attic floor | 22, 6-9 h and 19-22 h |

Every room falls back to 16 degC outside its schedule. Four occupants, with the default occupancy
gains of trano during the occupied hours of each room. Windows: 21.1 m2 (19 % of the floor area),
6.5 m2 of them facing the garden. The ceiling of the first floor is the floor of a cold, ventilated
attic, modelled as a roof towards the outdoor air.

The party wall is an **adiabatic wall** (new, see below): half of the 19 cm brick wall, whose
mid-plane is adiabatic because the neighbour is heated alike.

## Envelope

| Element | Before renovation | U [W/(m2.K)] | After renovation | U [W/(m2.K)] |
|---|---|---|---|---|
| External wall | brick cavity wall, 5 cm air cavity | 1.47 | + 12 cm EPS (ETICS) | 0.23 |
| Attic floor | timber floor on joists | 1.70 | + 22 cm mineral wool | 0.15 |
| Slab on ground | concrete and tiles | 3.43 (slab), 0.71 (with the ground, ISO 13370) | + 8 cm PUR | 0.27 (slab), 0.20 (with the ground) |
| Windows | double glazing 4-12-4 air, wooden frames | Ug 2.88, Uw 2.71, g 0.76 | triple glazing argon, two low-e coatings, insulated frames | Ug 0.70, Uw 0.80, g 0.47 |
| Airtightness | n50 = 12 1/h (0.6 1/h of infiltration) | | n50 = 3 1/h (0.15 1/h) | |

The glazings are described by their panes and gaps: trano computes U and g (EN 673, EN 410) for
IDEAS from the same layers that Buildings uses, so that both libraries see the same windows. Air
layers in opaque constructions are equivalent conductivities giving the EN ISO 6946 resistance of
an unventilated cavity (0.18 m2K/W for 5 cm, 0.16 m2K/W between the joists).

## Systems and sizing

The design heat loss follows EN 12831 (simplified): -8 degC outdoor (Uccle), the set points above,
thermal bridges as +0.10 (before) and +0.05 W/(m2.K) (after) on the U-values, 0.5 1/h of natural
ventilation before renovation, infiltration plus the supply air after an 80 % heat recovery after
renovation, and the ground loss of the ISO 13370 model of IDEAS.

| | Before renovation | After renovation |
|---|---|---|
| Design heat loss | 10.75 kW (97 W/m2) | 2.34 kW (21 W/m2) |
| Generator | non-condensing gas boiler, 16 kW (1.4 x the loss, reheat after the setback), efficiency 0.85 (lower heating value), 75 degC supply | air-water heat pump, 3 kW at -8 degC / 40 degC (1.25 x the loss), heating curve 40 degC at -8 degC to 30 degC at 16 degC outdoor, 100 l buffer |
| Emission | radiators rated 1.2 x the room loss at 75/65/20 degC, 13.2 kW in total, thermostatic valves (P control) | **the same radiators**: they cover the loss of every room from 39 degC supply |
| Ventilation | natural, by infiltration | balanced with heat recovery (system D), 150 m3/h (0.5 1/h), effectiveness 0.8, supply and extract in every room |
| PV | - | 21 m2 of 20 % modules (4.2 kWp), south, 35 degrees |

| Room | Design loss before [W] | Design loss after [W] | Radiator at 75/65/20 degC [W] |
|---|---|---|---|
| Living | 3239 | 815 | 3900 |
| Kitchen | 1446 | 355 | 1800 |
| Bedroom 1 | 2059 | 387 | 2500 |
| Bedroom 2 | 2078 | 396 | 2500 |
| Bedroom 3 | 921 | 179 | 1200 |
| Bathroom | 1006 | 204 | 1300 |

## Reference examples

The systems follow these examples of the libraries trano targets (IDEAS 3.0.0, Buildings 13.0.0):

- **IDEAS.Examples.PPD12** (a Belgian terraced house of about 120 m2): zoning, party walls as
  `BoundaryWall`s, radiators `RadiatorEN442_2` rated at 75/65/20 degC, a gas boiler of 30 kW at
  70 degC, balanced ventilation with a `ConstantEffectiveness` heat exchanger (0.75) and 300 m3/h.
- **Buildings.Fluid.HeatPumps.ModularReversible.Examples.AirToWater2D_OneRoomRadiator**: the
  air-water heat pump (`AirToWaterTableData2D`, Vitocal 251-A data), its safety control
  (`Wuellhorst2021`, minimum on and off times) and its own condenser pump; trano's heat pump now
  follows it (see below).
- **Buildings.Examples.HydronicHeating.TwoRoomsWithStorage**: `BoilerPolynomial`, radiators with
  two-way valves and balanced ventilation with heat recovery (0.9).
- **Buildings.Examples.Tutorial.SimpleHouse.SimpleHouse6**: heat recovery ventilation (0.85).
- **Buildings.Electrical.AC.OnePhase.Sources** (`PVSimpleOriented`): the PV array.

Neither library has a dwelling that combines a heat pump with PV: the after-renovation case is new.

## Component mapping

The hydronic and ventilation systems of trano are built from Buildings components in the models of
both libraries (the `Trano` package of every model); the envelope, the zones, the weather and the
radiators come from the library chosen.

| Component | trano YAML | IDEAS model | Buildings model | Reference | Supported |
|---|---|---|---|---|---|
| Zone | `spaces` (`infiltration` variant, `n50`) | `IDEAS.Buildings.Components.Zone` | `Trano.ThermalZones.MixedAirInf` (MixedAir + infiltration) | PPD12, BESTEST | yes |
| External wall, roof | `external_walls` | `IDEAS.Buildings.Components.OuterWall` | `datConExt`, `datConExtWin` | PPD12 | yes |
| Window | `windows` + `glazings` | `IDEAS.Buildings.Components.Window` (U, g from the layers) | `datConExtWin` (layers) | PPD12 | yes |
| Slab on ground | `floor_on_grounds` | `IDEAS.Buildings.Components.SlabOnGround` (ISO 13370) | `datConBou` + fixed temperature | | yes (different ground models) |
| Party wall | `adiabatic_walls` (**new**) | `IDEAS.Buildings.Components.BoundaryWall` (adiabatic) | `datConBou`, open outer port | PPD12 `CommonWall` | yes (new) |
| Floors and walls between rooms | `internal_walls` | `IDEAS.Buildings.Components.InternalWall` | `surBou` | | yes |
| Weather | `weather` | `IDEAS.BoundaryConditions.SimInfoManager` | `Buildings.BoundaryConditions.WeatherData.ReaderTMY3` | Uccle TMYx of IDEAS | yes |
| Gas boiler | `boiler`, `without_storage` | `Buildings.Fluid.Boilers.BoilerPolynomial` | same | TwoRoomsWithStorage, PPD12 | yes (fixed) |
| Air-water heat pump | `boiler`, `air_water_heat_pump` | `Buildings.Fluid.HeatPumps.ModularReversible.AirToWaterTableData2D` | same | AirToWater2D_OneRoomRadiator | yes (rebuilt) |
| Radiators and valves | `radiator`, `valve` + `emission_control` | `IDEAS.Fluid.HeatExchangers.Radiators.RadiatorEN442_2` | `Buildings.Fluid.HeatExchangers.Radiators.RadiatorEN442_2` | PPD12 | yes |
| Distribution | `pump`, `three_way_valve`, `split_valve`, `temperature_sensor` | Buildings movers and valves | same | | yes |
| Heat recovery ventilation | `air_handling_unit`, `system_d` + `duct`s | `Buildings.Fluid.HeatExchangers.ConstantEffectiveness` + 2 `FlowControlled_dp` | same | PPD12, SimpleHouse6 | yes |
| PV | `solar.photovoltaic` | `Buildings.Electrical.AC.OnePhase.Sources.PVSimpleOriented` | same | PVPanels | yes |
| Floor heating | - | `EmbeddedPipe` | `ParallelCircuitsSlab` | | no (radiators used) |

## Changes to trano for these cases

The cases revealed problems in the existing models of trano, fixed along with the extensions:

- **Air-water heat pump** rebuilt on the Buildings reference example: it could not start with
  Buildings 13 (swapped cooling temperatures), had no safety control and no condenser pump of its
  own, and drew its heat from a constant 10 degC source, so its water boiled when the radiator
  valves closed. It now charges a buffer through its own pump, follows a heating curve, keeps the
  library's safety control and takes the outdoor air as source (`heat_pump_source_outdoor_air`,
  true by default).
- **Boilers** fire on heat demand only; the casing loss defaults to 0.5 % of the nominal power at
  50 K instead of 5 % at 30 K; natural gas counts on its lower heating value for every variant.
- **Outputs on the data bus**: electrical power and energy of the heat pump and of the ventilation
  fans (in the grid load), fuel power and energy of the boilers, heat delivered, supply and return
  temperatures, PV production.
- **Adiabatic walls** (`external_boundaries.adiabatic_walls`) for party walls, in IDEAS, Buildings,
  the reduced-order and the ISO 13790 zones.
- **IDEAS ground model**: `average_indoor_temperature` of a floor on ground sets the room
  temperature of the ISO 13370 heat flow of IDEAS (22 degC otherwise).
- **Same defaults in every library**: the Uccle weather of IDEAS when none is given, the infiltration
  of the IDEAS zones (0.15 1/h) for the Buildings `infiltration` variant, and 21 degC as the default
  heating set point.

All of them are optional: models that do not use them only change where a default was wrong.

## Results over a year

Both cases simulated over a year in OpenModelica 1.26 (IDEAS 3.0.0, Buildings 13.0.0) with the Uccle
weather of IDEAS (11.1 degC mean). Energies are taken from the counters each element puts on the data
bus; the heat of the radiators is integrated from their heat flows.

| | Before, Buildings | Before, IDEAS | Difference | After, Buildings | After, IDEAS | Difference |
|---|---|---|---|---|---|---|
| Heat from the radiators [kWh] | 14 420 | 14 910 | +3.4 % | 1 952 | 1 809 | -7.3 % |
| Heating demand [kWh/m2] | 130 | 135 | | 17.6 | 16.4 | |
| Heat from the generator [kWh] | 15 591 | 15 845 | +1.6 % | 1 992 | 1 844 | -7.4 % |
| Gas, lower heating value [kWh] | 19 188 | 19 361 | +0.9 % | - | - | |
| Heat pump electricity [kWh] | - | - | | 529 | 494 | -6.6 % |
| Seasonal COP of the heat pump | - | - | | 3.77 | 3.73 | -0.9 % |
| Ventilation fans [kWh] | - | - | | 224 | 224 | 0.0 % |
| Circulation pumps [kWh] | 30 | 27 | | 17 | 16 | |
| PV production [kWh] | - | - | | 4 644 | 4 645 | 0.0 % |

The two libraries agree on the annual heating within 3.4 % before and 7.3 % after renovation, within
the 10 % aimed at. Per room:

| Room | Radiators before, Buildings / IDEAS [kWh] | Difference | Radiators after, Buildings / IDEAS [kWh] | Difference |
|---|---|---|---|---|
| Living | 4 639 / 4 612 | -1 % | 806 / 757 | -6 % |
| Kitchen | 2 579 / 2 691 | +4 % | 607 / 561 | -8 % |
| Bedroom 1 | 2 130 / 2 237 | +5 % | 67 / 54 | -19 % |
| Bedroom 2 | 2 594 / 2 770 | +7 % | 157 / 140 | -11 % |
| Bedroom 3 | 1 170 / 1 236 | +6 % | 46 / 38 | -17 % |
| Bathroom | 1 308 / 1 364 | +4 % | 267 / 258 | -3 % |

After renovation the bedrooms, at 18 degC, are kept warm by the rooms around them and by the supply air:
their few tens of kWh differ by up to 19 % between the libraries without weighing on the total.

<!-- figures of docs/tutorials/figures, written by docs_src/tutorials/figures.py -->
{!tutorials/figures/residential_annual_balance.html!}

The renovation cuts the heat delivered to the rooms by 86 % (130 to 18 kWh/m2) and the final energy
for heating from 19.2 MWh of gas to 0.53 MWh of electricity. With the fans, the house draws 0.75 MWh
of electricity a year for heating and ventilation, against 4.6 MWh produced by the PV, most of it
from April to September (see the
[tutorial](../tutorials/residential_renovation.md) for the monthly figures).

## Known differences and limits

- **Ground.** IDEAS computes the heat flow through the slab with the ISO 13370 model, from the exposed
  perimeter and the mean temperature of the room above; Buildings holds the outer face of the slab at
  a fixed temperature. `examples/residential_cases.py` sets that temperature so that both lose the
  same heat over the heating season. The IDEAS ground starts at 10.8 degC and takes about two weeks
  to settle: short simulations (the four January days of `tests/test_residential_examples.py`)
  differ more (within 15 % upstairs, 25 % for the house).
- **Comfort in the bathroom after renovation.** The bathroom (22 degC, 6-9 h and 19-22 h, 16 degC
  otherwise) is below 21 degC during 43 % of its occupied hours in both libraries: the radiator
  kept from before the renovation warms it slowly from the setback with water at 30-40 degC. A
  lighter setback, an earlier start or a towel radiator sized for low temperatures would fix it;
  the case keeps the radiators on purpose. The other rooms are below their set point minus 1 K during
  at most 8 % of their occupied hours before renovation (the reheat after the setback) and 4 % after.
- **Boiler losses.** `BoilerPolynomial` of Buildings 13 subtracts the loss of its casing twice
  from the water; trano sets that loss to 0.5 % of the nominal power at 50 K so that it stays small.
  Before renovation, 7.5 % of the heat of the boiler is lost by the boiler, the pipes and the valves
  before the radiators.
- **Air layers** of opaque constructions are equivalent solid layers in both libraries (no
  convection or radiation across the cavity).
- **Natural ventilation** before renovation is the infiltration of the zones, constant at
  n50 / 20; it does not vary with the wind or the temperature difference.
