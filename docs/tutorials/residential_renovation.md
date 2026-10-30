---
description: "Model a Belgian semi-detached house before and after a deep renovation with Trano and compare its heating demand with the IDEAS and Buildings libraries."
---
# A house before and after renovation

This tutorial models a whole house twice: a Belgian semi-detached house built before 1970, and the
same house after a deep renovation. It brings together what the previous tutorials introduced
(several zones, a hydronic heating system, ventilation, PV) and shows the outputs trano puts on
the data bus: zone temperatures, heat, gas and electricity.

The two configuration files are in the `examples` folder of the repository:

- [`residential_before_renovation.yaml`](https://github.com/andoludo/trano/blob/master/examples/residential_before_renovation.yaml):
  uninsulated cavity walls and attic floor, old double glazing, a leaky envelope (n50 = 12 1/h),
  a 16 kW gas boiler on radiators, natural ventilation.
- [`residential_after_renovation.yaml`](https://github.com/andoludo/trano/blob/master/examples/residential_after_renovation.yaml):
  external wall insulation, 22 cm on the attic floor, an insulated slab, triple glazing,
  n50 = 3 1/h, a 3 kW air-water heat pump on **the same radiators**, balanced ventilation with heat
  recovery and 4.2 kWp of PV.

Both are written by `examples/residential_cases.py` from one description of the house, so that
they share everything but the envelope, the airtightness and the systems. The sizing and the
comparison of the two libraries are detailed in the
[residential reference cases](../validation/residential_reference_cases.md).

## The house

Six zones on two storeys (110 m2): the living room with the hall and the kitchen on the ground
floor, three bedrooms and the bathroom upstairs. The street facade faces north, the garden south,
the gable west; the east wall is shared with the neighbour.

Each zone is a space of the `infiltration` variant that takes its airtightness, `n50`, from which
both libraries derive the same infiltration (n50 / 20). The living room of the renovated house:

```yaml
  - id: LIVING:001
    variant: infiltration
    occupancy:
      parameters:
        occupancy: 3600*{7, 8, 17, 23}
    parameters:
      floor_area: 40.0
      average_room_height: 2.6
      n50: 3.0
    external_boundaries:
      external_walls:
        - {surface: 19.24, azimuth: 0.0, tilt: wall, construction: EXTERNAL_WALL:001}
        - {surface: 14.04, azimuth: 1.5708, tilt: wall, construction: EXTERNAL_WALL:001}
        - {surface: 7.8, azimuth: 3.1416, tilt: wall, construction: EXTERNAL_WALL:001}
      windows:
        - {surface: 6.5, azimuth: 0.0, tilt: wall, construction: WINDOW:001, frame_fraction: 0.25}
        - {surface: 1.5, azimuth: 1.5708, tilt: wall, construction: WINDOW:001, frame_fraction: 0.25}
        - {surface: 0.6, azimuth: 3.1416, tilt: wall, construction: WINDOW:001, frame_fraction: 0.25}
      adiabatic_walls:
        - {surface: 21.84, azimuth: -1.5708, tilt: wall, construction: PARTY_WALL:001}
      floor_on_grounds:
        - surface: 40.0
          construction: GROUND_FLOOR:001
          perimeter: 15.8
          average_indoor_temperature: 293.15
          ground_temperature: 286.65
```

Three parts of this description are worth a closer look.

- **The party wall** is an `adiabatic_walls` entry: a wall whose outer face exchanges no heat
  because the neighbour is heated alike. Its construction is half of the shared brick wall; it
  stores heat in the zone but loses none. IDEAS renders it as an adiabatic `BoundaryWall`,
  Buildings as a construction of the zone whose outer face is left open.
- **The windows** are described by their panes and gaps (`glazings`). trano computes their U and g
  values for IDEAS (EN 673, EN 410) from the same layers that Buildings uses.
- **The slab on ground** is where the two libraries differ most. IDEAS models the ground below the
  slab (ISO 13370) from the exposed `perimeter` and the mean temperature of the room above,
  `average_indoor_temperature`; Buildings holds the outer face of the slab at `ground_temperature`.
  `examples/residential_cases.py` chooses that temperature so that both give the same heat flow
  over the heating season. These are the only library-specific inputs of the files.

## The systems

Before renovation a gas boiler heats the radiators; after renovation an air-water heat pump does,
with a balanced ventilation unit that recovers heat from the extract air, and PV on the roof:

```yaml
systems:
  - boiler:
      id: GENERATOR:001
      control:
        boiler_control:
          id: GENERATOR_CONTROL:001
          variant: simple
      # air-water heat pump rated 3 kW at -8 degC and 40 degC supply (heating curve down to 30 degC at
      # 16 degC outdoor), 100 l buffer
      parameters:
        nominal_heating_power: 3000
        temperature_supply_setpoint_heat_pump: 313.15
        use_storage_tank: true
        tank_volume: 0.1
        height_of_tank_without_insulation: 0.8
      variant: air_water_heat_pump
  - air_handling_unit:
      id: AHU:001
      variant: system_d
      parameters:
        m_flow_nominal: 0.05
        heat_exchanger_effectiveness: 0.8
solar:
  - photovoltaic:
      id: PV:001
      parameters:
        area: 21.0
        efficiency: 0.2
        azimuth: 0.0
        tilt: 35.0
```

The heat pump runs when a room asks for heat (the `simple` boiler control watches the radiator
valves) and the outdoor air is below 16 degC. It charges its buffer to a supply temperature that
follows a heating curve, from 40 degC at -8 degC outdoor down to 30 degC at 16 degC, and draws its
heat from the outdoor air of the weather file.

## Generating and simulating the models

One file gives the model of either library:

```python
from trano.main import create_model, simulate_model

create_model("examples/residential_after_renovation.yaml", library="IDEAS")
create_model("examples/residential_after_renovation.yaml", library="Buildings")

simulate_model(
    "examples/residential_after_renovation.yaml",
    library="Buildings",
    start=0,
    end=7 * 24 * 3600,
)
```

## Outputs on the data bus

Besides the zone temperatures, the systems publish their results on the data bus of the model,
where controllers, reports and other tools find them:

| Element | Outputs |
|---|---|
| Gas boiler | `fuel_power` (on the gas bus), `fuel_energy` [kWh], `heat_flow`, `heat_energy` [kWh], `gas_usage` [m3], supply and return temperatures |
| Air-water heat pump | `power` (on the grid load), `electric_energy` [kWh], `heat_flow`, `heat_energy` [kWh], supply, return and buffer temperatures, `compressor_speed` |
| Ventilation unit | `power` of the fans (on the grid load), `electric_energy` [kWh], `recovered_heat_flow`, `supply_temperature` |
| PV | `pv_power`, `pv_energy` [kWh] |
| Pumps | `power` (on the grid load) |

The figures below are drawn from these signals.

## Results

### A January week

The first week of January (Buildings). Before renovation the boiler burns up to 9 kW of gas to bring
the house back to temperature every morning; the living room and the kitchen are held at 21 degC, the
bedrooms at 18 degC and the bathroom reaches 22 degC morning and evening.

{!tutorials/figures/residential_before_week.html!}

After renovation the heat pump delivers about 2 kW at most and draws less than 1 kW of electricity. The
house cools slowly at night: the bedrooms barely leave 18-20 degC. The bathroom, heated by its old
radiator with water at 30-40 degC, struggles to reach its set point in the morning: a known drawback
of keeping high-temperature radiators with a heat pump and a setback. The PV produces on sunny
afternoons, out of step with the morning peak of the heat pump.

{!tutorials/figures/residential_after_week.html!}

### A year

Over the year, the renovation divides the heat delivered to the rooms by seven, from 130 to
18 kWh/m2, and replaces 19.2 MWh of gas by 0.53 MWh of electricity for the heat pump (seasonal COP
3.8) and 0.22 MWh for the fans. IDEAS and Buildings agree on these within 7.5 %:

{!tutorials/figures/residential_annual_balance.html!}

{!tutorials/figures/residential_monthly_heat.html!}

The PV produces 4.6 MWh a year, six times the electricity of the heat pump and the fans, but in
winter both are of the same order:

{!tutorials/figures/residential_monthly_electricity.html!}

The figures are drawn by `docs_src/tutorials/figures.py` from the results of OpenModelica:

```bash
uv run python docs_src/tutorials/figures.py residential <results directory>
```
