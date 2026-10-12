# Hydronic heating with mixing circuits

This tutorial heats the three zones of the [hydronic heating tutorial](multi-zone_with_hydronic_heating.md) with
a hydronic system wired the way a heating engineer would wire it: **each group of radiators has a mixing circuit
with its pump after its three-way valve**. The boiler can then run hot while the radiators get the water
temperature they are designed for.

## Why the pump goes after the three-way valve

A three-way valve controlled on temperature has to *mix*: it blends the hot water of the boiler (port_1) with
the cooler water coming back from the radiators (port_3) and sends the blend to the radiators (port_2). Its
controller measures the temperature after the valve and opens or closes the valve to hold it at a set point.

Mixing only happens if something pulls the return water into port_3: the pump of the circuit, placed **after**
the valve.

```
boiler ──► port_1 ┐
                  ├ THREE-WAY VALVE ── port_2 ──► sensor ──► pump ──► radiators ──► radiator valves
          port_3 ─┘                                                                       │
            ▲                                                                             ▼
            └──────────────────────────────── split valve ◄───────────────────────────────┘
                                                   │
                                                   └──► back to the boiler
```

When the pump is placed **before** the valve, as in the earlier hydronic tutorials, it pushes the water into
port_1: the water then leaves through port_3 back to the boiler instead of being drawn in. The valve no longer
mixes, it *diverts*: closing it sends hot water back to the boiler instead of to the radiators, but the water
that does reach the radiators is still as hot as the boiler supply. The valve can cut the flow, never lower the
temperature. If the boiler supply exceeds the set point of the valve, the controller closes the valve
completely and the rooms go cold: with the default boiler set point of 80 degC, the top of the storage tank
reaches about 95 degC, above the default 90 degC of the valve, and the rooms of such a model fell to 16 degC.
Trano reports that layout with a `HydronicLayoutWarning` (see
[the note in the hydronic heating tutorial](multi-zone_with_hydronic_heating.md#why-the-layout-of-this-tutorial-is-wrong)).

## Configuration of the systems

The envelope, the occupancy and the emissions are those of the hydronic heating tutorial; only the `systems`
change. Each mixing circuit is a chain `three_way_valve -> temperature_sensor -> pump -> radiators`:

- the **boiler** feeds the three-way valves directly (`outlets` of the boiler);
- the **three-way valve** lists the **split valve** of its circuit in its `inlets`: that is the return water
  mixed in through port_3. Its control holds `supply_temperature_setpoint` (70 degC here; 90 degC when absent)
  with a PI controller (see below);
- the **temperature sensor** comes right after the valve: its control measures the mixed water there;
- the **pump** comes after the sensor and feeds the radiators of the circuit. Each circuit has its own pump;
  a single pump before several valves cannot make them mix;
- the **split valve** collects the return of the radiator valves and sends it to the boiler; the three-way valve
  draws its share of it.

```yaml
systems:
  # A mixing circuit per group of rooms: boiler -> three-way valve -> temperature sensor -> pump -> radiators.
  # The pump after the valve draws the return water in through port_3 of the valve and mixes it with the
  # boiler water, so the valve holds the temperature sent to the radiators whatever the boiler supply.
  - boiler:
      id: BOILER:001
      parameters:
        nominal_heating_power: 12000
      control:
        boiler_control:
          id: BOILER_CONTROL:001
          parameters:
            tsup_nominal: 353.15  # 80 degC: the top of the storage tank reaches about 95 degC
      outlets:
        - THREE_WAY_VALVE:001
        - THREE_WAY_VALVE:002
  - three_way_valve:
      id: THREE_WAY_VALVE:001
      control:
        three_way_valve_control:
          id: THREE_WAY_VALVE_CONTROL:001
          parameters:
            supply_temperature_setpoint: 343.15  # 70 degC to the radiators
            # a PI controller tuned for a valve: the default proportional controller (gain 1 per kelvin) moves
            # the valve over its whole stroke for 1 K, hunts around the set point and slows the simulation down
            controller_type: Buildings.Controls.OBC.CDL.Types.SimpleController.PI
            k: 0.1
            ti: 300
      inlets:
        - SPLIT_VALVE:001  # the return water mixed in through port_3
      outlets:
        - TEMPERATURE_SENSOR:001
  - temperature_sensor:
      id: TEMPERATURE_SENSOR:001
      outlets:
        - PUMP:001
  - pump:
      id: PUMP:001
      control:
        collector_control:
      outlets:
        - RADIATOR:001
        - RADIATOR:003
  - split_valve:
      id: SPLIT_VALVE:001
      inlets:
        - VALVE:001
        - VALVE:003
      outlets:
        - BOILER:001
  - three_way_valve:
      id: THREE_WAY_VALVE:002
      control:
        three_way_valve_control:
          id: THREE_WAY_VALVE_CONTROL:002
          parameters:
            supply_temperature_setpoint: 343.15
            controller_type: Buildings.Controls.OBC.CDL.Types.SimpleController.PI
            k: 0.1
            ti: 300
      inlets:
        - SPLIT_VALVE:002
      outlets:
        - TEMPERATURE_SENSOR:002
  - temperature_sensor:
      id: TEMPERATURE_SENSOR:002
      outlets:
        - PUMP:002
  - pump:
      id: PUMP:002
      control:
        collector_control:
      outlets:
        - RADIATOR:002
  - split_valve:
      id: SPLIT_VALVE:002
      inlets:
        - VALVE:002
      outlets:
        - BOILER:001
```

??? example "Full configuration file (tests/tutorials/three_zones_hydronic_mixing.yaml)"

    ```yaml
    default: !include_default
    material:
      - id: MATERIAL:001  # facing brick
        thermal_conductivity: 0.6
        density: 1800.0
        specific_heat_capacity: 840.0
      - id: MATERIAL:002  # mineral wool
        thermal_conductivity: 0.035
        density: 30.0
        specific_heat_capacity: 1030.0
      - id: MATERIAL:003  # gypsum plaster
        thermal_conductivity: 0.5
        density: 1300.0
        specific_heat_capacity: 1000.0

    constructions:
      - id: CONSTRUCTION:001
        layers:
          - material: MATERIAL:001
            thickness: 0.1
          - material: MATERIAL:002
            thickness: 0.1
          - material: MATERIAL:003
            thickness: 0.015
    spaces:
      - occupancy:
        parameters:
          floor_area: 49.0
          average_room_height: 2.5
        id: SPACE:001
        external_boundaries:
          external_walls:
            - surface: 20
              azimuth: 0
              tilt: wall
              construction: CAVITYWALL:001
            - surface: 30
              azimuth: 1.57
              tilt: wall
              construction: CAVITYWALL:001
            - surface: 50
              azimuth: 3.14
              tilt: wall
              construction: CAVITYWALL:001
          windows:
            - surface: 5.0
              construction: EPCDOUBLE:001
              azimuth: 0
              tilt: wall
            - surface: 2.0
              construction: EPCDOUBLE:001
              azimuth: 3.14
              tilt: wall
          floor_on_grounds:
            - surface: 49
              construction: CONCRETESLAB:001
        emissions:
          - radiator:
              parameters:
                nominal_heating_power_positive_for_heating: 5000
                dp_nominal: 100
              id: RADIATOR:003
          - valve:
              id: VALVE:003
              control:
                emission_control:
      - occupancy:
          parameters:
            gain: "[ 45; 90; 40 ]"
            occupancy: 3600*{16, 21}
        parameters:
          floor_area: 40
          average_room_height: 2.5
        id: SPACE:002
        external_boundaries:
          external_walls:
            - surface: 25
              azimuth: 0
              tilt: wall
              construction: CAVITYWALLPARTIALFILL:001
            - surface: 25
              azimuth: 1.57
              tilt: wall
              construction: CAVITYWALLPARTIALFILL:001
            - surface: 34
              azimuth: 3.14
              tilt: wall
              construction: CAVITYWALLPARTIALFILL:001
          windows:
            - surface: 5.0
              construction: INS2AR2020:001
              azimuth: 0
              tilt: wall
            - surface: 2.0
              construction: INS2AR2020:001
              azimuth: 3.14
              tilt: wall
          floor_on_grounds:
            - surface: 40
              construction: CONCRETESLAB:001
        emissions:
          - radiator:
              parameters:
                nominal_heating_power_positive_for_heating: 4000
              id: RADIATOR:001
          - valve:
              id: VALVE:001
              control:
                emission_control:
      - occupancy:
          parameters:
            gain: "[35; 70; 30]"
            occupancy: 3600*{10, 14}
        parameters:
          floor_area: 50.0
          average_room_height: 2.5
        id: SPACE:003
        external_boundaries:
          external_walls:
            - surface: 22
              azimuth: 3.14
              tilt: wall
              construction: CONSTRUCTION:001
            - surface: 17
              azimuth: 3.14
              tilt: wall
              construction: CONSTRUCTION:001
            - surface: 36
              azimuth: 3.14
              tilt: wall
              construction: CONSTRUCTION:001
          floor_on_grounds:
            - surface: 50.0
              construction: CONCRETESLAB:001
        emissions:
          - radiator:
              parameters:
                nominal_heating_power_positive_for_heating: 4000
              id: RADIATOR:002
          - valve:
              id: VALVE:002
              control:
                emission_control:
    internal_walls:
      - space_1: SPACE:001
        space_2: SPACE:002
        construction: CAVITYWALL:001
        surface: 20
      - space_1: SPACE:002
        space_2: SPACE:001
        construction: CONSTRUCTION:001
        surface: 15
      - space_1: SPACE:002
        space_2: SPACE:003
        construction: CAVITYWALLPARTIALFILL:001
        surface: 22
    systems:
      # A mixing circuit per group of rooms: boiler -> three-way valve -> temperature sensor -> pump -> radiators.
      # The pump after the valve draws the return water in through port_3 of the valve and mixes it with the
      # boiler water, so the valve holds the temperature sent to the radiators whatever the boiler supply.
      - boiler:
          id: BOILER:001
          parameters:
            nominal_heating_power: 12000
          control:
            boiler_control:
              id: BOILER_CONTROL:001
              parameters:
                tsup_nominal: 353.15  # 80 degC: the top of the storage tank reaches about 95 degC
          outlets:
            - THREE_WAY_VALVE:001
            - THREE_WAY_VALVE:002
      - three_way_valve:
          id: THREE_WAY_VALVE:001
          control:
            three_way_valve_control:
              id: THREE_WAY_VALVE_CONTROL:001
              parameters:
                supply_temperature_setpoint: 343.15  # 70 degC to the radiators
                # a PI controller tuned for a valve: the default proportional controller (gain 1 per kelvin) moves
                # the valve over its whole stroke for 1 K, hunts around the set point and slows the simulation down
                controller_type: Buildings.Controls.OBC.CDL.Types.SimpleController.PI
                k: 0.1
                ti: 300
          inlets:
            - SPLIT_VALVE:001  # the return water mixed in through port_3
          outlets:
            - TEMPERATURE_SENSOR:001
      - temperature_sensor:
          id: TEMPERATURE_SENSOR:001
          outlets:
            - PUMP:001
      - pump:
          id: PUMP:001
          control:
            collector_control:
          outlets:
            - RADIATOR:001
            - RADIATOR:003
      - split_valve:
          id: SPLIT_VALVE:001
          inlets:
            - VALVE:001
            - VALVE:003
          outlets:
            - BOILER:001
      - three_way_valve:
          id: THREE_WAY_VALVE:002
          control:
            three_way_valve_control:
              id: THREE_WAY_VALVE_CONTROL:002
              parameters:
                supply_temperature_setpoint: 343.15
                controller_type: Buildings.Controls.OBC.CDL.Types.SimpleController.PI
                k: 0.1
                ti: 300
          inlets:
            - SPLIT_VALVE:002
          outlets:
            - TEMPERATURE_SENSOR:002
      - temperature_sensor:
          id: TEMPERATURE_SENSOR:002
          outlets:
            - PUMP:002
      - pump:
          id: PUMP:002
          control:
            collector_control:
          outlets:
            - RADIATOR:002
      - split_valve:
          id: SPLIT_VALVE:002
          inlets:
            - VALVE:002
          outlets:
            - BOILER:001
    ```

### Tuning the valve controller

`three_way_valve_control` takes the parameters of its controller (`controller_type`, `k`, `ti`, ...; see the
[parameters](../reference/parameters.md#threewayvalvecontrolparameters)). This example uses a PI controller with a
gain of 0.1 per kelvin and an integral time of 300 s; the
[lessons learned](#lessons-learned-the-pid-of-the-three-way-valves) at the end of this page explain why.

The model is created as in the other tutorials:

```python title='Test three zones hydronic mixing'
def test_three_zones_hydronic_mixing() -> None:
    from trano.main import create_model

    create_model(
        path_to_yaml_configuration_folder / "three_zones_hydronic_mixing.yaml",
        library="IDEAS",
    )
```

## Results

Two January weeks simulated with IDEAS. The rooms are heated to 21 degC from 7 to 19 h and fall back to 16 degC
at night; the boiler, with its default 80 degC set point, charges its storage tank up to about 95 degC.

{!tutorials/figures/three_zones_hydronic_mixing.html!}

The figure below shows the mixing at work. While the top of the tank is above 75 degC (it rises to 94 degC), the
water sent to the radiators stays at the 70 degC set point of the valves (median 70.0 degC; 95 % of the time below
76 degC; it dips when the rooms start heating after the night setback and the valves open fully). The rooms
average 20.3 to 20.5 degC during the occupied hours. With the pump before the valves, that water would follow the
tank temperature.

{!tutorials/figures/three_zones_hydronic_mixing_water.html!}

## Lessons learned: the PID of the three-way valves

These lessons come from simulating three January days of this model with IDEAS, changing only the controller of
the two valves. The figures are for the water sent to the radiators while the top of the tank is above 75 degC.

| Controller of the valves | Water to the radiators: median | 95 % of the time below | Rooms (occupied, mean) | Time to simulate |
|---|---|---|---|---|
| Default: proportional, `k` = 1 per K | 68 to 69 degC | 75 degC | 20.3 to 20.4 degC | 451 s |
| Proportional, `k` = 0.1 per K | **63 degC** | 71 degC | 20.2 to 20.3 degC | 128 s |
| **PI, `k` = 0.1 per K, `ti` = 300 s** | **70.0 degC** | 76 degC | 20.3 to 20.4 degC | **110 s** |
| Pump before the valves (earlier tutorial), default controller | follows the tank | — | — | 25 s |

**1. The tuning only matters once the valve really regulates.** With the pump before the valves, the valves sit
fully open, the controller never acts, and the default tuning looks fine: that model simulates in 25 s.
The same controller on valves that do mix made the simulation 18 times slower. A layout that works hides the
weaknesses of its controls, so a model only tells you about its controller once the controller has something to do.

**2. The gain has units, and 1 per kelvin is a lot for a valve.** The controller turns a temperature error in
kelvin into a valve opening between 0 and 1. With `k` = 1, an error of 1 K already moves the valve over its whole
stroke. Between the valve and the sensor there are delays: the mixing volume, the sensor's time constant, the
travel of the water and, in IDEAS, the actuator of the valve. A large gain acting through delays overshoots,
corrects and overshoots again: the valve hunts around the set point, and every reversal is a fast change the solver
follows with small steps. We did not record the valve finely enough to see each reversal, but the timings point to
it: lowering the gain alone cut the simulation from 451 s to 128 s.

**3. A proportional controller cannot reach its set point.** It only opens the valve in proportion to the error,
so it needs a permanent error to hold the valve open: the opening divided by `k`. With `k` = 1 and the valves about
70 % open, that offset is 0.7 K (68 to 69 degC for a 70 degC set point), which is why it goes unnoticed. Lowering the gain to calm the valve
makes the offset ten times larger: `k` = 0.1 held the water at 63 degC, 7 K below the set point. A proportional
controller makes you choose between a calm valve and an accurate temperature.

**4. The integral action removes that choice.** A PI controller keeps adding up the error, so it holds the set point
with a small gain: 70.0 degC with `k` = 0.1, in 110 s. Its integral time must be slow compared with the delays of
the circuit (mixing volume, sensor, valve actuator): 300 s here.

**5. Never switch to PI without setting `ti`.** The default `ti` is 0.5 s. It is unused by the proportional
controller, but it becomes active as soon as `controller_type` is set to PI. An integral time of half a second is far faster than
the delays of the circuit, a classic cause of oscillation (we did not simulate it). Always give `controller_type`, `k` and `ti` together.

**6. How to check a tuning.** Plot the valve opening and the temperature after the valve (as in the second figure
of the results):

- the opening zig-zags, or the simulation is unusually slow: the gain is too high, lower `k`;
- the temperature settles away from the set point: there is no integral action, use PI;
- the temperature drifts slowly back and forth around the set point: the integral is too fast, raise `ti`.

The PI values of this example are a starting point that worked here. We tested only the three tunings above, so
check them on your own circuit with the plots above rather than reusing them blindly. The default of
`three_way_valve_control` is still the proportional controller, so that the existing models keep their results.
