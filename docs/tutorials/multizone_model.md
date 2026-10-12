# Multizone model
Similarly to changing libraries, building a multi-zone model is as easy as adding a few lines to the configuration file. This tutorial demonstrates how to generate a model for a three-zone building.

The configuration below presents the YAML file that describes a three-zone building.

```yaml
building:
  zones:
    - name: Zone 1
      area: 500
      temperature: 22
    - name: Zone 2
      area: 400
      temperature: 21
    - name: Zone 3
      area: 600
      temperature: 23
```


```yaml
material:
  - id: MATERIAL:001  # facing brick
    thermal_conductivity: 0.9
    density: 1700.0
    specific_heat_capacity: 840.0
  - id: MATERIAL:002  # mineral wool
    thermal_conductivity: 0.035
    density: 30.0
    specific_heat_capacity: 1030.0
  - id: MATERIAL:003  # concrete block
    thermal_conductivity: 1.1
    density: 1400.0
    specific_heat_capacity: 840.0
  - id: MATERIAL:004  # concrete
    thermal_conductivity: 1.7
    density: 2300.0
    specific_heat_capacity: 1000.0

constructions:
  # cavity wall filled with 12 cm of mineral wool, U = 0.26 W/m2K
  - id: CONSTRUCTION:001
    layers:
      - material: MATERIAL:001
        thickness: 0.09
      - material: MATERIAL:002
        thickness: 0.12
      - material: MATERIAL:003
        thickness: 0.14
  # concrete slab on 8 cm of insulation
  - id: SLAB:001
    layers:
      - material: MATERIAL:002
        thickness: 0.08
      - material: MATERIAL:004
        thickness: 0.15

glass_material:
  - density: 2500.0
    id: GLASS:001
    longwave_emissivity: 0.82
    shortwave_emissivity: 0.65
    specific_heat_capacity: 860.0
    thermal_conductivity: 1.1

gas:
  - density: 1.18
    id: AIR:001
    longwave_emissivity: 0.0
    shortwave_emissivity: 0.0
    specific_heat_capacity: 1005.0
    thermal_conductivity: 0.026

glazings:
  - id: INS2AR2020:001
    layers:
      - glass: GLASS:001
        thickness: 0.005
      - gas: AIR:001
        thickness: 0.014
      - glass: GLASS:001
        thickness: 0.005

spaces:
  - parameters:
      floor_area: 80.0  # 10 m x 8 m
      average_room_height: 2.5
    id: SPACE:001
    external_boundaries:
      external_walls:
        - surface: 25.0
          azimuth: 3.14
          tilt: wall
          construction: CONSTRUCTION:001
        - surface: 20.0
          azimuth: 1.57
          tilt: wall
          construction: CONSTRUCTION:001
        - surface: 20.0
          azimuth: 4.71
          tilt: wall
          construction: CONSTRUCTION:001
      floor_on_grounds:
        - surface: 80.0
          construction: SLAB:001
      windows:
        - surface: 4.0
          azimuth: 3.14
          tilt: wall
          construction: INS2AR2020:001
          width: 2.5
          height: 1.6
        - surface: 4.0
          azimuth: 1.57
          tilt: wall
          construction: INS2AR2020:001
          width: 2.5
          height: 1.6

  - parameters:
      floor_area: 120.0  # 12 m x 10 m
      average_room_height: 2.7
    id: SPACE:002
    external_boundaries:
      external_walls:
        - surface: 32.4
          azimuth: 3.14
          tilt: wall
          construction: CONSTRUCTION:001
        - surface: 27.0
          azimuth: 1.57
          tilt: wall
          construction: CONSTRUCTION:001
        - surface: 32.4
          azimuth: 0.0
          tilt: wall
          construction: CONSTRUCTION:001
      floor_on_grounds:
        - surface: 120.0
          construction: SLAB:001
      windows:
        - surface: 4.0
          azimuth: 3.14
          tilt: wall
          construction: INS2AR2020:001
          width: 2.5
          height: 1.6
        - surface: 10.0
          azimuth: 0.0
          tilt: wall
          construction: INS2AR2020:001
          width: 5.0
          height: 2.0
  - parameters:
      floor_area: 59.0  # 7.7 m x 7.7 m, an upper room without floor on ground
      average_room_height: 2.7
    id: SPACE:003
    external_boundaries:
      external_walls:
        - surface: 20.8
          azimuth: 3.14
          tilt: wall
          construction: CONSTRUCTION:001
        - surface: 20.8
          azimuth: 1.57
          tilt: wall
          construction: CONSTRUCTION:001
      windows:
        - surface: 4.0
          azimuth: 1.57
          tilt: wall
          construction: INS2AR2020:001
          width: 2.5
          height: 1.6

internal_walls:
  - space_1: SPACE:002
    space_2: SPACE:001
    construction: CONSTRUCTION:001
    surface: 18.0
```
            

This time, we will utilize a reduced-order model defined in the AIXLIB library.


```python title='Test tutorials'
    from trano.main import create_model

    create_model(
        path_to_yaml_configuration_folder / "multi_zones.yaml",
        library="reduced_order",
    )

```
### Code Snippet Explanation
This code snippet imports the `create_model` function from the `trano.main` module and calls it to create a model based on the specified YAML configuration file. It uses a specified library type.

### General Description and Parameters
- **Function**: `create_model`
- **Parameters**:
  - `path_to_yaml_configuration_folder / "multi_zones.yaml"`: Path to the YAML configuration file.
  - `library`: Type of library to be used (in this case, set to `"reduced_order"`).


The figure below illustrates the envelope subcomponent generated using the reduced order building component from the [AIXLIB](https://github.com/AIXLIB) library.

![Envelope components using IDEAS](./img/multi_zones.jpg)

