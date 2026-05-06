# Component variants

Most components in Trano (radiators, spaces, controls, air handling units,
boilers, valves, ...) ship with **several pre-defined implementations** called
**variants**. A variant is a concrete Modelica realisation of a component
class — for example the `Radiator` class has a `default` (`RadiatorEN442_2`)
variant and an `ideal` variant that is just an idealised heat injection
block.

This tutorial explains:

1. **What variants are** and where they live in the codebase.
2. **How to use** an existing variant from your YAML model.
3. **How to create a new variant** (built-in or in your own folder).
4. **How to register an external variants folder** so Trano picks up your
   variants automatically.
5. **How to test** custom variants.


## 1. What is a variant?

A variant describes how an instance of a Trano element class (e.g.
`Radiator`, `Space`, `EmissionControl`, `AirHandlingUnit`, ...) is rendered to
Modelica code.

Each variant is a YAML entry that contains:

| Field                  | Purpose                                                                 |
| ---------------------- | ----------------------------------------------------------------------- |
| `classes`              | The Trano element class(es) this variant applies to (e.g. `Radiator`).  |
| `variant`              | The string name used to select this variant from a YAML model.          |
| `library`              | The library this variant targets (`default`, `ideas`, `iso_13790`, ...). |
| `template`             | The Jinja2 template rendered into the generated Modelica model.         |
| `ports`                | Connection ports the component exposes.                                 |
| `parameter_processing` | How the user-provided parameters are mapped to the template.            |
| `figures`              | Plots auto-generated for the reporting view (optional).                 |
| `component_template`   | An optional inner Modelica model wrapping the component (optional).     |

The built-in variants live under
`trano/elements/library/models/<library>/<component>.yaml`. Each YAML file
contains a **list** of variant definitions for one or several classes.


### Examples of variants currently shipped

The table below summarises the variants that already exist for the most
common components, so you can pick the right one for your model. The full
list can always be discovered with:

```python
from trano.elements.library.components import COMPONENTS

for c in COMPONENTS.components:
    print(c["classes"], c["variant"], c.get("library"))
```

#### Radiator (`Radiator`)
| Variant   | Library | Use case                                                                                       |
| --------- | ------- | ---------------------------------------------------------------------------------------------- |
| `default` | default | Detailed `Buildings.Fluid.HeatExchangers.Radiators.RadiatorEN442_2` model with hydronic ports. |
| `ideal`   | default | Idealised heat emission system with a single control input — no fluid loop required.           |

#### Space (`Space`)
| Variant        | Library         | Use case                                                                       |
| -------------- | --------------- | ------------------------------------------------------------------------------ |
| `default`      | buildings       | `Buildings`-library zone (mixed air, detailed envelope).                       |
| `default`      | ideas           | `IDEAS.Buildings.Components.Zone` zone.                                        |
| `infiltration` | ideas           | Same as `default`, but configured for infiltration studies.                    |
| `default`      | iso_13790       | Reduced-order ISO 13790 5R1C zone.                                             |
| `default`      | reduced_order   | Reduced-order RC zone for fast simulations.                                    |

#### Air Handling Unit (`AirHandlingUnit`)
| Variant     | Library | Use case                                                                  |
| ----------- | ------- | ------------------------------------------------------------------------- |
| `default`   | default | Detailed AHU partial model with VAV boxes, dampers and fans.              |
| `test`      | default | Simplified HVAC model used in test cases — no VAV boxes.                  |
| `system_d`  | default | System-D mechanical balanced ventilation with heat recovery.              |

#### Emission Control (`EmissionControl`)
| Variant             | Library | Use case                                                                                  |
| ------------------- | ------- | ----------------------------------------------------------------------------------------- |
| `default`           | default | ASHRAE G36 thermal zone control loops with heating/cooling outputs.                       |
| `onoff`             | default | Simple on/off thermostat with occupancy schedule.                                         |
| `PController`       | default | Proportional (P) controller using `IDEAS.Controls.Continuous.LimPID`.                     |
| `MonitoredSetpoint` | default | P controller whose setpoint is read from a data source (e.g. measured boundary data).     |

#### Boiler (`Boiler`)
| Variant                  | Library | Use case                                                                |
| ------------------------ | ------- | ----------------------------------------------------------------------- |
| `default`                | default | Boiler with thermal storage tank.                                       |
| `without_storage`        | default | Boiler without storage (direct loop).                                   |
| `water_water_heat_pump`  | default | Water/water heat pump as production system.                             |
| `air_water_heat_pump`    | default | Air/water heat pump as production system.                               |

#### Boiler Control (`BoilerControl`)
| Variant   | Library | Use case                                                  |
| --------- | ------- | --------------------------------------------------------- |
| `default` | default | Default boiler control loop.                              |
| `complex` | default | Detailed boiler control with cascade logic.               |
| `simple`  | default | Simple on/off boiler control.                             |

#### VAV box (`VAV`)
| Variant   | Library | Use case                                                      |
| --------- | ------- | ------------------------------------------------------------- |
| `default` | default | Pressure-independent damper as a single block.                |
| `complex` | default | Full VAV box partial model with reheat coil, dampers, sensors.|

#### VAV Control (`VAVControl`)
| Variant    | Library | Use case                                                          |
| ---------- | ------- | ----------------------------------------------------------------- |
| `default`  | default | ASHRAE G36 reheat-box terminal-unit controller.                   |
| `constant` | default | Simple constant-flow VAV controller.                              |

#### Occupancy (`Occupancy`)
| Variant   | Library         | Use case                                                                |
| --------- | --------------- | ----------------------------------------------------------------------- |
| `default` | default / iso_13790 / reduced_order | Schedule-based occupancy block.                       |
| `data`    | default         | Occupancy gains read from external CSV/data.                            |


## 2. Selecting a variant from a YAML model

Use the `variant:` field of the component you are configuring. If a component
does not declare a `variant`, Trano falls back to the variant called
`default`.

```yaml
spaces:
  - id: SPACE:001
    parameters:
      floor_area: 80.0
      average_room_height: 2.5
    external_boundaries: ...
    emissions:
      - radiator:
          id: RADIATOR:001
          # Use the *ideal* radiator instead of the default detailed one
          variant: ideal
          parameters:
            nominal_heating_power_positive_for_heating: 2500
          control:
            emission_control:
              id: EMISSION_CONTROL:001
              # Use the on/off emission control variant
              variant: onoff
              parameters:
                schedule: 3600*{10, 20}
                temperature_heating_setpoint: 295.15
                temperature_heating_setback: 291.15
```

The `variant` key works the same way for any component in the YAML schema:
`space.variant`, `boiler.variant`, `pump.variant`, `air_handling_unit.variant`,
`v_a_v.variant`, etc.


## 3. Creating a new variant

Creating a new variant is a small, focused YAML edit. There are two equally
valid options:

| Option                           | When to use                                                       |
| -------------------------------- | ----------------------------------------------------------------- |
| **A — built-in variant**         | The variant should ship with Trano and be available to everyone.  |
| **B — external variants folder** | The variant is project- or vendor-specific; lives in your repo.   |

In both cases the YAML schema is identical.

### Step-by-step

1. **Pick the class** you want to extend (e.g. `Radiator`).
2. **Pick a unique variant name** for that class. Name + library combo must
   be unique. The string is what users will write under `variant:` in their
   YAML model.
3. **Write the Modelica template** as a Jinja2 string. Trano renders it with
   the `element`, `package_name`, `library_name`, `parameters` and
   `library_name` variables (see existing templates for reference).
4. **Declare the ports** — for fluid systems typically `port_a` /
   `port_b`, plus optional `heat` ports (`heatPortCon`, `heatPortRad`) and
   `data` ports for control buses.
5. **Choose a `parameter_processing` function** — `default_parameters`,
   `modify_alias`, `exclude_parameters` (see
   `trano/elements/library/parameters.py`).
6. **Save the file**.

Below is the YAML you need to add a custom radiator that injects a fixed
heat flow:

```yaml
- classes:
  - Radiator
  variant: my_constant_emission
  library: default
  parameter_processing:
    function: default_parameters
  ports:
    - flow: convective
      medium: heat
      names: [heatPortCon]
      targets: [Space]
    - flow: radiative
      medium: heat
      names: [heatPortRad]
      targets: [Space]
    - flow: inlet
      medium: fluid
      names: [port_a]
    - flow: outlet
      medium: fluid
      names: [port_b]
  template: |2-

      MyCompany.Trano.Emission.ConstantHeatRadiator
      {{ element.name }}(
        {{ macros.render_parameters(parameters) | safe }},
        redeclare package Medium = MediumW)
        "Constant-power radiator"
  figures: []
```

### Option A — Add the variant to Trano's built-in catalogue

Drop the YAML snippet above (as a list) into the right file under
`trano/elements/library/models/<library>/`. For a radiator targeting the
default library, append it to `trano/elements/library/models/default/radiator.yaml`.

The built-in catalogue is loaded automatically at import time, so the new
variant is available immediately.


### Option B — Keep the variant in an external folder

If your variant should not live inside Trano (proprietary code, project
specific, internal Modelica library...), place it in **any folder of your
choice** and tell Trano about it.

```
my_project/
├── modelica/
│   └── MyCompany/...
└── trano_variants/             ← any path, anywhere
    └── radiator_constant.yaml  ← contains the YAML list shown above
```

There are three equivalent ways to register the folder:

#### From the Python API

```python
from trano.elements.library.components import register_variants_folder
from trano.main import create_model

register_variants_folder("my_project/trano_variants")
create_model("my_project/model.yaml")
```

`register_variants_folder` accepts `str` and `pathlib.Path`. The folder is
**scanned recursively** for `*.yaml` files, so you can organise variants
into sub-folders.

#### From the CLI

```bash
trano create-model my_project/model.yaml --variants-folder my_project/trano_variants
trano simulate-model my_project/model.yaml --variants-folder my_project/trano_variants
```

The `--variants-folder` option (`-V`) can be passed multiple times to
register several folders.

#### From an environment variable

```bash
export TRANO_VARIANTS_PATH="$HOME/my_project/trano_variants:$HOME/shared_variants"
```

Trano reads `TRANO_VARIANTS_PATH` at import time. The value is a
`os.pathsep`-separated list of folders.


### Using the new variant

Once registered, your variant is just like any built-in one — reference it
by name:

```yaml
emissions:
  - radiator:
      id: RADIATOR:001
      variant: my_constant_emission
      parameters:
        nominal_heating_power_positive_for_heating: 1500
```


## 4. Programmatic listing and inspection

```python
from trano.elements.library.components import (
    COMPONENTS,
    register_variants_folder,
    registered_variants_folders,
    clear_variants_folders,
)

register_variants_folder("./trano_variants")

# What folders are currently registered?
print(registered_variants_folders())

# All variants known to Trano (built-in + external)
for c in COMPONENTS.components:
    if "Radiator" in c["classes"]:
        print(c["library"], c["variant"])

# Reset to the built-in catalogue (useful in tests)
clear_variants_folders()
```


## 5. Testing your custom variant

A custom variant is just YAML, but it is still good practice to test it
end-to-end. The recommended pattern:

```python
from pathlib import Path

import pytest

from trano.elements.library.components import (
    clear_variants_folders,
    register_variants_folder,
)
from trano.elements.system import Radiator


@pytest.fixture(autouse=True)
def reset_variants():
    clear_variants_folders()
    yield
    clear_variants_folders()


def test_my_variant_is_registered(tmp_path: Path):
    register_variants_folder(Path(__file__).parent / "trano_variants")
    radiator = Radiator(name="r1", variant="my_constant_emission")
    library_data = next(ld for ld in radiator.libraries_data
                         if ld.variant == "my_constant_emission")
    assert "ConstantHeatRadiator" in library_data.template
```

For an end-to-end test that builds a network from a YAML model, see
`tests/test_variants.py::test_create_model_yaml_using_custom_variant`.


## 6. Cheat sheet

| I want to ...                                            | Do this                                                                |
| -------------------------------------------------------- | ---------------------------------------------------------------------- |
| Use an existing variant                                  | Set `variant: <name>` on the component in your YAML model.             |
| Add a new variant for everyone                           | Edit `trano/elements/library/models/<library>/<component>.yaml`.       |
| Add a new variant only for my project                    | Put YAML files in a folder, then `register_variants_folder(folder)`.   |
| Register variants without changing my Python code        | `export TRANO_VARIANTS_PATH=/path/to/folder`.                          |
| Register variants from the CLI                           | `trano create-model model.yaml --variants-folder /path/to/folder`.     |
| List registered external folders                         | `registered_variants_folders()`.                                       |
| Reset to built-in only                                   | `clear_variants_folders()`.                                            |

That's it — variants are the central extension point of Trano. Any new
modelling capability (a new emission technology, a new control law, a new
zone model from your in-house Modelica library...) is just a new YAML file.
