import sys
from pathlib import Path

QUESTION = """
Give a general description and parameters description. Don't be verbose.
Be to the point of the following code snippet. Format it in markdown as such:
**General Description**: ....
**Parameters Description**:

    - ....

    - ....
"""

QUESTION_CONFIG_BEFORE = """
Explain what kind of building is described in the following configuration.
Be succinct and to the point and stay with the fact:
"""

sys.path.append(str(Path(__file__).parents[2]))
from docs_src.utils import (  # noqa: E402
    CleanedText,
    CommentCodeObject,
    CommentObject,
    DisplayObject,
    TitleText,
    Tutorial,
)

tutorial_path = Path(__file__).parents[2].joinpath("tests", "tutorials")
TUTORIALS = [
    Tutorial(
        title="First simulation",
        contents=[
            CleanedText(
                content="""This tutorial will guide you through your
first simulation using the Trano library.
Trano is a Python library that allows you to simulate building energy systems using the Buildings and IDEAS libraries.
And all you need is a configuration file and a few lines of code. This first tutorials aims at showing
you how easy it is to run building energy simulation using Trano. As well as displaying the results of the simulation.
Which will be automatically generated by Trano and displayed in a report."""
            ),
            TitleText(content="Input configuration file"),
            CommentObject(
                question=QUESTION_CONFIG_BEFORE,
                object=tutorial_path.joinpath("first_simulation.yaml"),
            ),
            DisplayObject(
                language="yaml",
                object=tutorial_path.joinpath("first_simulation.yaml"),
            ),
            TitleText(content="Code"),
            CommentCodeObject(
                language="python",
                function_name="test_first_simulation",
                object=tutorial_path.joinpath("test_tutorials.py"),
            ),
            TitleText(content="Results"),
            CleanedText(
                content="""
Once the simulation is terminated, Trano will generate Thie following report.
It contains a detailed description of the
different parameters used during the simulation. In addtion to the
parameters given in the yaml, the report also provides all the default
parameters that are used during the simulation...'"""
            ),
            DisplayObject(object=tutorial_path.joinpath("first_simulation.html")),
        ],
    ),
    Tutorial(
        title="First model",
        contents=[
            CleanedText(
                content="""I assume the previous tutorial has got you exited with running a first simulation.
This tutorial shows how to generate only the modelica model
that has been used in the simulation in the previous tutorial."""
            ),
            TitleText(content="Input configuration file"),
            CommentObject(
                question=QUESTION_CONFIG_BEFORE,
                object=tutorial_path.joinpath("first_model.yaml"),
            ),
            DisplayObject(
                language="yaml",
                object=tutorial_path.joinpath("first_model.yaml"),
            ),
            TitleText(content="Code"),
            CommentCodeObject(
                language="python",
                function_name="test_first_model",
                object=tutorial_path.joinpath("test_tutorials.py"),
            ),
            TitleText(content="Generated Modelica model"),
            CleanedText(
                content="""
The following code snippet is the generated Modelica model
corresponding to the configuration file above.
One can copy this code and paste it toa Modelica editor to run the simulation.
provided that the required library such as the
buildings library is loaded within the simulation environment"""
            ),
            DisplayObject(
                language="modelica", object=tutorial_path.joinpath("first_model.mo")
            ),
        ],
    ),
    Tutorial(
        title="Two zones",
        contents=[
            CleanedText(
                content="""The previous sections have shown you how to run a simulation and generate a model.
        And focused on a simple one zone free float building.
        This tutorial will show you how to simulate a two zone building.
        A similar modelling/building description approach as this one can be taken
        for more complex multi-zone buildings."""
            ),
            TitleText(content="Input configuration file"),
            CommentObject(
                question=QUESTION_CONFIG_BEFORE,
                object=tutorial_path.joinpath("two_zones.yaml"),
            ),
            DisplayObject(
                language="yaml",
                object=tutorial_path.joinpath("two_zones.yaml"),
            ),
            TitleText(content="Code"),
            CleanedText(
                content="""
This code snippet shows how to simulate a two zone building using Trano. It is simlar to simulating one-zone building.
                The main difference lies on how the configuration file is structured. Also, we can note that the
                library name "buildings" is used here.
                By default, Trano uses the Buildings library for simulation, however we will shown in
                the next tutorial how to use different libraries such as the IDEAS library."""
            ),
            CommentCodeObject(
                language="python",
                function_name="test_two_zones",
                object=tutorial_path.joinpath("test_tutorials.py"),
            ),
            TitleText(content="Outputs"),
            CleanedText(
                content="""
            The following report is generated by Trano after the simulation of the two zones building"""
            ),
            DisplayObject(object=tutorial_path.joinpath("two_zones.html")),
        ],
    ),
    Tutorial(
        title="Three zones ideal heating",
        contents=[
            CleanedText(
                content="""The previous sections have mainly focused on simulating the building envelope
without system to have a feeling of how the yaml files input of Trano can be built.
In this section, the focus will be on simulating a building with a heating system.
The building will be a three zone building with an ideal heating system.
Below is a description of the yaml configuration file describing the building with heating system.
The main differencea between this configuration and the previous ones is the addition of the heating
system in one of the space of the building. As follows in the yaml code snippet:
```yaml
emissions:
- radiator:
  id: RADIATOR:001
  variant: ideal
  parameters:
      nominal_heating_power_positive_for_heating: 2500
  control:
    emission_control:
      id: EMISSION_CONTROL:001
      parameters:
        schedule: 3600*{10, 20}
        temperature_heating_setpoint: 295.15
        temperature_heating_setback: 291.15
```
The emissions field is added to describe the fact that the space is linked to an emission system.
The variant field is used to specify the type of radiator, in this case it is an ideal radiator. Also,
the control object of the radiator is described within the control field.
Parameters such as setpoints and schedule are defined in the control object
can be provided as well as the heating power of the ideal radiator."""
            ),
            TitleText(content="Input configuration file"),
            CommentObject(
                question=QUESTION_CONFIG_BEFORE,
                object=tutorial_path.joinpath("three_zones_ideal_heating.yaml"),
            ),
            DisplayObject(
                language="yaml",
                object=tutorial_path.joinpath("three_zones_ideal_heating.yaml"),
            ),
            TitleText(content="Code"),
            CleanedText(
                content="""
This code snippet shows how to simulate the multizone house with Trano.
In short it uses the same code as the previous tutorial making
                it easy to use."""
            ),
            CommentCodeObject(
                language="python",
                function_name="test_three_zones_ideal_heating",
                object=tutorial_path.joinpath("test_tutorials.py"),
            ),
            TitleText(content="Outputs"),
            CleanedText(
                content="""
            The following report is generated by Trano after the simulation of the three zone with
            ideal heating system building"""
            ),
            DisplayObject(
                object=tutorial_path.joinpath("three_zones_ideal_heating.html")
            ),
        ],
    ),
    Tutorial(
        title="Three zones hydronic heating",
        contents=[
            CleanedText(
                content="""The previous tutorials walked us through idealized relatively unrealistic examples.
                In this tutorial, we will simulate a three zone building with a hydronic heating system and try
        to build a house model that is relatively relaistic. As usual, the configuration file will be shown and
        described below. However, there are few
        main points that should be noted in the configuration file:
        - Use of default construction values. Trano comes with a set of predefined construction,
        glazing and gas properties that can be used in the configuration by importing them in your configuration
        file by adding the following line? The content can be seen in:
```yaml
default: !include_default
```
- Similarly to the ideal heating emission, a list of emission systems is attributed to each space.
Each of them is linked to a radiator object (RADIATOR:001) and a valve object (VALVE:001).
 In this case, the valve is controlled instead of the radiator.
```yaml
    emissions:
      - radiator:
          id: RADIATOR:001
      - valve:
          id: VALVE:001
          control:
            emission_control:
```
- Also, a full hydronic system needs to be defined.
It defined the boiler, the pump and the different valves in the system.

```yaml
systems:
  - boiler:
      id: BOILER:001
      control:
        boiler_control:
  - pump:
      id: PUMP:001
      control:
        collector_control:
      outlets:
        - THREE_WAY_VALVE:001
        - THREE_WAY_VALVE:002
      inlets:
        - BOILER:001
  - split_valve:
      id: SPLIT_VALVE:001
      inlets:
        - VALVE:003
        - VALVE:001
      outlets:
        - BOILER:001
  - three_way_valve:
      id: THREE_WAY_VALVE:001
      control:
        three_way_valve_control:
      outlets:
        - TEMPERATURE_SENSOR:001
        - SPLIT_VALVE:001
  - temperature_sensor:
      id: TEMPERATURE_SENSOR:001
      outlets:
        - RADIATOR:001
        - RADIATOR:003
  - split_valve:
      id: SPLIT_VALVE:002
      inlets:
        - VALVE:002
      outlets:
        - BOILER:001
  - three_way_valve:
      id: THREE_WAY_VALVE:002
      control:
        three_way_valve_control:
      outlets:
        - TEMPERATURE_SENSOR:002
        - SPLIT_VALVE:002
  - temperature_sensor:
      id: TEMPERATURE_SENSOR:002
      inlets:
        - THREE_WAY_VALVE:002
      outlets:
        - RADIATOR:002

```
        """
            ),
            TitleText(content="Input configuration file"),
            CommentObject(
                question=QUESTION_CONFIG_BEFORE,
                object=tutorial_path.joinpath("three_zones_hydronic_heating.yaml"),
            ),
            DisplayObject(
                language="yaml",
                object=tutorial_path.joinpath("three_zones_hydronic_heating.yaml"),
            ),
            TitleText(content="Code"),
            CleanedText(
                content="""
                    The following code snippet shows how to simulate a multizone model using Trano.
                    The simulation will also generate a report with
                    detailed edscription of parameters as well as figures."""
            ),
            CommentCodeObject(
                language="python",
                function_name="test_three_zones_hydronic_heating",
                object=tutorial_path.joinpath("test_tutorials.py"),
            ),
            TitleText(content="Outputs"),
            CleanedText(
                content="""
            The following report is generated by Trano after the simulation of the three zone with ideal
            heating system building"""
            ),
            DisplayObject(
                object=tutorial_path.joinpath("three_zones_hydronic_heating.html")
            ),
        ],
    ),
]


def gen_docs() -> None:
    for tutorial in TUTORIALS:
        tutorial.write(
            Path(__file__)
            .parents[2]
            .joinpath("docs", "tutorials", f"{tutorial.name()}.md")
        )


if __name__ == "__main__":
    gen_docs()