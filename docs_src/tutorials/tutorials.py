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
    Text,
    CommentCodeObject,
    CommentObject,
    DisplayObject,
    TitleText,
    Tutorial,
    DisplayImage,
)

tutorial_path = Path(__file__).parents[2].joinpath("tests", "tutorials")
TUTORIALS = [
    Tutorial(
        title="First model and simulation",
        contents=[
            CleanedText(
                content="""This tutorial will guide you through how to create and simulate your
first Building Energy Simulation model in Modelica using the Trano library.
All you need is a configuration file and a few lines of code. This first tutorials aims at showing
you how easy it is to run building energy simulation using Trano. As well as displaying how the model can be 
browsed and modified with OpenModelica
Or directly run through Trano using the official openModelica docker image. Also, Trano has a feature that 
automatically generates a report of the
key parameters."""
            ),
            TitleText(content="Input configuration file"),
            CleanedText(
                content="""The following file describes a single-zone house: 100 m2 of floor with a large south-facing
                window, brick and insulation walls, an insulated flat roof and a slab on the ground, occupied in the
                morning and in the evening, with 0.5 air changes per hour of infiltration and an ideal heating and
                cooling system keeping it between 21 degC by day (18 degC at night) and 26 degC. Azimuths are in
                radians, 0 being south; a wall's surface is the gross area of the facade, the windows facing the
                same way are cut out of it. Every parameter is optional and takes the default of the library
                otherwise."""
            ),
            CommentObject(
                question=QUESTION_CONFIG_BEFORE,
                object=tutorial_path.joinpath("first_model.yaml"),
            ),
            DisplayObject(
                language="yaml",
                object=tutorial_path.joinpath("first_model.yaml"),
            ),
            TitleText(content="Generate Modelica model"),
            CommentCodeObject(
                language="python",
                function_name="test_first_model",
                object=tutorial_path.joinpath("test_tutorials.py"),
            ),
            CleanedText(
                content="""The model is generated in the same folder as the configuration file but with the 
                extension .mo.
                For the present case, the model is generated in the file first_model.mo. Provided that the required 
                library such as the
                Buildings library in this case are loaded, the generated model can be opened in OpenModelica. 
                As shown in the figure below. Also, if not specified, Trano uses the Buildings library to generate 
                the model."""
            ),
            DisplayImage(title="Generated model", path="./img/first_simulation_1.jpg"),
            CleanedText(
                content="""If one opens the main building components, the generated components is subdivided into 
                different sub-components
                as shown below: the envelope holds the walls, the roof, the floor and the windows, the emission
                holds the ideal heating and cooling system."""
            ),
            DisplayImage(title="Building components", path="./img/first_simulation_2.jpg"),
            CleanedText(
                content="""Opening the envelope subcomponents, one can see the different base components and 
                information
                constituting the building envelope model. From this point on, the user can modify the model 
                as needed."""
            ),
            DisplayImage(title="Envelope components", path="./img/first_simulation_3.jpg"),
            TitleText(content="Simulate"),
            CleanedText(
                content="""The following code snippet can be used to directly simulate the model after generation.
                The model won't be generated but will be directly simulated within openModelica container."""
            ),
            CommentCodeObject(
                language="python",
                function_name="test_first_simulation",
                object=tutorial_path.joinpath("test_tutorials.py"),
            ),
            CleanedText(
                content="""Once the simulation is completed some key parameters are displayed in a report file such as
                displayed below. The first figure shows the air temperature of the zone over two weeks of January,
                held at the set point by day, falling towards the night set back, with the solar gains through the
                south window easing the heating on sunny afternoons; the second shows the heat flows of the
                occupants and of the ideal system."""
            ),
            DisplayObject(object=tutorial_path.joinpath("first_model.html")),
        ],
    ),
    Tutorial(
        title="Model with another library",
        contents=[
            CleanedText(
                content="""Switching between libraries is easy with Trano. 
                This tutorial shows how to generate the model using the IDEAS library from the same yaml 
                file as previously used."""
            ),
            TitleText(content="Supported libraries"),
            Text(
                content="""The same YAML file generates a model for each of the libraries trano supports: `Buildings`
(13.0.0) and `IDEAS` (3.0.0) for detailed zones, `reduced_order` (the AixLib 3.0.1 reduced-order zone) and
`iso_13790` (the ISO 13790 zone of AixLib) for simplified zones, and `mpc` for control-oriented RC models.
Buildings and IDEAS are validated against ASHRAE Standard 140 (BESTEST), see the
[validation results](../validation/bestest.md) and the [walk through the cases](../validation/bestest_cases.md);
the simplified zones are reported there for information. The [parameter reference](../reference/parameters.md)
lists, for every parameter, the Modelica name it is rendered under in each library and the parameters a library
does not take."""
            ),
            TitleText(content="Generate Modelica model"),
            CleanedText(
                content="""The only difference with the previous tutorial is that the library is specified in 
                the command as shwon below."""
            ),
            CommentCodeObject(
                language="python",
                function_name="test_first_model_other_library",
                object=tutorial_path.joinpath("test_tutorials.py"),
            ),
            CleanedText(
                content="""The figure below shows the envelope subcomponent generated using the IDEAS library 
                as opposed to the 
                previous tutorial where Buildings library has been used."""
            ),
            DisplayImage(
                title="Envelope components using IDEAS",
                path="./img/other_library_1.jpg",
            ),
            TitleText(content="Zone template (IDEAS only)"),
            CleanedText(
                content="""By default Trano renders an IDEAS envelope with separate components: an
                IDEAS.Buildings.Components.Zone plus one OuterWall, Window or SlabOnGround array per
                construction. IDEAS also provides IDEAS.Buildings.Components.RectangularZoneTemplate, a single
                component bundling the zone, its four vertical faces, floor and ceiling. Set the space variant
                to rectangular_zone to use it."""
            ),
            DisplayObject(
                language="yaml",
                object=tutorial_path.parent.joinpath("models", "two_zones_ideas_rectangular_zone.yaml"),
            ),
            CleanedText(
                content="""Trano maps the walls onto the four faces of the template (face A takes the
                orientation shared by most surfaces, the other faces are at 90 degree steps from it), lumps the
                windows of a face sharing a glazing into one window, and uses the floor on ground as
                SlabOnGround and a horizontal roof as the ceiling. Surfaces that do not fit the rectangle
                (pitched roofs, a second construction or glazing on the same face, extra orientations) and the
                internal walls stay separate components connected through the template's proBusExt bus, so
                nothing is lost. The variant is only available with the IDEAS library."""
            ),
        ],
    ),
    Tutorial(
        title="Multizone model",
        contents=[
            CleanedText(
                content="""Similarly to changing libraries, building a multi-zone model is a s easy as 
                adding few lines in the
                configuration file. This tutorial shows how to generate a model for a three-zone building."""
            ),
            CleanedText(content="""The configuration below shows the yaml file describing a three-zone building."""),
            DisplayObject(
                language="yaml",
                object=tutorial_path.joinpath("multi_zones.yaml"),
            ),
            CleanedText(
                content="""This time we are going to use a reduced order model defined in the AIXLIB library."""
            ),
            CommentCodeObject(
                language="python",
                function_name="test_multi_zones",
                object=tutorial_path.joinpath("test_tutorials.py"),
            ),
            CleanedText(
                content="""The figure below shows the envelope subcomponent generated using reduced 
                order building component
                from the AIXLIB library."""
            ),
            DisplayImage(title="Envelope components using IDEAS", path="./img/multi_zones.jpg"),
        ],
    ),
    Tutorial(
        title="Multi-zone with ideal heating",
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
            CleanedText(
                content="""
    The following code snippet shows how to create the model using Trano. Which at is point a bit redundant since 
    the interface stays the same.
    And the model generated is mainly based on the yaml configuration file."""
            ),
            CommentCodeObject(
                language="python",
                function_name="test_three_zones_ideal_heating",
                object=tutorial_path.joinpath("test_tutorials.py"),
            ),
            TitleText(content="Outputs"),
            CleanedText(
                content="""The figure below shows the building component of the generated model. One can note 
                that in addition to the
                    building envelope sub-component, a component related to emissions was generated."""
            ),
            DisplayImage(
                title="Envelope components using IDEAS",
                path="./img/ideal_heating_1.jpg",
            ),
            CleanedText(
                content="""Opening this component will show an ideal heting system which will be linked with 
                the sapece 3.
                    Since we only defined one heating system for one of the spaces"""
            ),
            DisplayImage(
                title="Envelope components using IDEAS",
                path="./img/ideal_heating_2.jpg",
            ),
            CleanedText(
                content="""Opening the building envelope component we can note that there are three spaces 
                defined in the model."""
            ),
            DisplayImage(
                title="Envelope components using IDEAS",
                path="./img/ideal_heating_3.jpg",
            ),
        ],
    ),
    Tutorial(
        title="Multi-zone with hydronic heating",
        contents=[
            CleanedText(
                content="""The previous tutorials walked us through idealized examples.
                    In this tutorial, we will simulate a three zone building with a hydronic heating system and try
            to build a house model that is relatively realistic. As usual, the configuration file will be shown and
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
    It defined the boiler, the pump and the different valves in the system. Default parameters are used 
    if not specified.

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
            CleanedText(
                content="""
    The following code snippet shows how to create the model using Trano. This time the IDEAS library will be used."""
            ),
            CommentCodeObject(
                language="python",
                function_name="test_three_zones_hydronic_heating",
                object=tutorial_path.joinpath("test_tutorials.py"),
            ),
            TitleText(content="Outputs"),
            CleanedText(
                content="""The figure below shows the building component of the generated model. One can note that in 
                addition to the
                    building envelope and emission sub-components, components related to the hydronic distribution 
                    system and the production boiler were added."""
            ),
            DisplayImage(title="hydronic_1", path="./img/hydronic_1.jpg"),
            CleanedText(
                content="""Opening the emission component will show the different radiator and valves that are
                 connected to the different spaces
                    and composing the emission system."""
            ),
            DisplayImage(title="hydronic_2", path="./img/hydronic_2.jpg"),
            CleanedText(
                content="""The figure below shows the hydronic distribution system that link the emission system
                 with the boiler."""
            ),
            DisplayImage(title="hydronic_3", path="./img/hydronic_3.jpg"),
            CleanedText(
                content="""Since the IDEAS library was used, the figure below shows the content of the envelope
                 sub-component in which the
                    IDEAS sub components are used."""
            ),
            DisplayImage(title="hydronic_4", path="./img/hydronic_4.jpg"),
        ],
    ),
    Tutorial(
        title="Multi-zone with hydronic heating",
        contents=[
            CleanedText(
                content="""The previous tutorials walked us through idealized examples.
                    In this tutorial, we will simulate a three zone building with a hydronic heating system and try
            to build a house model that is relatively realistic. As usual, the configuration file will be shown and
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
    It defined the boiler, the pump and the different valves in the system. Default parameters are used if not 
    specified.

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
            CleanedText(
                content="""
    The following code snippet shows how to create the model using Trano. This time the IDEAS library will be used."""
            ),
            CommentCodeObject(
                language="python",
                function_name="test_three_zones_hydronic_heating",
                object=tutorial_path.joinpath("test_tutorials.py"),
            ),
            TitleText(content="Outputs"),
            CleanedText(
                content="""The figure below shows the building component of the generated model. One can note that in 
                addition to the
                    building envelope and emission sub-components, components related to the hydronic distribution 
                    system and the production boiler were added."""
            ),
            DisplayImage(title="hydronic_1", path="./img/hydronic_1.jpg"),
            CleanedText(
                content="""Opening the emission component will show the different radiator and valves that are 
                connected to the different spaces
                    and composing the emission system."""
            ),
            DisplayImage(title="hydronic_2", path="./img/hydronic_2.jpg"),
            CleanedText(
                content="""The figure below shows the hydronic distribution system that link the emission system 
                with the boiler."""
            ),
            DisplayImage(title="hydronic_3", path="./img/hydronic_3.jpg"),
            CleanedText(
                content="""Since the IDEAS library was used, the figure below shows the content of the envelope 
                sub-component in which the
                    IDEAS sub components are used."""
            ),
            DisplayImage(title="hydronic_4", path="./img/hydronic_4.jpg"),
        ],
    ),
    Tutorial(
        title="Zone with ventilation",
        contents=[
            CleanedText(content="""The following tutorial shows how to add a ventilation to the building model."""),
            DisplayObject(
                language="yaml",
                object=tutorial_path.joinpath("zone_with_ventilation.yaml"),
            ),
            CleanedText(content="""This time we are going to use zone models based on the ISO 13790."""),
            CommentCodeObject(
                language="python",
                function_name="test_ventilation",
                object=tutorial_path.joinpath("test_tutorials.py"),
            ),
            CleanedText(
                content="""The figure below shows the model generated in which the subcomponent related to 
                ventilation is 
                displayed."""
            ),
            DisplayImage(title="Envelope components using IDEAS", path="./img/ventilation_1.jpg"),
            CleanedText(
                content="""Opening the ventilation component, one can find the different components related to 
                the ventilation system."""
            ),
            DisplayImage(title="Envelope components using IDEAS", path="./img/ventilation_2.jpg"),
        ],
    ),
    Tutorial(
        title="Zone with photovoltaics",
        contents=[
            CleanedText(content="""The following tutorial shows how to add PVs to buildings."""),
            DisplayObject(
                language="yaml",
                object=tutorial_path.joinpath("multizones_with_pv.yaml"),
            ),
            CommentCodeObject(
                language="python",
                function_name="test_multizones_with_pv",
                object=tutorial_path.joinpath("test_tutorials.py"),
            ),
            CleanedText(
                content="""The figure below shows the model generated in which the subcomponent related to PV is 
                added to the already seen components."""
            ),
            DisplayImage(title="Envelope components using IDEAS", path="./img/pv_1.jpg"),
            CleanedText(content="""Opening the solar component, one can find the defined PVs assigned on the model."""),
            DisplayImage(title="Envelope components using IDEAS", path="./img/pv_2.jpg"),
        ],
    ),
]


def gen_docs() -> None:
    for tutorial in TUTORIALS:
        tutorial.write(Path(__file__).parents[2].joinpath("docs", "tutorials", f"{tutorial.name()}.md"))


if __name__ == "__main__":
    gen_docs()
