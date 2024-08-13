from pathlib import Path
from typing import Callable, List

import yaml
from linkml.validator import validate_file

from trano.models.elements.ahu import BaseAirHandlingUnit
from trano.models.elements.base import Port, DynamicComponentTemplate, LibraryData

from pydantic import create_model

from trano.models.elements.boiler import BaseBoiler
from trano.models.elements.boundary import BaseBoundaryComponent
from trano.models.elements.bus import data_bus_factory
from trano.models.elements.controls.ahu import BaseAhuControl
from trano.models.elements.controls.boiler import BaseBoilerControl, SimpleBoilerControl
from trano.models.elements.controls.collector import BaseCollectorControl
from trano.models.elements.controls.emission import BaseEmissionControl
from trano.models.elements.controls.three_way_valve import BaseThreeWayValveControl
from trano.models.elements.controls.vav import BaseVavControl
from trano.models.elements.damper import BaseDamperDetailed, BaseDamper
from trano.models.elements.duct import BaseDuct
from trano.models.elements.envelope.external_wall import IdeasMergedExternalWall
from trano.models.elements.envelope.floor_on_ground import IdeasFloorOnGround
from trano.models.elements.envelope.internal_element import (
    BuildingsInternalElement,
    IdeasInternalElement,
)
from trano.models.elements.envelope.window import IdeasMergedWindows
from trano.models.elements.occupancy import OccupancyComponent
from trano.models.elements.pump import BasePump
from trano.models.elements.radiator import BaseRadiator, BaseIdealRadiator
from trano.models.elements.space import IdeasSpace, BuildingsSpace
from trano.models.elements.split_valve import BaseSplitValve
from trano.models.elements.temperature_sensor import BaseTemperatureSensor
from trano.models.elements.three_way_valve import BaseThreeWayValve
from trano.models.elements.valve import BaseValve
from trano.models.elements.weather import (
    BuildingsWeatherComponent,
    IdeasWeatherComponent,
)


def test_validate_schema() -> None:
    for name in ["boiler", "valve", "window"]:
        boiler = Path(f"/home/aan/Documents/trano/trano/models/elements/models/{name}.yaml")
        data_model_path = (
            Path(__file__)
            .parents[1]
            .joinpath("trano", "models", "elements", "element.yaml")
        )
        report = validate_file(boiler, data_model_path, "Building")
        assert report.results == []


def test_load():
    boiler = Path(__file__).parents[1].joinpath("tests", "boiler.yaml")
    data = yaml.safe_load(boiler.read_text())
    for component in data["components"]:
        dynamic_component = DynamicComponentTemplate(**component["component_template"])
        ports = []
        for port in component["ports"]:
            ports.append(Port(**port))
        test = create_model(
            "BaseBoiler",
            __base__=LibraryData,
            ports_factory=(Callable[[], List[Port]], lambda: ports),
            component_template=(DynamicComponentTemplate, dynamic_component),
        )

        a = 12


import yaml_include


def test_pyaml_include():

    yaml.add_constructor(
        "!inc",
        yaml_include.Constructor(
            base_dir="/home/aan/Documents/trano/trano/data_models/"
        ),
    )
    with open("/home/aan/Documents/trano/trano/data_models/trano.yaml") as f:
        data = yaml.full_load(f)
        a = 12


def test_create_yaml():
    a = BaseAirHandlingUnit()
    component = a.model_dump(include=["template", "variant"])
    a.ports_factory()
    ports = []
    for p in a.ports_factory():
        port = p.model_dump(exclude_defaults=True)
        port["targets"] = [t.__name__ for t in port["targets"]]
        if "flow" in port:
            port["flow"] = port["flow"].value
        ports.append(port)
    component["ports"] = ports

    component_template = a.component_template.model_dump(
        exclude_defaults=True,
        exclude={"bus": {"integer_outputs": {"__all__": {"input_model"}}}},
    )
    component["component_template"] = component_template
    ff = Path("/home/aan/Documents/trano/tests/ahu.yaml")
    with ff.open("w+") as f:
        yaml.safe_dump({"components": [component]}, f)


def test_create_yamls():
    configs = {
        "boiler": {"default": [BaseBoiler()]},
        "radiator": {
            "default": [BaseRadiator(), BaseIdealRadiator()],
        },
        "ahu": {"default": [BaseAirHandlingUnit()]},
        "boundary": {
            "default": [BaseBoundaryComponent()],
        },
        "bus": {"default": [data_bus_factory()]},
        "damper": {
            "default": [BaseDamper(), BaseDamperDetailed()],
        },
        "duct": {"default": [BaseDuct()]},
        "occupancy": {"default": [OccupancyComponent()]},
        "pump": {"default": [BasePump()]},
        "space": {"ideas": [IdeasSpace()], "buildings": [BuildingsSpace()]},
        "split_valve": {"default": [BaseSplitValve()]},
        "temperature_sensor": {"default": [BaseTemperatureSensor()]},
        "three_way_valve": {"default": [BaseThreeWayValve()]},
        "valve": {"default": [BaseValve()]},
        "weather": {
            "ideas": [IdeasWeatherComponent()],
            "buildings": [BuildingsWeatherComponent()],
        },
        "external_wall": {"ideas": [IdeasMergedExternalWall()]},
        "floor_on_ground": {"ideas": [IdeasFloorOnGround()]},
        "internal_element": {
            "ideas": [IdeasInternalElement()],
            "buildings": [BuildingsInternalElement()],
        },
        "merged_wall": {"ideas": [IdeasMergedExternalWall()]},
        "merged_windows": {"ideas": [IdeasMergedWindows()]},
        "window": {"ideas": [IdeasMergedWindows()]},
        "ahu_control": {"default": [BaseAhuControl()]},
        "boiler_control": {
            "default": [BaseBoilerControl(), SimpleBoilerControl()],
        },
        "collector_control": {
            "default": [BaseCollectorControl()],
        },
        "emission_control": {
            "default": [BaseEmissionControl()],
        },
        "three_way_valve_control": {
            "default": [BaseThreeWayValveControl()],
        },
        "vav_control": {"default": [BaseVavControl()]},
    }
    for name, libraries in configs.items():
        components = []
        for library_name, components__ in libraries.items():
            for component in components__:
                component_ = component.model_dump(include=["template", "variant"])
                if component.parameter_processing:
                    if hasattr(component.parameter_processing, "keywords"):
                        parameters = component.parameter_processing.keywords
                        parameters = list(parameters.values())[0]
                        if isinstance(parameters, set):
                            parameters = list(parameters)
                        function_name = component.parameter_processing.func.__name__
                        component_["parameter_processing"] = {"function": function_name, "parameter":parameters}
                    else:
                        function_name = component.parameter_processing.__name__
                        component_["parameter_processing"] = {"function": function_name}
                component_["library"] = library_name
                component.ports_factory()
                ports = []
                for p in component.ports_factory():
                    port = p.model_dump(exclude_defaults=True)
                    if "targets" in port:
                        port["targets"] = [t.__name__ for t in port["targets"]]
                    if "flow" in port:
                        port["flow"] = port["flow"].value
                    ports.append(port)
                component_["ports"] = ports
                if component.component_template:
                    component_template = component.component_template.model_dump(
                        exclude_defaults=True,
                        exclude={
                            "bus": {
                                "integer_outputs": {"__all__": {"input_model"}},
                                "real_outputs": {"__all__": {"input_model"}},
                                "boolean_outputs": {"__all__": {"input_model"}},
                                "integer_inputs": {"__all__": {"input_model"}},
                                "real_inputs": {"__all__": {"input_model"}},
                                "boolean_inputs": {"__all__": {"input_model"}},
                            }
                        },
                    )
                    component_["component_template"] = component_template
                components.append(component_)
        ff = Path(f"/home/aan/Documents/trano/trano/models/elements/models/{name}.yaml")
        with ff.open("w+") as f:
            yaml.safe_dump({"components": components}, f)


def test_alias():
    class Alias(yaml.YAMLObject):
        yaml_tag = "!alias"

        def __init__(self, alias):
            self.alias = alias

        @classmethod
        def to_yaml(cls, dumper, data):
            return dumper.represent_scalar(cls.yaml_tag, data.alias)

    class Anchor(yaml.YAMLObject):
        yaml_tag = "!anchor"

        def __init__(self, anchor):
            self.anchor = anchor

        @classmethod
        def to_yaml(cls, dumper, data):
            node = dumper.represent_scalar(cls.yaml_tag, data.anchor)
            node.anchor = data.anchor
            return node

    data = {
        "real_input": Anchor("id001"),
        "real_input_alias": Alias("id001"),
        "real_output": Anchor("id002"),
    }

    yaml_str = yaml.dump(data, default_flow_style=False, allow_unicode=True)

    print(yaml_str)


def test_create_radiator_yaml():
    base = BaseRadiator()
    ideal = BaseIdealRadiator()
    components = []
    for component in [base, ideal]:
        component_ = component.model_dump(include=["template", "variant"])
        component.ports_factory()
        ports = []
        for p in component.ports_factory():
            port = p.model_dump(exclude_defaults=True)
            if "targets" in port:
                port["targets"] = [t.__name__ for t in port["targets"]]
            if "flow" in port:
                port["flow"] = port["flow"].value
            ports.append(port)
        component_["ports"] = ports
        if component.component_template:
            component_template = component.component_template.model_dump(
                exclude_defaults=True,
                exclude={"bus": {"integer_outputs": {"__all__": {"input_model"}}}},
            )
            component_["component_template"] = component_template
        components.append(component_)
    ff = Path("/home/aan/Documents/trano/tests/radiator.yaml")
    with ff.open("w+") as f:
        yaml.safe_dump({"components": components}, f)
