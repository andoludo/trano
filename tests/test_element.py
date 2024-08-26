from pathlib import Path
from typing import Callable, List

import yaml
from linkml.validator import validate_file

from trano.models.elements.ahu import BaseAirHandlingUnit
from trano.models.elements.base import Port, DynamicComponentTemplate, LibraryData

from pydantic import create_model

from trano.models.elements.boiler import BaseBoiler, BoilerParameters
from trano.models.elements.boundary import BaseBoundaryComponent
from trano.models.elements.bus import data_bus_factory
from trano.models.elements.controls.ahu import BaseAhuControl
from trano.models.elements.controls.base import PIDParameters
from trano.models.elements.controls.boiler import (
    BaseBoilerControl,
    SimpleBoilerControl,
    BoilerParameters as BoilerControlParameters,
)
from trano.models.elements.controls.collector import BaseCollectorControl
from trano.models.elements.controls.emission import (
    BaseEmissionControl,
    ControlLoopsParameters,
)
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
from trano.models.elements.occupancy import OccupancyComponent, OccupancyParameters
from trano.models.elements.pump import BasePump, PumpParameters
from trano.models.elements.radiator import (
    BaseRadiator,
    BaseIdealRadiator,
    RadiatorParameter,
)
from trano.models.elements.space import IdeasSpace, BuildingsSpace, SpaceParameter
from trano.models.elements.split_valve import BaseSplitValve, SplitValveParameters
from trano.models.elements.temperature_sensor import BaseTemperatureSensor
from trano.models.elements.three_way_valve import (
    BaseThreeWayValve,
    ThreeWayValveParameters,
)
from trano.models.elements.valve import BaseValve, ValveParameters
from trano.models.elements.weather import (
    BuildingsWeatherComponent,
    IdeasWeatherComponent,
    WeatherParameters,
)


def test_validate_schema() -> None:
    for name in ["boiler", "valve", "window"]:
        boiler = Path(
            f"/home/aan/Documents/trano/trano/models/elements/models/{name}.yaml"
        )
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


def to_camel_case(snake_str: str) -> str:
    return "".join(x.capitalize() for x in snake_str.lower().split("_"))


def test_figures():
    components = [
        BaseBoiler(),
        BaseRadiator(),
        BaseIdealRadiator(),
        BaseAirHandlingUnit(),
        BaseBoundaryComponent(),
        data_bus_factory(),
        BaseDamper(),
        BaseDamperDetailed(),
        BaseDuct(),
        OccupancyComponent(),
        BasePump(),
        IdeasSpace(),
        BuildingsSpace(),
        BaseSplitValve(),
        BaseTemperatureSensor(),
        BaseThreeWayValve(),
        BaseValve(),
        BaseBoilerControl(),
        SimpleBoilerControl(),
        BaseCollectorControl(),
        BaseEmissionControl(),
        BaseThreeWayValveControl(),
        BaseVavControl(),
    ]
    for co in components:
        if co.figures:
            a = 12


def test_create_yamls():
    configs = {
        "boiler": {"default": [BaseBoiler()]},
        "radiator": {
            "default": [BaseRadiator(), BaseIdealRadiator()],
        },
        "air_handling_unit": {"default": [BaseAirHandlingUnit()]},
        "boundary": {
            "default": [BaseBoundaryComponent()],
        },
        "data_bus": {"default": [data_bus_factory()]},
        "vav": {
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
        # "merged_external_wall": {"ideas": [IdeasMergedExternalWall()]},
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
                        component_["parameter_processing"] = {
                            "function": function_name,
                            "parameter": parameters,
                        }
                    else:
                        function_name = component.parameter_processing.__name__
                        component_["parameter_processing"] = {"function": function_name}
                if name == "external_wall":
                    component_["classes"] = [to_camel_case(name), "MergedExternalWall"]
                elif "vav" in name:
                    component_["classes"] = [to_camel_case(name).replace("Vav", "VAV")]
                else:
                    component_["classes"] = [to_camel_case(name)]
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
                component_["figures"] = []
                for figure in component.figures:
                    figure_ = figure.model_dump(exclude_defaults=True, exclude_unset=True)
                    component_["figures"].append(figure_)
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


def test_parameters():
    parameters = [
        BoilerParameters(),
        OccupancyParameters(),
        PumpParameters(),
        RadiatorParameter(),
        SpaceParameter(),
        SplitValveParameters(),
        ThreeWayValveParameters(),
        ValveParameters(),
        WeatherParameters(),
        PIDParameters(),
        BoilerControlParameters(),
        ControlLoopsParameters(),
    ]
    for p in parameters:
        p.model_json_schema()


def _get_range(annotation):
    if "Literal" in str(annotation) or "str" in str(annotation):
        return "string"
    elif "float" in str(annotation):
        return "float"
    elif "int" in str(annotation):
        return "integer"
    elif "Boolean" in str(annotation) or "bool" in str(annotation):
        return "boolean"
    else:
        raise Exception(str(annotation))


def _get_type(_type):
    if _type == "string":
        return str
    elif _type == "float":
        return float
    elif _type == "integer":
        return int
    elif _type == "boolean":
        return bool
    else:
        raise Exception("Unknown type")


def _get_default(v):
    value = v["ifabsent"].replace(v["range"], "")[1:-1]
    try:
        return _get_type(v["range"])(value)
    except Exception as e:
        if value == "None":
            return None
        raise e


from pydantic.fields import computed_field, Field

computed_fields = {
    "nominal_mass_flow_rate_boiler": "lambda self: self.sca_fac_rad * self.nominal_heating_power / self.dt_rad_nominal / 4200",
    "nominal_mass_flow_radiator_loop": "lambda self: self.sca_fac_rad * self.nominal_heating_power / self.dt_rad_nominal / 4200",
    "v_flow": "lambda self: f'{self.nominal_mass_flow_rate_boiler}' '/1000*{0.5,1}'",
    "dry_mass_of_radiator_that_will_be_lumped_to_water_heat_capacity": "lambda self: 0.0263 * abs(self.nominal_heating_power_positive_for_heating)",
    "water_volume_of_radiator": "lambda self:5.8e-5 * abs(self.nominal_heating_power_positive_for_heating)",
    "volume": "lambda self:self.floor_area * self.average_room_height",
}


def test_dump_schema():
    from linkml_runtime.dumpers import json_dumper
    from linkml.generators.pydanticgen import PydanticGenerator

    parameters_ = [
        (BoilerParameters, ["Boiler"]),
        (OccupancyParameters, ["Occupancy"]),
        (PumpParameters, ["Pump"]),
        (RadiatorParameter, ["Radiator"]),
        (SpaceParameter, ["Space"]),
        (SplitValveParameters, ["SplitValve"]),
        (ThreeWayValveParameters, ["ThreeWayValve"]),
        (ValveParameters, ["Valve"]),
        (WeatherParameters, ["Weather"]),
        (PIDParameters, ["ThreeWayValveControl", "CollectorControl"]),
        (ControlLoopsParameters, ["EmissionControl"]),
        (BoilerControlParameters, ["BoilerControl"]),
    ]
    parameter_path = Path("/home/aan/Documents/trano/trano/data_models/parameters.yaml")
    parameter = {}
    for par, classes_ in parameters_:
        if "<class 'trano.models.elements.controls.boiler.BoilerParameters'>" in str(
            par
        ):
            par_name = "BoilerControlParameters"
        else:
            par_name = par.__name__
        parameter[par_name] = {"attributes": {}, "classes": classes_}

        for field_name, field in par.model_fields.items():
            if (
                par_name == SplitValveParameters.__name__
                and field_name == "m_flow_nominal"
            ):
                range = "string"
                default = "string(0.008*{1,-1,-1})"
            else:
                range = _get_range(field.annotation)
                default = f"{_get_range(field.annotation)}({field.default})"
            parameter[par_name]["attributes"][field_name] = {
                "range": range,
                "description": str(field.description),
                "alias": str(field.alias),
                "ifabsent": default,
            }
        for field_name, field in par.model_computed_fields.items():
            parameter[par_name]["attributes"][field_name] = {
                "func": computed_fields.get(field_name),
                "type": field.return_type.__name__,
                "alias": field.alias,
            }
    with parameter_path.open("w+") as f:
        yaml.safe_dump(parameter, f)


def test_computed_field():
    m = create_model(
        "Test",
        a=(int, Field(default=12, alias="b", description="c")),
        b=computed_field(lambda self: 10 * self.a, return_type=float),
    )
    m().b


from pydantic.fields import FieldInfo


# def test_parameters_loading():
#     parameter_path = Path("/home/aan/Documents/trano/trano/data_models/parameters.yaml")
#     data = yaml.safe_load(parameter_path.read_text())
#     # a = 12
#     classes = []
#     for name, parameter in data.items():
#         attrib_ = {
#             k: (
#                 _get_type(v["range"]),
#                 FieldInfo(default=_get_default(v), alias=v.get("alias", None), description=v.get("description", None)),
#             )
#             for k, v in parameter["attributes"].items()
#         }
#         classes.append(create_model(
#             name, **attrib_
#         ))
#     a = 12
