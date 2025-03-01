import copy
import json
import tempfile
from collections import Counter
from functools import partial
from pathlib import Path
from typing import Any, Dict, List, Optional

import yaml
from linkml.validator import validate_file  # type: ignore
from pydantic import BaseModel

from trano.data.include import Loader
from trano.data_models.converter import converter
from trano.elements import (
    Boundary,
    ExternalWall,
    FloorOnGround,
    InternalElement,
    Window,
    param_from_config,
)
from trano.elements.construction import (
    Construction,
    Gas,
    GasLayer,
    Glass,
    GlassLayer,
    GlassMaterial,
    Layer,
    Material,
)
from trano.elements.control import BoilerControl  # noqa: F401
from trano.elements.space import Space

# TODO: fix these imports
from trano.elements.system import Boiler  # noqa: F401
from trano.elements.system import AirHandlingUnit, Occupancy, Weather
from trano.elements.types import Tilt
from trano.elements.utils import import_element_function
from trano.elements.library.library import Library
from trano.topology import Network

SpaceParameter = param_from_config("Space")
DATA_MODEL_PATH = Path(__file__).parent.joinpath("trano_final.yaml")
COUNTER: Dict[Any, Any] = Counter()


def to_camel_case(snake_str: str) -> str:
    return "".join(x.capitalize() for x in snake_str.lower().split("_"))


class Component(BaseModel):
    name: str
    component_instance: Any


def validate_model(data: Any, suffix: str) -> None:  # noqa: ANN401
    with tempfile.NamedTemporaryFile(mode="w+", suffix=suffix, delete=False) as f2:
        enriched_path = Path(f2.name)
        enriched_path.write_text(json.dumps(data))
        report = validate_file(enriched_path, DATA_MODEL_PATH, "Building")
        if report.results:
            raise Exception("Invalid model.")


def _instantiate_component(component_: Dict[str, Any]) -> Component:
    component = copy.deepcopy(component_)
    components = component.items()
    if len(components) != 1:
        raise NotImplementedError("Only one component type is allowed")
    component_type, component_parameters = next(iter(components))
    component_parameters.pop("inlets", None)
    component_parameters.pop("outlets", None)
    component_type = to_camel_case(component_type)
    # TODO: just find a way to import the required components directly here!

    component_class = import_element_function(component_type)
    name = component_parameters.pop("id")
    component_parameters.update({"name": name})
    component_parameters_class = param_from_config(component_type)
    if "parameters" in component_parameters and component_parameters_class is not None:
        parameters = component_parameters_class(**component_parameters["parameters"])
        component_parameters = component_parameters | {"parameters": parameters}
    if component_parameters.get("control"):
        controls = component_parameters["control"].items()
        if len(controls) != 1:
            raise NotImplementedError("Only one component type is allowed")
        control_type, control_parameter = next(iter(controls))
        control_class = import_element_function(to_camel_case(control_type))
        control_name = control_parameter.pop("id", None)
        if control_name:
            control_parameter.update({"name": control_name})
        component_parameter_class = param_from_config(f"{to_camel_case(control_type)}")
        if "parameters" in control_parameter and component_parameter_class is not None:
            parameters = component_parameter_class(**control_parameter["parameters"])
            control_parameter = control_parameter | {"parameters": parameters}
        control = control_class(**control_parameter)
        component_parameters.update({"control": control})
    component = component_class(**component_parameters)
    return Component(name=name, component_instance=component)


class EnrichedModel(BaseModel):
    data: Any


def load_and_enrich_model(model_path: Path) -> EnrichedModel:
    if model_path.suffix == ".yaml":
        load_function = partial(yaml.load, Loader=Loader)
        dump_function = yaml.safe_dump
    elif model_path.suffix == ".json":
        load_function = json.loads  # type: ignore
        dump_function = json.dump  # type: ignore # TODO: why?
    else:
        raise Exception("Invalid file format")
    data = load_function(model_path.read_text())
    data = assign_space_id(data)
    _parse(data)
    with tempfile.NamedTemporaryFile(
        mode="w+", suffix=model_path.suffix, delete=False  # TODO: delete later?
    ) as f:
        dump_function(data, f)

    converted_data = converter(
        input=str(Path(f.name)),
        target_class="Building",
        schema=str(DATA_MODEL_PATH),
        output_format=model_path.suffix[1:],
    )
    return EnrichedModel(data=converted_data)


def _build_materials(data: Dict[str, Any]) -> Dict[str, Any]:
    materials = {}
    material_types = {"material": Material, "gas": Gas, "glass_material": GlassMaterial}
    for material_type, material_class in material_types.items():
        for material in data.get(material_type, []):
            materials[material["id"]] = material_class(
                **(material | {"name": material["id"]})
            )
    return materials


def _merge_default(data: Dict[str, Any]) -> Dict[str, Any]:
    data["constructions"] = data.get("constructions", []) + data.get("default", {}).get(
        "constructions", []
    )
    data["material"] = data.get("material", []) + data.get("default", {}).get(
        "material", []
    )
    data["glazings"] = data.get("glazings", []) + data.get("default", {}).get(
        "glazings", []
    )
    data["gas"] = data.get("gas", []) + data.get("default", {}).get("gas", [])
    data["glass_material"] = data.get("glass_material", []) + data.get(
        "default", {}
    ).get("glass_material", [])
    data.pop("default", None)
    return data


# TODO: reduce complexity
def convert_network(  # noqa: PLR0915, C901, PLR0912
    name: str, model_path: Path, library: Optional[Library] = None
) -> Network:
    network = Network(name=name, library=library)
    occupancy = None
    system_counter: Any = Counter()

    enriched_model = load_and_enrich_model(model_path)
    validate_model(enriched_model.data, model_path.suffix)
    data = enriched_model.data
    data = _merge_default(data)
    materials = _build_materials(data)
    constructions: Dict[str, Construction | Glass] = {}
    for construction in data["constructions"]:
        layers = [
            Layer(**(layer | {"material": materials[layer["material"]]}))
            for layer in construction["layers"]
        ]
        constructions[construction["id"]] = Construction(
            name=construction["id"], layers=layers
        )
    for glazing in data.get("glazings", []):
        glazing_layers: List[GasLayer | GlassLayer] = []
        for layer in glazing["layers"]:
            if layer.get("gas", None):
                glazing_layers.append(
                    GasLayer(
                        thickness=layer["thickness"], material=materials[layer["gas"]]
                    )
                )
            if layer.get("glass", None):
                glazing_layers.append(
                    GlassLayer(
                        thickness=layer["thickness"], material=materials[layer["glass"]]
                    )
                )
        constructions[glazing["id"]] = Glass(
            name=glazing["id"],
            layers=glazing_layers,
            u_value_frame=glazing["u_value_frame"],
        )

    spaces = []
    space_dict = {}
    systems = {}
    for space in data["spaces"]:
        external_boundaries = space["external_boundaries"]
        external_walls: List[ExternalWall | Window | FloorOnGround] = []
        for external_wall in external_boundaries.get("external_walls", []):
            external_wall_ = ExternalWall(
                **(
                    external_wall
                    | {"construction": constructions[external_wall["construction"]]}
                )
            )
            external_walls.append(external_wall_)
        for window in external_boundaries.get("windows", []):
            window_ = Window(
                **(window | {"construction": constructions[window["construction"]]})
            )
            external_walls.append(window_)
        for floor_on_ground in external_boundaries.get("floor_on_grounds", []):
            floor_on_ground_ = FloorOnGround(
                **(
                    floor_on_ground
                    | {"construction": constructions[floor_on_ground["construction"]]}
                )
            )
            external_walls.append(floor_on_ground_)
        occupancy_parameter_class = param_from_config("Occupancy")
        if space.get("occupancy") is not None and occupancy_parameter_class is not None:
            system_counter.update(["occupancy"])

            occupancy_ = space["occupancy"]
            occupancy = Occupancy(
                **(
                    occupancy_
                    | {"name": f"occupancy_{system_counter['occupancy']}"}
                    | {
                        "parameters": occupancy_parameter_class(
                            **occupancy_.get("parameters", {})
                        )
                    }
                )
            )
        elif (
            network.library.default_parameters.get("occupancy") is not None
            and occupancy_parameter_class is not None
        ):
            system_counter.update(["occupancy"])
            occupancy = Occupancy(
                name=f"occupancy_{system_counter['occupancy']}",
                parameters=occupancy_parameter_class(
                    **network.library.default_parameters["occupancy"]
                ),
            )
        emissions = []
        for emission in space.get("emissions", []):
            emission_ = _instantiate_component(emission)
            systems[emission_.name] = emission_.component_instance
            emissions.append(emission_.component_instance)
        ventilation_inlets = []
        for inlet in space.get("ventilation_inlets", []):
            inlet_ = _instantiate_component(inlet)
            systems[inlet_.name] = inlet_.component_instance
            ventilation_inlets.append(inlet_.component_instance)
        ventilation_outlets = []
        for outlet in space.get("ventilation_outlets", []):
            outlet_ = _instantiate_component(outlet)
            systems[outlet_.name] = outlet_.component_instance
            ventilation_outlets.append(outlet_.component_instance)
        if SpaceParameter is None:
            raise Exception("SpaceParameter is not defined")
        space_ = Space(
            name=space["id"],
            external_boundaries=external_walls,
            occupancy=occupancy,
            parameters=SpaceParameter(**space["parameters"]),
            emissions=emissions,
            ventilation_inlets=ventilation_inlets,
            ventilation_outlets=ventilation_outlets,
        )
        space_dict[space["id"]] = space_
        spaces.append(space_)
    create_internal = not data.get("internal_walls", [])

    if (
        data.get("weather", {}).get("parameters")
        and param_from_config("Weather") is not None
    ):
        weather = Weather(
            name="weather",
            parameters=param_from_config("Weather")(**data["weather"]["parameters"]),  # type: ignore
        )
    else:
        weather = Weather(name="weather")

    network.add_boiler_plate_spaces(
        spaces, weather=weather, create_internal=create_internal
    )
    for internal_wall in data.get("internal_walls", []):
        space_1 = space_dict[internal_wall["space_1"]]
        space_2 = space_dict[internal_wall["space_2"]]
        internal_element = InternalElement(
            name=f"internal_{space_1.name}_{space_2.name}_{internal_wall['construction'].lower().split(':')[0]}",
            surface=internal_wall["surface"],
            azimuth=10,
            construction=constructions[internal_wall["construction"]],
            tilt=Tilt.wall,
        )
        network.connect_spaces(space_1, space_2, internal_element=internal_element)

    edges = []
    for system in data["systems"]:
        system_ = _instantiate_component(system)
        systems[system_.name] = system_.component_instance
    for system in data["systems"]:
        for value in system.values():
            edges += [
                (systems[value["id"]], systems[outlet])
                for outlet in value.get("outlets", [])
            ]
            edges += [
                (systems[inlet], systems[value["id"]])
                for inlet in value.get("inlets", [])
            ]
    for edge in edges:
        network.connect_systems(*edge)

    ahus = [n for n in network.graph.nodes if isinstance(n, AirHandlingUnit)]
    if ahus:
        boundary = Boundary(name="boundary")
        network.connect_elements(boundary, ahus[0])
        weather = next(n for n in network.graph.nodes if isinstance(n, Weather))
        network.connect_elements(boundary, weather)
    for solar in data.get("solar", []):
        solar_ = _instantiate_component(solar)
        solar_.component_instance.add_to_network(network)

    return network


def convert_model(name: str, model_path: Path) -> str:
    network = convert_network(name, model_path)
    return network.model()


def _parse(data: Dict[str, Any]) -> None:
    for k, v in data.items():
        if isinstance(v, dict):
            _parse(v)
        elif isinstance(v, list):
            for i in v:
                if isinstance(i, dict):
                    _parse(i)
        elif v is None:
            if "control" in k:
                COUNTER.update(["control"])  # type: ignore
                data[k] = {"id": f"CONTROL:{COUNTER['control']}"}
            if "occupancy" in k:
                COUNTER.update(["occupancy"])  # type: ignore
                data[k] = {"parameters": {"occupancy": "3600 * {7, 19}"}}


def assign_space_id(data: Dict[str, Any]) -> Dict[str, Any]:
    space_counter: Dict[Any, Any] = Counter()
    spaces = []
    for space in data.get("spaces", []):
        space_counter.update(["space"])  # type: ignore
        spaces.append({"id": f"SPACE:{space_counter['space']}"} | space)
    data["spaces"] = spaces
    return data
