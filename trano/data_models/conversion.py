import copy
import json
import os
import subprocess
import tempfile
from collections import Counter
from pathlib import Path
from typing import Any, Dict, List

import yaml
from linkml.validator import validate_file  # type: ignore
from pydantic import BaseModel

from trano.elements import ExternalWall, FloorOnGround, Window, param_from_config
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
from trano.elements.control import (  # noqa: F401
    BoilerControl,
    CollectorControl,
    EmissionControl,
    ThreeWayValveControl,
)
from trano.elements.space import Space

# TODO: fix these imports
from trano.elements.system import (  # noqa: F401
    Boiler,
    Occupancy,
    Pump,
    Radiator,
    SplitValve,
    TemperatureSensor,
    ThreeWayValve,
    Valve,
    Weather,
)
from trano.topology import Network

SpaceParameter = param_from_config("Space")
DATA_MODEL_PATH = Path(__file__).parent.joinpath("trano_final.yaml")
COUNTER: Dict[Any, Any] = Counter()


def to_camel_case(snake_str: str) -> str:
    return "".join(x.capitalize() for x in snake_str.lower().split("_"))


class Component(BaseModel):
    name: str
    component_instance: Any


def validate_model(model_path: Path) -> None:

    report = validate_file(model_path, DATA_MODEL_PATH, "Building")
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
    component_class = globals()[component_type]
    name = component_parameters.pop("id")
    component_parameters.update({"name": name})
    if component_parameters.get("control"):
        controls = component_parameters["control"].items()
        if len(controls) != 1:
            raise NotImplementedError("Only one component type is allowed")
        control_type, control_parameter = next(iter(controls))
        control_class = globals()[to_camel_case(control_type)]
        control_name = control_parameter.pop("id", None)
        if control_name:
            control_parameter.update({"name": control_name})
        control = control_class(**control_parameter)
        component_parameters.update({"control": control})
    component = component_class(**component_parameters)
    return Component(name=name, component_instance=component)


class EnrichedModel(BaseModel):
    data: Dict[str, Any]
    path: Path


def convert(schema: Path, input_file: Path, target: str, output: Path) -> bool:
    root_path = Path(__file__).parents[1]
    os.chdir(root_path)
    command = [
        "poetry",
        "run",
        "linkml-convert",
        "-o",
        f"{output}",
        "-t",
        target,
        "-C",
        "Building",
        "-s",
        str(schema),
        f"{input_file}",
    ]
    # TODO: use (python_module = PythonGenerator(schema).compile_module()) instead of console.
    process = subprocess.run(
        command, check=True, capture_output=True, text=True  # noqa: S603
    )
    return process.returncode == 0


def load_and_enrich_model(model_path: Path) -> EnrichedModel:
    if model_path.suffix == ".yaml":
        load_function = yaml.safe_load
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
        mode="w+", suffix=model_path.suffix, delete=False
    ) as f:
        dump_function(data, f)
    with tempfile.NamedTemporaryFile(
        mode="w+", suffix=model_path.suffix, delete=False
    ) as f2:
        enriched_path = Path(f2.name)
    convert(DATA_MODEL_PATH, Path(f.name), model_path.suffix[1:], enriched_path)
    return EnrichedModel(
        path=enriched_path, data=load_function(enriched_path.read_text())
    )


def _build_materials(data: Dict[str, Any]) -> Dict[str, Any]:
    materials = {}
    material_types = {"material": Material, "gas": Gas, "glass_material": GlassMaterial}
    for material_type, material_class in material_types.items():
        for material in data.get(material_type, []):
            materials[material["id"]] = material_class(
                **(material | {"name": material["id"]})
            )
    return materials


# TODO: reduce complexity
def convert_network(name: str, model_path: Path) -> Network:  # noqa: PLR0912, C901
    network = Network(name=name)
    occupancy = None
    system_counter: Any = Counter()

    enriched_model = load_and_enrich_model(model_path)
    validate_model(enriched_model.path)
    data = enriched_model.data

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
            if "gas" in layer:
                glazing_layers.append(
                    GasLayer(
                        thickness=layer["thickness"], material=materials[layer["gas"]]
                    )
                )
            if "glass" in layer:
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
        if space.get("occupancy") is not None:
            system_counter.update(["occupancy"])
            occupancy = Occupancy(
                **(
                    space["occupancy"]
                    | {"name": f"occupancy_{system_counter['occupancy']}"}
                )
            )
        emissions = []
        for emission in space["emissions"]:
            emission_ = _instantiate_component(emission)
            systems[emission_.name] = emission_.component_instance
            emissions.append(emission_.component_instance)
        if SpaceParameter is None:
            raise Exception("SpaceParameter is not defined")
        space_ = Space(
            name=space["id"],
            external_boundaries=external_walls,
            occupancy=occupancy,
            parameters=SpaceParameter(**space["parameters"]),
            emissions=emissions,
        )
        space_dict[space_.name] = space_
        spaces.append(space_)
    network.add_boiler_plate_spaces(spaces, weather=Weather(name="weather"))
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