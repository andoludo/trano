import copy
import json
import tempfile
from collections import Counter
from pathlib import Path
from typing import Any, Dict

import yaml  # type: ignore
from linkml.validator import validate_file  # type: ignore
from pydantic import BaseModel

from trano.construction import Construction, Layer
from trano.material import Material
from trano.models.elements.boiler import Boiler  # noqa: F401
from trano.models.elements.controls.boiler import BoilerControl  # noqa: F401
from trano.models.elements.controls.collector import CollectorControl  # noqa: F401
from trano.models.elements.controls.emission import EmissionControl  # noqa: F401
from trano.models.elements.controls.three_way_valve import (  # noqa: F401
    ThreeWayValveControl,
)
from trano.models.elements.envelope.external_wall import ExternalWall
from trano.models.elements.occupancy import Occupancy
from trano.models.elements.pump import Pump  # noqa: F401
from trano.models.elements.radiator import Radiator  # noqa: F401
from trano.models.elements.space import Space, SpaceParameter
from trano.models.elements.split_valve import SplitValve  # noqa: F401
from trano.models.elements.temperature_sensor import TemperatureSensor  # noqa: F401
from trano.models.elements.three_way_valve import ThreeWayValve  # noqa: F401
from trano.models.elements.valve import Valve  # noqa: F401
from trano.models.elements.weather import Weather
from trano.topology import Network

DATA_MODEL_PATH = Path(__file__).parent.joinpath("trano.yaml")
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


def load_and_enrich_model(model_path: Path) -> EnrichedModel:
    if model_path.suffix == ".yaml":
        load_function = yaml.safe_load
        dump_function = yaml.safe_dump
    elif model_path.suffix == ".json":
        load_function = json.loads
        dump_function = json.dump
    else:
        raise Exception("Invalid file format")
    data = load_function(model_path.read_text())
    data = assign_space_id(data)
    _parse(data)
    with tempfile.NamedTemporaryFile(
        mode="w+", suffix=model_path.suffix, delete=False
    ) as f:
        dump_function(data, f)
    return EnrichedModel(path=Path(f.name), data=data)


# TODO: reduce complexity
def convert_network(name: str, model_path: Path) -> Network:
    network = Network(name=name)
    occupancy = None
    system_counter: Any = Counter()

    enriched_model = load_and_enrich_model(model_path)
    validate_model(enriched_model.path)
    data = enriched_model.data

    materials = {
        material["id"]: Material(**(material | {"name": material["id"]}))
        for material in data["materials"]
    }
    constructions = {}
    for construction in data["constructions"]:
        layers = [
            Layer(**(layer | {"material": materials[layer["material"]]}))
            for layer in construction["layers"]
        ]
        constructions[construction["id"]] = Construction(
            name=construction["id"], layers=layers
        )
    spaces = []
    space_dict = {}
    systems = {}
    for space in data["spaces"]:
        external_boundaries = space["external_boundaries"]
        external_walls = []
        for external_wall in external_boundaries["external_walls"]:
            external_wall_ = ExternalWall(
                **(
                    external_wall
                    | {"construction": constructions[external_wall["construction"]]}
                )
            )
            external_walls.append(external_wall_)
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
        elif v is None and "control" in k:
            COUNTER.update(["control"])  # type: ignore
            data[k] = {"id": f"CONTROL:{COUNTER['control']}"}


def assign_space_id(data: Dict[str, Any]) -> Dict[str, Any]:
    space_counter: Dict[Any, Any] = Counter()
    spaces = []
    for space in data.get("spaces", []):
        space_counter.update(["space"])  # type: ignore
        spaces.append({"id": f"SPACE:{space_counter['space']}"} | space)
    data["spaces"] = spaces
    return data
