import json
from collections import Counter
from pathlib import Path
from typing import Any, Dict

from trano.construction import Construction, Layer
from trano.material import Material
from trano.models.elements.boiler import Boiler
from trano.models.elements.controls.boiler import BoilerControl
from trano.models.elements.controls.collector import CollectorControl
from trano.models.elements.controls.emission import EmissionControl
from trano.models.elements.controls.three_way_valve import ThreeWayValveControl
from trano.models.elements.envelope.external_wall import ExternalWall
from trano.models.elements.occupancy import Occupancy
from trano.models.elements.pump import Pump
from trano.models.elements.radiator import Radiator
from trano.models.elements.space import Space, SpaceParameter
from trano.models.elements.split_valve import SplitValve
from trano.models.elements.three_way_valve import ThreeWayValve
from trano.models.elements.valve import Valve
from trano.models.elements.weather import Weather
from trano.topology import Network


# TODO: reduce complexity
def convert_model(name: str, model_path: Path) -> str:  # noqa: PLR0912, PLR0915, C901
    network = Network(name=name)
    occupancy = None
    house_json = model_path
    data = json.loads(house_json.read_text())
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
        if space.get("occupancy"):
            occupancy = Occupancy(**space["occupancy"])
        emissions = []
        for emission in space["emissions"]:
            if emission.get("radiator"):
                radiator = emission["radiator"]
                name = radiator.pop("id")
                radiator = Radiator(**(radiator | {"name": name}))
                emissions.append(radiator)
                systems[name] = radiator
            if emission.get("valve"):
                valve = emission["valve"]
                name = valve.pop("id")
                control = None
                if valve.get("control"):
                    control = valve["control"]
                    if control.get("emission_control"):
                        control = EmissionControl(**control["emission_control"])
                valve = Valve(**(valve | {"control": control, "name": name}))
                systems[name] = valve
                emissions.append(valve)
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
    system_counter: Dict[str, Any] = Counter()
    for system in data["systems"]:
        for system_type, value in system.items():
            if system_type == "boiler":

                boiler = Boiler(
                    name=value["id"], control=BoilerControl(name="boiler_control")
                )
                systems[value["id"]] = boiler
            if system_type == "three_way_valve":
                system_counter.update([system_type])
                three_way_valve = ThreeWayValve(
                    name=value["id"],
                    control=ThreeWayValveControl(
                        name=f"{system_type}_{system_counter[system_type]}"
                    ),
                )
                systems[value["id"]] = three_way_valve
            if system_type == "pump":
                pump = Pump(
                    name=value["id"], control=CollectorControl(name="pump_control")
                )
                systems[value["id"]] = pump
            if system_type == "split_valve":
                split_valve = SplitValve(name=value["id"])
                systems[value["id"]] = split_valve
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
    model = network.model()
    return model
