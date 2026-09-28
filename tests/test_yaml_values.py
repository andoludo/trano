"""Check that the values of a YAML building description reach the generated Modelica model.

The golden-file tests only detect *changes* in a generated model. The tests below
compare the generated model with the YAML input itself, value by value, so that a
value silently dropped or replaced by a default is caught.

Known gaps are marked ``xfail(strict=True)``: they document values that are not
transmitted yet and will turn into failures as soon as they are, so the marker gets
removed together with the fix.
"""

import re
from collections import Counter
from pathlib import Path
from typing import Any

import pytest
import yaml

from trano.data_models.conversion import convert_network
from trano.elements.library.library import Library

MODEL_PATH = Path(__file__).parent.joinpath("models", "house_values.yaml")
NUMBER = r"[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?"
BUILDINGS_TILTS = {
    "wall": "Buildings.Types.Tilt.Wall",
    "floor": "Buildings.Types.Tilt.Floor",
    "ceiling": "Buildings.Types.Tilt.Ceiling",
    "pitched_roof_35": "0.611",
}


# --------------------------------------------------------------------------- #
# YAML input (single source of truth for the expected values)
# --------------------------------------------------------------------------- #

DATA: dict[str, Any] = yaml.safe_load(MODEL_PATH.read_text())
MATERIALS: dict[str, dict[str, Any]] = {
    material["id"]: material
    for material_type in ("material", "gas", "glass_material")
    for material in DATA[material_type]
}
CONSTRUCTIONS: dict[str, dict[str, Any]] = {construction["id"]: construction for construction in DATA["constructions"]}
GLAZINGS: dict[str, dict[str, Any]] = {glazing["id"]: glazing for glazing in DATA["glazings"]}
SPACES: list[dict[str, Any]] = DATA["spaces"]
EMISSIONS: list[tuple[str, dict[str, Any]]] = [
    (space["id"], emission["radiator"]) for space in SPACES for emission in space.get("emissions", [])
]


def _used_constructions() -> set[str]:
    boundaries = [
        boundary
        for space in SPACES
        for boundary_type in ("external_walls", "floor_on_grounds")
        for boundary in space["external_boundaries"].get(boundary_type, [])
    ]
    return {boundary["construction"] for boundary in boundaries} | {
        internal_wall["construction"] for internal_wall in DATA["internal_walls"]
    }


# Only the constructions referenced by a boundary are written to the model.
USED_CONSTRUCTIONS = sorted(_used_constructions())
USED_MATERIALS = sorted(
    {layer["material"] for name in USED_CONSTRUCTIONS for layer in CONSTRUCTIONS[name]["layers"]}
    | {layer.get("glass") or layer["gas"] for glazing in GLAZINGS.values() for layer in glazing["layers"]}
)


# --------------------------------------------------------------------------- #
# Modelica parsing helpers
# --------------------------------------------------------------------------- #


def modelica_name(identifier: str) -> str:
    return identifier.lower().replace(":", "_")


def optional_call_arguments(text: str, prefix: str) -> str | None:
    """Text between the parentheses opened right after the first match of the `prefix` regex."""
    match = re.search(prefix + r"\s*\(", text)
    if match is None:
        return None
    depth = 0
    for index in range(match.end() - 1, len(text)):
        if text[index] == "(":
            depth += 1
        elif text[index] == ")":
            depth -= 1
            if depth == 0:
                return text[match.end() : index]
    raise AssertionError(f"Unbalanced parentheses after {prefix!r}.")


def call_arguments(text: str, prefix: str) -> str:
    arguments = optional_call_arguments(text, prefix)
    assert arguments is not None, f"{prefix!r} not found in the generated model."
    return arguments


def class_body(model: str, class_name: str) -> str:
    match = re.search(rf"model {class_name}\b(.*?)end {class_name};", model, re.DOTALL)
    assert match is not None, f"Class {class_name} not found in the generated model."
    return match.group(1)


def scalar(arguments: str, key: str) -> float:
    match = re.search(rf"(?<![\w.]){re.escape(key)}\s*=\s*({NUMBER})", arguments)
    assert match is not None, f"{key} not found in {arguments!r}."
    return float(match.group(1))


def array(arguments: str, key: str) -> list[str]:
    match = re.search(rf"(?<![\w.]){re.escape(key)}\s*=\s*\{{([^{{}}]*)\}}", arguments)
    assert match is not None, f"{key} not found in {arguments!r}."
    return [item.strip() for item in match.group(1).split(",") if item.strip()]


def numbers(arguments: str, key: str) -> list[float]:
    return [float(value) for value in array(arguments, key)]


def without_spaces(text: str) -> str:
    return re.sub(r"\s+", "", text)


# --------------------------------------------------------------------------- #
# Generated models (built once per library: the conversion is expensive)
# --------------------------------------------------------------------------- #


def generate_model(library_name: str) -> str:
    network = convert_network("house_values", MODEL_PATH, library=Library.from_configuration(library_name))
    return network.model()


@pytest.fixture(scope="module")
def buildings_model() -> str:
    return generate_model("Buildings")


@pytest.fixture(scope="module")
def ideas_model() -> str:
    return generate_model("IDEAS")


def buildings_construction(model: str, construction_id: str) -> str:
    return call_arguments(model, rf"OpaqueConstructions\.Generic\s+{modelica_name(construction_id)}")


def buildings_glazing(model: str, glazing_id: str) -> str:
    return call_arguments(model, rf"GlazingSystems\.Generic\s+{modelica_name(glazing_id)}")


def buildings_zone(model: str, space_id: str) -> str:
    return call_arguments(model, rf"(?:MixedAir|MixedAirInf)\s+{modelica_name(space_id)}")


# --------------------------------------------------------------------------- #
# Buildings library: materials, constructions and glazing
# --------------------------------------------------------------------------- #


@pytest.mark.parametrize("construction_id", USED_CONSTRUCTIONS)
def test_buildings_construction_layers(buildings_model: str, construction_id: str) -> None:
    arguments = buildings_construction(buildings_model, construction_id)
    rendered = [
        tuple(map(float, layer))
        for layer in re.findall(
            rf"Solids\.Generic\(\s*x=({NUMBER}),\s*k=({NUMBER}),\s*c=({NUMBER}),\s*d=({NUMBER})\)", arguments
        )
    ]
    expected = [
        (
            layer["thickness"],
            MATERIALS[layer["material"]]["thermal_conductivity"],
            MATERIALS[layer["material"]]["specific_heat_capacity"],
            MATERIALS[layer["material"]]["density"],
        )
        for layer in CONSTRUCTIONS[construction_id]["layers"]
    ]
    assert scalar(arguments, "nLay") == len(expected)
    assert rendered == pytest.approx(expected)


@pytest.mark.xfail(
    strict=True,
    reason="Buildings template hard-codes absIR=0.9 and absSol=0.6 instead of the layer emissivities.",
)
@pytest.mark.parametrize("construction_id", USED_CONSTRUCTIONS)
def test_buildings_construction_surface_emissivities(buildings_model: str, construction_id: str) -> None:
    arguments = buildings_construction(buildings_model, construction_id)
    outside = MATERIALS[CONSTRUCTIONS[construction_id]["layers"][0]["material"]]
    room = MATERIALS[CONSTRUCTIONS[construction_id]["layers"][-1]["material"]]
    assert {key: scalar(arguments, key) for key in ("absIR_a", "absIR_b", "absSol_a", "absSol_b")} == pytest.approx(
        {
            "absIR_a": outside["longwave_emissivity"],
            "absIR_b": room["longwave_emissivity"],
            "absSol_a": outside["shortwave_emissivity"],
            "absSol_b": room["shortwave_emissivity"],
        }
    )


@pytest.mark.parametrize("glazing_id", sorted(GLAZINGS))
def test_buildings_glazing_layers(buildings_model: str, glazing_id: str) -> None:
    arguments = buildings_glazing(buildings_model, glazing_id)
    layers = GLAZINGS[glazing_id]["layers"]
    glass_pattern = rf"Glasses\.Generic\(\s*x=({NUMBER}),\s*k=({NUMBER})"
    glasses = [tuple(map(float, glass)) for glass in re.findall(glass_pattern, arguments)]
    gases = [float(x) for x in re.findall(rf"Gases\.\w+\(x=({NUMBER})\)", arguments)]
    assert glasses == pytest.approx(
        [
            (layer["thickness"], MATERIALS[layer["glass"]]["thermal_conductivity"])
            for layer in layers
            if "glass" in layer
        ]
    )
    assert gases == pytest.approx([layer["thickness"] for layer in layers if "gas" in layer])


@pytest.mark.xfail(
    strict=True,
    reason="Buildings glazing template always renders Gases.Air: the gas type and properties are lost.",
)
@pytest.mark.parametrize("glazing_id", sorted(GLAZINGS))
def test_buildings_glazing_gas_type(buildings_model: str, glazing_id: str) -> None:
    arguments = buildings_glazing(buildings_model, glazing_id)
    rendered = re.findall(r"Gases\.(\w+)\(", arguments)
    # Buildings ships Air, Argon, Krypton and Xenon records; the YAML ids name the gas.
    expected = [layer["gas"].split(":")[0].capitalize() for layer in GLAZINGS[glazing_id]["layers"] if "gas" in layer]
    assert rendered == expected


# --------------------------------------------------------------------------- #
# Buildings library: spaces and their boundaries
# --------------------------------------------------------------------------- #


def _rendered_boundaries(zone_arguments: str, block: str) -> list[tuple[str, float, float, str]]:
    arguments = optional_call_arguments(zone_arguments, rf"(?<![\w.]){block}")
    if arguments is None:
        return []
    return list(
        zip(
            array(arguments, "layers"),
            [round(value, 6) for value in numbers(arguments, "A")],
            [round(value, 6) for value in numbers(arguments, "azi")],
            array(arguments, "til"),
            strict=True,
        )
    )


def _internal_walls(space_id: str) -> list[tuple[float, str]]:
    return [
        (round(wall["surface"], 6), BUILDINGS_TILTS[wall.get(f"{side}_tilt", "wall")])
        for wall in DATA["internal_walls"]
        for side in ("space_1", "space_2")
        if wall[side] == space_id
    ]


@pytest.mark.parametrize("space", SPACES, ids=[space["id"] for space in SPACES])
def test_buildings_space_parameters(buildings_model: str, space: dict[str, Any]) -> None:
    arguments = buildings_zone(buildings_model, space["id"])
    parameters = space["parameters"]
    expected = {
        "hRoo": parameters["average_room_height"],
        "AFlo": parameters["floor_area"],
        "T_start": parameters["temperature_initial"],
        "mSenFac": parameters["sensible_thermal_mass_scaling_factor"],
    }
    if "ach" in parameters:
        expected["ACH"] = parameters["ach"]
    assert {key: scalar(arguments, key) for key in expected} == pytest.approx(expected)
    zone_type = "Trano.ThermalZones.MixedAirInf" if space.get("variant") == "infiltration" else "MixedAir"
    assert re.search(rf"{re.escape(zone_type)}\s+{modelica_name(space['id'])}\s*\(", buildings_model)


@pytest.mark.parametrize("space", SPACES, ids=[space["id"] for space in SPACES])
def test_buildings_space_external_walls(buildings_model: str, space: dict[str, Any]) -> None:
    arguments = buildings_zone(buildings_model, space["id"])
    # Walls sharing their azimuth with a window are rendered in datConExtWin, the others in datConExt.
    rendered = _rendered_boundaries(arguments, "datConExt") + _rendered_boundaries(arguments, "datConExtWin")
    expected = [
        (
            modelica_name(wall["construction"]),
            round(wall["surface"], 6),
            round(wall["azimuth"], 6),
            BUILDINGS_TILTS[wall["tilt"]],
        )
        for wall in space["external_boundaries"].get("external_walls", [])
    ]
    assert Counter(rendered) == Counter(expected)


@pytest.mark.parametrize("space", SPACES, ids=[space["id"] for space in SPACES])
def test_buildings_space_windows(buildings_model: str, space: dict[str, Any]) -> None:
    arguments = buildings_zone(buildings_model, space["id"])
    windows = space["external_boundaries"].get("windows", [])
    window_arguments = optional_call_arguments(arguments, r"(?<![\w.])datConExtWin")
    if window_arguments is None:
        assert not windows
        return
    rendered = [
        (glazing, round(width * height, 6), round(azimuth, 6))
        for glazing, width, height, azimuth in zip(
            array(window_arguments, "glaSys"),
            numbers(window_arguments, "wWin"),
            numbers(window_arguments, "hWin"),
            numbers(window_arguments, "azi"),
            strict=True,
        )
    ]
    expected = [
        (modelica_name(window["construction"]), round(window["surface"], 6), round(window["azimuth"], 6))
        for window in windows
    ]
    assert Counter(rendered) == Counter(expected)


@pytest.mark.parametrize("space", SPACES, ids=[space["id"] for space in SPACES])
def test_buildings_space_floors(buildings_model: str, space: dict[str, Any]) -> None:
    arguments = buildings_zone(buildings_model, space["id"])
    rendered = [(layers, area, tilt) for layers, area, _, tilt in _rendered_boundaries(arguments, "datConBou")]
    expected = [
        (modelica_name(floor["construction"]), round(floor["surface"], 6), BUILDINGS_TILTS["floor"])
        for floor in space["external_boundaries"].get("floor_on_grounds", [])
    ]
    assert Counter(rendered) == Counter(expected)


@pytest.mark.parametrize("space", SPACES, ids=[space["id"] for space in SPACES])
def test_buildings_space_internal_surfaces(buildings_model: str, space: dict[str, Any]) -> None:
    arguments = buildings_zone(buildings_model, space["id"])
    surfaces = optional_call_arguments(arguments, r"(?<![\w.])surBou")
    rendered = (
        []
        if surfaces is None
        else list(zip([round(a, 6) for a in numbers(surfaces, "A")], array(surfaces, "til"), strict=True))
    )
    assert Counter(rendered) == Counter(_internal_walls(space["id"]))


@pytest.mark.parametrize(
    "internal_wall",
    DATA["internal_walls"],
    ids=[f"{wall['space_1']}-{wall['space_2']}" for wall in DATA["internal_walls"]],
)
def test_buildings_internal_wall(buildings_model: str, internal_wall: dict[str, Any]) -> None:
    name = "_".join(
        [
            "internal",
            modelica_name(internal_wall["space_1"]),
            modelica_name(internal_wall["space_2"]),
            internal_wall["construction"].split(":")[0].lower(),
        ]
    )
    arguments = call_arguments(buildings_model, rf"MultiLayer\s+{name}")
    assert scalar(arguments, "A") == pytest.approx(internal_wall["surface"])
    assert re.search(rf"layers\s*=\s*{modelica_name(internal_wall['construction'])}\b", arguments)


# --------------------------------------------------------------------------- #
# Buildings library: occupancy, emission, systems and weather
# --------------------------------------------------------------------------- #


@pytest.mark.parametrize(("index", "space"), list(enumerate(SPACES, start=1)), ids=[space["id"] for space in SPACES])
def test_buildings_occupancy(buildings_model: str, index: int, space: dict[str, Any]) -> None:
    occupancy = space["occupancy"]
    parameters = occupancy["parameters"]
    arguments = call_arguments(buildings_model, rf"(?<![\w.])occupancy_{index}")
    if occupancy.get("variant") == "co2":
        assert scalar(arguments, "ACH") == pytest.approx(parameters["ach"])
        assert scalar(arguments, "AFlo") == pytest.approx(parameters["floor_area"])
        body = class_body(buildings_model, f"OccupancyOccupancy_{index}")
        for data in parameters["data"]:
            assert f"connect(dataBus.{data['variable']}, {data['component']}.u)" in body
    else:
        compact = without_spaces(arguments)
        assert f"occupancy={without_spaces(parameters['occupancy'])}" in compact
        assert f"gain={without_spaces(parameters['gain'])}" in compact


@pytest.mark.parametrize(("space_id", "radiator"), EMISSIONS, ids=[radiator["id"] for _, radiator in EMISSIONS])
def test_buildings_radiator_and_control(buildings_model: str, space_id: str, radiator: dict[str, Any]) -> None:
    radiator_name = modelica_name(radiator["id"])
    arguments = call_arguments(buildings_model, rf"(?<![\w.]){radiator_name}")
    power = radiator["parameters"]["nominal_heating_power_positive_for_heating"]
    assert scalar(arguments, "power") == pytest.approx(power)

    control = radiator["control"]["emission_control"]
    control_name = modelica_name(control["id"])
    control_arguments = call_arguments(buildings_model, rf"(?<![\w.]){control_name}")
    assert scalar(control_arguments, "k") == pytest.approx(control["parameters"]["controller_gain"])

    body = class_body(buildings_model, f"EmissionControl{control['variant']}{control_name.capitalize()}")
    for data in control["parameters"]["data"]:
        assert f"connect(dataBus.{data['variable']}, {data['component']}.u)" in body
    # The controller reads its own zone temperature and drives its own radiator.
    assert f"dataBus.TZon{modelica_name(space_id).capitalize()}," in body
    assert f"dataBus.yHea{radiator_name.capitalize()}," in body


def test_buildings_power_sensor(buildings_model: str) -> None:
    (sensor,) = [system["power_sensor"] for system in DATA["systems"] if "power_sensor" in system]
    name = modelica_name(sensor["id"])
    body = class_body(buildings_model, f"PowerSensor{name.capitalize()}")
    assert scalar(body, "n") == len(sensor["inlets"])
    for inlet in sensor["inlets"]:
        assert f"dataBus.power{modelica_name(inlet).capitalize()}," in body


def test_buildings_air_handling_unit(buildings_model: str) -> None:
    (ahu,) = [system["air_handling_unit"] for system in DATA["systems"] if "air_handling_unit" in system]
    arguments = call_arguments(buildings_model, rf"SystemD\w+\s+{modelica_name(ahu['id'])}")
    parameters = ahu["parameters"]
    assert {key: scalar(arguments, key) for key in ("m_flow_nominal", "dp_nominal", "eps")} == pytest.approx(
        {
            "m_flow_nominal": parameters["m_flow_nominal"],
            "dp_nominal": parameters["dp_nominal"],
            "eps": parameters["heat_exchanger_effectiveness"],
        }
    )
    for inlet in ahu["inlets"]:
        assert f"connect({modelica_name(inlet)}.port_b,{modelica_name(ahu['id'])}.port_a)" in buildings_model
    for outlet in ahu["outlets"]:
        assert f"connect({modelica_name(ahu['id'])}.port_b,{modelica_name(outlet)}.port_a)" in buildings_model


def test_buildings_weather(buildings_model: str) -> None:
    arguments = call_arguments(buildings_model, r"(?<![\w.])weather")
    assert DATA["weather"]["parameters"]["path"] in arguments


# --------------------------------------------------------------------------- #
# IDEAS library: materials, constructions and glazing records
# --------------------------------------------------------------------------- #


def _ideas_material(model: str, material_id: str) -> str:
    return call_arguments(
        model, rf"record\s+{modelica_name(material_id)}\s*=\s*IDEAS\.Buildings\.Data\.Interfaces\.Material"
    )


def _ideas_layers(arguments: str) -> list[tuple[str, float]]:
    return [(name, float(d)) for name, d in re.findall(rf"Materials\.(\w+)\s*\(d=({NUMBER})\)", arguments)]


@pytest.mark.parametrize("material_id", USED_MATERIALS)
def test_ideas_material(ideas_model: str, material_id: str) -> None:
    arguments = _ideas_material(ideas_model, material_id)
    material = MATERIALS[material_id]
    assert {key: scalar(arguments, key) for key in ("k", "c", "rho")} == pytest.approx(
        {
            "k": material["thermal_conductivity"],
            "c": material["specific_heat_capacity"],
            "rho": material["density"],
        }
    )


@pytest.mark.xfail(strict=True, reason="IDEAS material template hard-codes epsLw=0.88 and epsSw=0.55.")
@pytest.mark.parametrize("material_id", USED_MATERIALS)
def test_ideas_material_emissivities(ideas_model: str, material_id: str) -> None:
    arguments = _ideas_material(ideas_model, material_id)
    material = MATERIALS[material_id]
    assert {key: scalar(arguments, key) for key in ("epsLw", "epsSw")} == pytest.approx(
        {"epsLw": material["longwave_emissivity"], "epsSw": material["shortwave_emissivity"]}
    )


@pytest.mark.parametrize("construction_id", USED_CONSTRUCTIONS)
def test_ideas_construction(ideas_model: str, construction_id: str) -> None:
    name = modelica_name(construction_id)
    arguments = call_arguments(
        ideas_model, rf"record\s+{name}\s+\"{name}\"\s+extends\s+IDEAS\.Buildings\.Data\.Interfaces\.Construction"
    )
    expected = [
        (modelica_name(layer["material"]), layer["thickness"]) for layer in CONSTRUCTIONS[construction_id]["layers"]
    ]
    assert _ideas_layers(arguments) == expected


@pytest.mark.parametrize("glazing_id", sorted(GLAZINGS))
def test_ideas_glazing(ideas_model: str, glazing_id: str) -> None:
    arguments = call_arguments(
        ideas_model, rf"record\s+{modelica_name(glazing_id)}\s*=\s*IDEAS\.Buildings\.Data\.Interfaces\.Glazing"
    )
    layers = GLAZINGS[glazing_id]["layers"]
    assert scalar(arguments, "nLay") == len(layers)
    assert _ideas_layers(arguments) == [
        (modelica_name(layer.get("glass") or layer["gas"]), layer["thickness"]) for layer in layers
    ]
