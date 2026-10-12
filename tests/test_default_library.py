"""Physical checks of the default library of materials, glazings and constructions (`trano/data/default.yaml`).

The expected values are those of the comments of the library, which document each construction: the U-value of
the layers with the surface resistances of a wall (EN ISO 6946) and, for the glazings, the center-of-glass U-value
(EN 673) and g-value (EN 410).
"""

from functools import cache
from pathlib import Path

import pytest
import yaml

from trano.data.include import Loader
from trano.data_models.conversion import _build_constructions, _build_materials, _merge_default, load_and_enrich_model
from trano.elements.construction import Construction, Glass, GlassLayer
from trano.elements.glazing import GlazingProperties

DEFAULT_LIBRARY = Path(__file__).parents[1] / "trano" / "data" / "default.yaml"
# a model that includes the default library, loaded through the pipeline to get the schema defaults
MODEL = Path(__file__).parent / "models" / "single_zone_hydronic.yaml"
SURFACE_RESISTANCES = 0.13 + 0.04  # [m2.K/W] inside and outside, horizontal heat flow (EN ISO 6946)
LOW_EMISSIVITY = 0.5

# U-value [W/(m2.K)] of each opaque construction
OPAQUE = {
    "FLOORONGROUND:001": 0.31,
    "FLOORONGROUNDUNINSULATED:001": 3.0,
    "INSULATEDFLOORHEATING:001": 0.32,
    "CAVITYWALL:001": 0.28,
    "CAVITYWALLPARTIALFILL:001": 0.26,
    "CAVITYWALLEMPTY:001": 1.0,
    "SOLIDBRICKWALL:001": 1.8,
    "ETICSWALL:001": 0.19,
    "AACWALL:001": 0.28,
    "TIMBERFRAMEWALL:001": 0.16,
    "PITCHEDROOF:001": 0.15,
    "FLATROOF:001": 0.17,
    "UNINSULATEDROOF:001": 2.6,
}
# U-value [W/(m2.K)] and g-value [1] of each glazing
GLAZINGS = {
    "INS2AR2020:001": (1.1, 0.63),
    "INS3AR2020:001": (0.58, 0.47),
    "INS2KR2020:001": (0.96, 0.63),
    "INS3KR2020:001": (0.45, 0.47),
    "INS2AIR2000:001": (1.4, 0.63),
    "EPCDOUBLE:001": (2.9, 0.76),
    "EPCSINGLE:001": (5.8, 0.86),
}


@cache
def constructions() -> dict[str, Construction | Glass]:
    data = _merge_default(load_and_enrich_model(MODEL).data)
    return _build_constructions(data, _build_materials(data))


def _construction(id_: str) -> Construction | Glass:
    return constructions()[id_]


def _library() -> dict[str, list[dict]]:
    return yaml.load(DEFAULT_LIBRARY.read_text(), Loader=Loader)  # noqa: S506


def _glazing(glass: Glass) -> GlazingProperties:
    panes = [layer for layer in glass.layers if isinstance(layer, GlassLayer)]
    gaps = [layer for layer in glass.layers if not isinstance(layer, GlassLayer)]
    return GlazingProperties.from_layers(panes, gaps)


def test_every_construction_is_documented() -> None:
    library = _library()
    assert {item["id"] for item in library["glazings"]} == set(GLAZINGS)
    walls_and_floors = {"CONCRETESLAB:001", "TABS:001", "TIMBERFLOOR:001"}
    partitions = {"INTERNALWALLMASONRY:001", "INTERNALWALLLIGHT:001"}
    assert {item["id"] for item in library["constructions"]} == set(OPAQUE) | walls_and_floors | partitions


@pytest.mark.parametrize(("id_", "u_value"), OPAQUE.items())
def test_opaque_construction_u_value(id_: str, u_value: float) -> None:
    construction = _construction(id_)
    assert 1 / (construction.total_thermal_resistance + SURFACE_RESISTANCES) == pytest.approx(u_value, rel=0.05)


@pytest.mark.parametrize(("id_", "values"), GLAZINGS.items())
def test_glazing_u_and_g_values(id_: str, values: tuple[float, float]) -> None:
    glazing = _construction(id_)
    assert isinstance(glazing, Glass)
    properties = _glazing(glazing)
    assert (properties.u_value, properties.g_value) == pytest.approx(values, rel=0.05)


def test_opaque_constructions_take_no_gas() -> None:
    """A gas conducts as still air in an opaque construction: cavities take the AIRCAVITY materials."""
    gases = {gas["id"] for gas in _library()["gas"]}
    for construction in _library()["constructions"]:
        assert not {layer["material"] for layer in construction["layers"]} & gases, construction["id"]


@pytest.mark.parametrize("id_", GLAZINGS)
def test_low_e_coatings_face_a_gas_cavity(id_: str) -> None:
    """A coating facing the outside air or the room would not reduce the radiation across a cavity."""
    layers = _construction(id_).layers
    for index, layer in enumerate(layers):
        if not isinstance(layer, GlassLayer):
            continue
        if layer.material.infrared_absorptivity_outside_facing < LOW_EMISSIVITY:
            assert index > 0, f"{layer.material.name} has its coating towards the outside air"
        if layer.material.infrared_absorptivity_room_facing < LOW_EMISSIVITY:
            assert index < len(layers) - 1, f"{layer.material.name} has its coating towards the room"


def test_panes_absorb_part_of_the_sun() -> None:
    for glass in _library()["glass_material"]:
        reflectance = max(glass["solar_reflectance_outside_facing"][0], glass["solar_reflectance_room_facing"][0])
        absorptance = 1 - glass["solar_transmittance"][0] - reflectance
        assert 0 < absorptance < 0.5, glass["id"]
