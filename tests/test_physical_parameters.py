"""The physical parameters of the zone, the occupancy, the weather and the envelope reach every library."""

import re
from pathlib import Path

import pytest
import yaml

from tests.fixtures.simple_space_1 import simple_space_1_fixture
from trano.data_models.conversion import convert_network
from trano.elements import FloorOnGround, param_from_config
from trano.elements.library.library import Library
from trano.elements.system import Occupancy, Weather
from trano.exceptions import InvalidBuildingStructureError
from trano.topology import Network

SpaceParameters = param_from_config("Space")
OccupancyParameters = param_from_config("Occupancy")
WeatherParameters = param_from_config("Weather")
assert SpaceParameters and OccupancyParameters and WeatherParameters
MODELS = Path(__file__).parent.joinpath("models")


def zone_model(
    library: str,
    variant: str = "infiltration",
    occupancy: dict[str, object] | None = None,
    **parameters: object,
) -> str:
    space = simple_space_1_fixture()
    space.variant = variant
    space.parameters = SpaceParameters(floor_area=48, average_room_height=2.7, **parameters)
    if occupancy is not None:
        space.occupancy = Occupancy(name="occupancy", parameters=OccupancyParameters(**occupancy))
        space.occupancy.space_name = space.name
    network = Network(name=f"{library}_parameters", library=Library.from_configuration(library))
    network.add_boiler_plate_spaces([space])
    return re.sub(r"\s+", " ", network.model())


def yaml_model(tmp_path: Path, library: str, **lines_after: str) -> str:
    """The single-zone model with lines inserted after the given ones (its default data is an include tag)."""
    text = MODELS.joinpath("single_zone_hydronic_occupancy_from_data.yaml").read_text()
    for anchor, line in lines_after.items():
        anchor_line = next(candidate for candidate in text.splitlines() if candidate.strip() == anchor)
        text = text.replace(
            anchor_line + "\n",
            anchor_line + "\n" + " " * (len(anchor_line) - len(anchor_line.lstrip())) + line + "\n",
            1,
        )
    model = tmp_path.joinpath("house.yaml")
    model.write_text(text)
    return re.sub(r"\s+", " ", convert_network("house", model, library=Library.from_configuration(library)).model())


# --------------------------------------------------------------------------- #
# Zone
# --------------------------------------------------------------------------- #


def test_the_airtightness_gives_the_infiltration_when_the_air_change_rate_is_absent() -> None:
    assert SpaceParameters(n50=3).ach == pytest.approx(0.15)
    assert SpaceParameters(n50=3, n50_to_ach=25).ach == pytest.approx(0.12)
    assert SpaceParameters(n50=3, ach=0.5).ach == 0.5
    assert SpaceParameters().ach is None


def test_the_airtightness_reaches_each_library() -> None:
    assert "ACH=0.15," in zone_model("Buildings", n50=3)
    assert re.search(r"Zone space_1\([^;]*n50=3\.0,", zone_model("IDEAS", n50=3))
    assert re.search(r"n50toAch=25\.0,[^;]*n50=3\.0,", zone_model("IDEAS", n50=3, n50_to_ach=25))
    assert "baseACH=0.15," in zone_model("reduced_order", n50=3)
    assert "airRat=0.15," in zone_model("iso_13790", n50=3)


def test_fixed_convection_coefficients() -> None:
    buildings = zone_model("Buildings", interior_convection_coefficient=3.5, exterior_convection_coefficient=12)
    assert (
        "hIntFixed=3.5," in buildings and "intConMod=Buildings.HeatTransfer.Types.InteriorConvection.Fixed" in buildings
    )
    assert (
        "hExtFixed=12.0," in buildings
        and "extConMod=Buildings.HeatTransfer.Types.ExteriorConvection.Fixed" in buildings
    )
    assert "intConMod=Buildings.HeatTransfer.Types.InteriorConvection.Fixed" not in zone_model("Buildings")

    reduced = zone_model("reduced_order", interior_convection_coefficient=3.5, exterior_convection_coefficient=12)
    assert "hConExt=3.5," in reduced and "hConFloor=3.5," in reduced and "hConWallOut=12.0," in reduced
    assert "hConExt=2.7," in zone_model("reduced_order") and "hConWallOut=20.0," in zone_model("reduced_order")

    assert "hInt=3.5," in zone_model("iso_13790", interior_convection_coefficient=3.5)


def test_the_iso_13790_zone_takes_its_mass_class_ground_factor_and_shading() -> None:
    model = zone_model(
        "iso_13790", thermal_mass_class="heavy", ground_heat_transfer_factor=0.6, shading_reduction_factor=0.8
    )

    assert "Data.Heavy buiMas" in model and "b=0.6," in model and "shaRedFac=0.8," in model
    assert "Data.Light buiMas" in zone_model("iso_13790")
    with pytest.raises(InvalidBuildingStructureError, match="light, medium or heavy"):
        zone_model("iso_13790", thermal_mass_class="feather")


def test_the_reduced_order_zone_takes_the_sunblind_settings() -> None:
    model = zone_model("reduced_order", shading_reduction_factor=0.8, sunblind_irradiance_threshold=150)

    assert re.search(r"shadingFactor = \{ ?0\.8(, 0\.8)* ?\}", model)
    assert re.search(r"maxIrr = \{ ?150\.0(, 150\.0)* ?\}", model)
    default = zone_model("reduced_order")
    assert re.search(r"shadingFactor = \{ ?0\.7(, 0\.7)* ?\}", default)
    assert re.search(r"maxIrr = \{ ?100(, 100)* ?\}", default)


# --------------------------------------------------------------------------- #
# Occupancy
# --------------------------------------------------------------------------- #


def test_the_gains_per_person_give_the_gain_matrix() -> None:
    parameters = OccupancyParameters(sensible_heat_per_person=100, latent_heat_per_person=40, radiant_fraction=0.6)
    assert parameters.gain == "[60; 40; 40]"
    assert OccupancyParameters(sensible_heat_per_person=100).gain == "[50; 50; 0]"
    assert OccupancyParameters(gain="[1; 2; 3]", sensible_heat_per_person=100).gain == "[1; 2; 3]"
    assert OccupancyParameters().gain == "[35; 70; 30]"


def test_the_occupant_density_is_a_short_name_of_the_heat_gain_if_occupied() -> None:
    assert OccupancyParameters(occupant_density="0.05").heat_gain_if_occupied == "0.05"
    with pytest.raises(ValueError, match="same parameter"):
        OccupancyParameters(occupant_density="0.05", heat_gain_if_occupied="0.1")


def test_lighting_and_equipment_gains_reach_the_wrappers_and_the_reduced_order_zone() -> None:
    occupancy = {"lighting_power_density": 5, "lighting_radiant_fraction": 0.3, "equipment_power_density": 8}

    buildings = zone_model("Buildings", occupancy=occupancy)
    assert (
        "lightingPower=5.0" in buildings
        and "lightingRadiantFraction=0.3" in buildings
        and "equipmentPower=8.0" in buildings
    )
    assert "y = gai2.y + {lightingPower*lightingRadiantFraction" in buildings

    reduced = zone_model("reduced_order", occupancy=occupancy)
    assert "lightingPowerSpecific=5.0," in reduced and "ratioConvectiveHeatLighting=0.7," in reduced
    assert "internalGainsMachinesSpecific=8.0," in reduced and "ratioConvectiveHeatMachines=0.6," in reduced


def test_the_co2_variant_takes_its_generation_and_outdoor_concentration() -> None:
    space = simple_space_1_fixture()
    space.occupancy = Occupancy(
        name="occupancy",
        variant="co2",
        parameters=OccupancyParameters(co2_generation_per_person=4e-6, outdoor_co2_concentration=400),
    )
    space.occupancy.space_name = space.name
    network = Network(name="co2", library=Library.from_configuration("Buildings"))
    network.add_boiler_plate_spaces([space])
    model = re.sub(r"\s+", " ", network.model())

    assert "gCO2=4e-06" in model and "ppmOut=400.0" in model


# --------------------------------------------------------------------------- #
# Weather and envelope
# --------------------------------------------------------------------------- #


def test_the_ideas_simulation_manager_takes_the_site_parameters() -> None:
    space = simple_space_1_fixture()
    network = Network(name="site", library=Library.from_configuration("IDEAS"))
    network.add_boiler_plate_spaces([space])
    weather = next(node for node in network.graph.nodes if isinstance(node, Weather))
    weather.parameters = WeatherParameters(
        path="weather.mos", outdoor_co2_concentration=450, building_height=12, default_n50=4
    )
    model = re.sub(r"\s+", " ", network.model())

    assert re.search(r"SimInfoManager sim\([^;]*ppmCO2=450\.0[^;]*H=12\.0[^;]*n50=4\.0", model)


def test_the_atmospheric_pressure_of_the_reader() -> None:
    space = simple_space_1_fixture()
    network = Network(name="site", library=Library.from_configuration("Buildings"))
    network.add_boiler_plate_spaces([space])
    weather = next(node for node in network.graph.nodes if isinstance(node, Weather))
    weather.parameters = WeatherParameters(path="weather.mos", atmospheric_pressure=90000)

    assert "pAtm=90000.0" in re.sub(r"\s+", " ", network.model())


def test_the_ground_temperature_and_perimeter_of_a_floor_come_from_the_yaml(tmp_path: Path) -> None:
    floor = {"construction: CONCRETESLAB:001": "ground_temperature: 285.15"}
    assert "TSoil=285.15," in yaml_model(tmp_path, "reduced_order", **floor)
    space = simple_space_1_fixture()
    next(
        boundary for boundary in space.external_boundaries if isinstance(boundary, FloorOnGround)
    ).ground_temperature = 285.15
    network = Network(name="ground", library=Library.from_configuration("Buildings"))
    network.add_boiler_plate_spaces([space])
    assert "FixedTemperature floor_2(T=285.15)" in network.model()

    floor = {"construction: CONCRETESLAB:001": "perimeter: 30"}
    assert "PWall=30.0, A=50.0" in yaml_model(tmp_path, "IDEAS", **floor)
    assert "PWall" not in yaml_model(tmp_path, "IDEAS")


def test_the_glazing_u_and_g_values_can_be_given(tmp_path: Path) -> None:
    """The BESTEST case 600 describes its glazing in full: its U and g values are then computed, unless given."""
    data = yaml.safe_load(Path("validation/bestest/cases/case_600.yaml").read_text())
    data["glazings"][0] |= {"u_value": 1.1, "g_value": 0.5}
    model = tmp_path.joinpath("case_600.yaml")
    model.write_text(yaml.safe_dump(data, sort_keys=False))

    models = {
        library: re.sub(
            r"\s+", " ", convert_network("case_600", model, library=Library.from_configuration(library)).model()
        )
        for library in ("iso_13790", "reduced_order", "IDEAS")
    }

    assert "UWin=1.1," in models["iso_13790"] and "gFac=0.5)" in models["iso_13790"]
    assert "UWin=1.1," in models["reduced_order"] and "gWin=0.5," in models["reduced_order"]
    assert "U_value=1.1," in models["IDEAS"] and "g_value=0.5" in models["IDEAS"]
