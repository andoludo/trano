"""Occupancy must reach the zone of every library: as heat flows, occupant densities or people gains."""

import re
from pathlib import Path

import pytest
import yaml

from tests.fixtures.three_spaces import three_spaces
from tests.golden import remove_trano_package
from trano.data_models.conversion import convert_network
from trano.elements import param_from_config
from trano.elements.library.library import Library
from trano.elements.space import Space
from trano.elements.system import Occupancy, evaluate_number
from trano.exceptions import InvalidBuildingStructureError
from trano.topology import Network

OccupancyParameters = param_from_config("Occupancy")


def generated_model(library: str, occupancy: bool = True) -> tuple[str, str]:
    """Model without the Trano package, and the Trano package itself."""
    network = Network(name=f"{library}_occupancy", library=Library.from_configuration(library))
    network.add_boiler_plate_spaces(three_spaces(occupancy=occupancy))
    model = network.model()
    return remove_trano_package(model), model


def zone_declaration(model: str) -> str:
    start = model.index(" space_1(")
    return model[start : model.index("annotation", start)]


def test_default_gains_per_person() -> None:
    gains = Occupancy(name="occupancy").gains_per_person

    assert (gains.radiant, gains.convective, gains.latent) == (35, 70, 30)
    assert gains.sensible == 105
    assert gains.radiant_fraction == pytest.approx(1 / 3)


def test_gains_per_person_accept_numeric_expressions() -> None:
    gains = Occupancy(name="occupancy", parameters=OccupancyParameters(gain="[60*0.4; 60*0.6; 40]")).gains_per_person

    assert (gains.radiant, gains.convective, gains.latent) == pytest.approx((24, 36, 40))


@pytest.mark.parametrize("gain", ["[35; 70]", "35; 70; 30", "[35; 70; latent]", "[__import__('os'); 1; 2]"])
def test_gains_per_person_reject_anything_but_three_numbers(gain: str) -> None:
    occupancy = Occupancy(name="occupancy", parameters=OccupancyParameters(gain=gain))

    with pytest.raises(InvalidBuildingStructureError, match="three numbers"):
        _ = occupancy.gains_per_person


def test_numeric_expressions_are_evaluated_without_eval() -> None:
    assert evaluate_number("1/6/4") == pytest.approx(1 / 24)
    assert evaluate_number("-2 + 3*4") == 10
    with pytest.raises(ValueError, match="Unsupported expression"):
        evaluate_number("abs(-1)")


def test_buildings_zone_receives_the_occupancy_heat_flows() -> None:
    model, _ = generated_model("Buildings")

    assert re.search(r"connect\(space_1\.qGai_flow,\s*occupancy_0\.y\)", model)


def test_ideas_zone_counts_its_occupants_from_the_occupant_density() -> None:
    model, _ = generated_model("IDEAS")
    declaration = zone_declaration(model)

    assert "redeclare IDEAS.Buildings.Components.Occupants.AreaWeightedInput occNum" in declaration
    assert re.search(r"QsenPp=105\.0,\s*QlatPp=30\.0,\s*radFra=0\.333333\)", declaration)
    assert re.search(r"connect\(space_1\.yOcc,\s*occupancy_0\.occupantDensity\)", model)


def test_ideas_zone_without_occupancy_keeps_the_library_default() -> None:
    model, _ = generated_model("IDEAS", occupancy=False)

    assert "occNum" not in zone_declaration(model)
    assert "yOcc" not in model


def test_reduced_order_zone_scales_the_people_gains_itself() -> None:
    model, _ = generated_model("reduced_order")
    declaration = zone_declaration(model)

    # internalGainsMode 2: persons = intGains[1] * specificPeople * AZone, sensible heat only.
    assert "internalGainsMode=2" in declaration
    assert "specificPeople=1/6/4" in declaration
    assert "fixedHeatFlowRatePersons=105.0" in declaration
    assert "ratioConvectiveHeatPeople=0.666667" in declaration
    assert "internalGainsMachinesSpecific=0," in declaration
    assert "lightingPowerSpecific=0," in declaration
    assert re.search(r"connect\(space_1\.intGains,\s*occupancy_0\.relativeGains\)", model)


def test_reduced_order_zone_without_occupancy_has_no_people() -> None:
    model, _ = generated_model("reduced_order", occupancy=False)

    assert "specificPeople=0," in zone_declaration(model)


def test_iso_13790_zone_scales_the_gains_per_floor_area() -> None:
    model, full_model = generated_model("iso_13790")

    assert "Trano.ThermalZones.ISO13790ZoneHVAC space_1(" in model
    assert re.search(r"connect\(space_1\.intSenGaiFlo,\s*occupancy_0\.sensibleGains\)", model)
    assert re.search(r"connect\(space_1\.intLatGaiFlo,\s*occupancy_0\.latentGains\)", model)
    assert "final intSenGai=intSenGaiFlo*AFlo" in full_model
    assert "final intLatGai=intLatGaiFlo*AFlo" in full_model


def test_a_none_occupancy_variant_leaves_the_space_without_occupancy(tmp_path: Path) -> None:
    """``occupancy: {variant: none}`` is explicit; an empty ``occupancy:`` gets the default schedule."""
    data = yaml.safe_load(Path(__file__).parent.joinpath("models", "house.yaml").read_text())
    data["spaces"][0]["occupancy"] = {"variant": "none"}
    model = tmp_path.joinpath("house.yaml")
    model.write_text(yaml.safe_dump(data, sort_keys=False))
    network = convert_network("house", model, library=Library.from_configuration("IDEAS"))
    spaces = sorted((node for node in network.graph.nodes if isinstance(node, Space)), key=lambda s: s.name)

    assert spaces[0].occupancy is None
    assert all(space.occupancy is not None for space in spaces[1:])
    # A Buildings zone needs its gain input connected: the space gets an occupancy with no gains.
    network = convert_network("house", model, library=Library.from_configuration("Buildings"))
    spaces = sorted((node for node in network.graph.nodes if isinstance(node, Space)), key=lambda s: s.name)
    assert spaces[0].occupancy is not None and spaces[0].occupancy.parameters.gain == "[0; 0; 0]"  # type: ignore[union-attr]
    assert "no_occupancy_" in spaces[0].occupancy.name
