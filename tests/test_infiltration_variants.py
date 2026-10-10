"""The infiltration variant feeds the YAML air change rate to every library's zone."""

import re

from tests.fixtures.simple_space_1 import simple_space_1_fixture
from tests.golden import remove_trano_package
from trano.elements import param_from_config
from trano.elements.library.library import Library
from trano.topology import Network

SpaceParameters = param_from_config("Space")
assert SpaceParameters is not None


def infiltration_model(library: str, **parameters: float | str) -> str:
    space = simple_space_1_fixture()
    space.variant = "infiltration"
    space.parameters = SpaceParameters(floor_area=48, average_room_height=2.7, ach=0.414, **parameters)
    network = Network(name=f"{library}_infiltration", library=Library.from_configuration(library))
    network.add_boiler_plate_spaces([space])
    return re.sub(r"\s+", " ", remove_trano_package(network.model()))


def test_reduced_order_zone_uses_a_constant_dry_air_change_rate() -> None:
    model = infiltration_model("reduced_order")

    assert "useConstantACHrate=true, baseACH=0.414," in model and "use_moisture_balance=false" in model


def test_iso_13790_zone_takes_the_air_change_rate() -> None:
    assert "airRat=0.414," in infiltration_model("iso_13790")


def test_ideas_zone_with_a_ventilation_schedule_brings_in_outdoor_air() -> None:
    model = infiltration_model("IDEAS", ventilation_schedule="[0, 0.4; 25200, 0]")

    assert re.search(
        r"Trano\.ThermalZones\.ZoneScheduledVentilation space_1\( nPortsExt = \d+, "
        r"ventilationSchedule=\[0, 0\.4; 25200, 0\], .*?n50=0\.414\*space_1\.n50toAch",
        model,
    )


def test_ideas_zone_without_schedule_stays_a_plain_zone() -> None:
    model = infiltration_model("IDEAS")

    assert "IDEAS.Buildings.Components.Zone space_1(" in model and "ZoneScheduledVentilation" not in model


def test_the_scheduled_ventilation_zone_is_part_of_the_trano_package() -> None:
    space = simple_space_1_fixture()
    network = Network(name="package", library=Library.from_configuration("IDEAS"))
    network.add_boiler_plate_spaces([space])
    model = network.model()

    assert "extends IDEAS.Buildings.Components.Zone(nPorts=nPortsExt + 2);" in model
    assert "connect(souVen.ports[1], ports[nPortsExt + 1])" in model
