from pathlib import Path

from tests.test_template import _read, clean_model
from trano.library.library import Buildings
from trano.models.elements.bus import transform_csv_to_table
from trano.models.elements.space import Space
from trano.topology import Network


def test_transform_csv_to_table() -> None:
    validation_data = transform_csv_to_table(
        Path(__file__).parent.joinpath("resources", "validation.csv")
    )
    assert validation_data.data is not None


def test_template_buildings_free_float_single_zone_with_data(
    simple_space_1_with_occupancy: Space,
) -> None:
    network = Network(
        name="buildings_free_float_single_zone",
        library=Buildings(
            constants="""package Medium = Buildings.Media.Air(extraPropertiesNames={"CO2"}) "Medium model";
    package MediumW = Buildings.Media.Water "Medium model";"""
        ),
        external_data=Path(__file__).parent.joinpath("resources", "validation.csv"),
    )
    network.add_boiler_plate_spaces([simple_space_1_with_occupancy])
    model_ = network.model()
    assert clean_model(model_, network.name) == set(_read(network.name))
