from pathlib import Path

from trano.models.elements.bus import transform_csv_to_table


def test_transform_csv_to_table() -> None:
    validation_data = transform_csv_to_table(
        Path(__file__).parent.joinpath("resources", "validation.csv")
    )
    assert validation_data.data is not None
