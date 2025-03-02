from pathlib import Path

import pytest
from linkml.validator import validate_file


@pytest.mark.skip(reason="This test is not relevant")
def test_validate_schema() -> None:
    for name in ["boiler", "valve", "window"]:

        element = (
            Path(__file__)
            .parents[1]
            .joinpath(
                "trano", "elements", "library", "models", "default", f"{name}.yaml"
            )
        )
        data_model_path = (
            Path(__file__)
            .parents[1]
            .joinpath("trano", "data_models", "trano_final.yaml")
        )
        report = validate_file(element, data_model_path, "Building")
        assert report.results == []
