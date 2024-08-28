from pathlib import Path

from linkml.validator import validate_file


def test_validate_schema() -> None:
    for name in ["boiler", "valve", "window"]:

        element = (
            Path(__file__)
            .parents[1]
            .joinpath("trano", "models", "elements", "models", f"{name}.yaml")
        )
        data_model_path = (
            Path(__file__)
            .parents[1]
            .joinpath("trano", "models", "elements", "element.yaml")
        )
        report = validate_file(element, data_model_path, "Building")
        assert report.results == []
