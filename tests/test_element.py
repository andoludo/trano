from pathlib import Path

import yaml
from linkml.validator import validate_file

from trano.library.library import Ideas, Buildings, Library


# from trano.models.elements.materials.properties import BuildingsData, IdeasData


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


def test_material():
    buildings_data = BuildingsData()
    ideas_data = IdeasData()

    materials_path = Path(
        "/trano/elements/materials/materials.yaml"
    )

    with materials_path.open("w") as f:
        yaml.safe_dump(
            {"ideas": ideas_data.model_dump(), "building": buildings_data.model_dump()},
            f,
        )


def test_library():
    def get_library(name, library):
        materials_path = Path(
            "/trano/elements/materials/materials.yaml"
        )
        templates = yaml.safe_load(materials_path.read_text())
        library_template = templates[name]
        templates = {
            "construction": library_template["construction"]["template"],
            "glazing": library_template["glazing"]["template"],
            "material": library_template["material"]["template"],
            "main": library_template["template"],
        }
        library_ = library().model_dump()
        library_.pop("functions")
        library_["templates"] = templates
        return library_
    ideas = get_library("ideas", Ideas)
    buildings = get_library("building", Buildings)
    materials_path = Path(
        "/home/aan/Documents/trano/trano/library/library.yaml"
    )

    with materials_path.open("w") as f:
        yaml.safe_dump(
            {"IDEAS": ideas, "Buildings": buildings},
            f,
        )


def test_load():
    # Library.from_configuration("IDEAS")
    Library.load_default()