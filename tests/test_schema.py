import os
import subprocess
import tempfile
from pathlib import Path
from typing import Optional

import pytest
from linkml.validator import validate_file

from tests.conftest import _read, clean_model
from trano.data_models.conversion import convert_model


def convert(
    schema: Path, input_file: str, target: str, output: Optional[Path] = None
) -> bool:
    root_path = Path(__file__).parents[1]
    os.chdir(root_path)
    input = root_path.joinpath("tests", input_file)
    output = output or root_path.joinpath(
        "tests", f"{input.stem}.{target.replace('-', '_')}"
    )
    command = [
        "poetry",
        "run",
        "linkml-convert",
        "-o",
        f"{output}",
        "-t",
        target,
        "-f",
        "yaml",
        "-C",
        "Building",
        "-s",
        str(schema),
        f"{input}",
    ]

    process = subprocess.run(command, check=True, capture_output=True, text=True)
    return process.returncode == 0


@pytest.fixture
def schema() -> Path:
    return Path(__file__).parents[1].joinpath("trano", "data_models", "trano.yaml")


def test_validate_schema() -> None:
    house = Path(__file__).parents[1].joinpath("tests", "house.yaml")
    data_model_path = (
        Path(__file__).parents[1].joinpath("trano", "data_models", "trano.yaml")
    )
    report = validate_file(house, data_model_path, "Building")
    assert report.results == []


def test_convert_to_json(schema: Path) -> None:
    for target in ["ttl", "json", "rdf", "json-ld"]:
        with tempfile.NamedTemporaryFile() as temp:
            assert convert(schema, "house.yaml", target, Path(temp.name))


def test_convert_to_json_(schema: Path) -> None:
    for target in ["json"]:
        assert convert(schema, "house.yaml", target)


def test_create_model() -> None:
    model_name = "house"
    model_path = Path(__file__).parents[1].joinpath("tests", "house.json")
    model_ = convert_model(model_name, model_path)
    assert clean_model(model_, model_name) == set(_read(model_name))
