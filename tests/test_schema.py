import os
import subprocess
import tempfile
from pathlib import Path
from typing import Optional

import pytest
import yaml
from linkml.validator import validate_file

from tests.conftest import _read, clean_model, is_success
from trano.data_models.conversion import (
    _parse,
    assign_space_id,
    convert_model,
    convert_network,
)
from trano.simulate.simulate import SimulationOptions, simulate


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


def test_create_model(schema: Path) -> None:
    model_name = "house"
    with tempfile.NamedTemporaryFile(suffix=".json") as temp:
        assert convert(schema, "house.yaml", "json", Path(temp.name))
        model_ = convert_model(model_name, Path(temp.name))
        assert clean_model(model_, model_name) == set(_read(model_name))


def test_create_model_yaml() -> None:
    model_name = "house_yaml"
    model_path = Path(__file__).parents[1].joinpath("tests", "house.yaml")
    model_ = convert_model(model_name, model_path)
    assert clean_model(model_, model_name) == set(_read(model_name))


def test_simulate_model_yaml() -> None:
    model_name = "house_yaml"
    model_path = Path(__file__).parents[1].joinpath("tests", "house.yaml")
    network = convert_network(model_name, model_path)
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as project_path:
        results = simulate(
            Path(project_path),
            network,
            options=SimulationOptions(end_time=3600),
        )
        assert is_success(results)


def test_simplified_yaml() -> None:
    model_path = Path(__file__).parents[1].joinpath("tests", "simplified_house.yaml")
    data = yaml.safe_load(model_path.read_text())
    data = assign_space_id(data)
    _parse(data)
    with tempfile.NamedTemporaryFile(mode="w+", suffix=".yaml") as f:
        yaml.safe_dump(data, f)
        network = convert_network("simplified_yaml", Path(f.name))

        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as project_path:
            results = simulate(
                Path(project_path),
                network,
                options=SimulationOptions(end_time=3600),
            )
            assert is_success(results)
