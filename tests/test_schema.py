import json
import tempfile
from pathlib import Path

import pytest
from linkml.validator import validate_file

from tests.golden import assert_model_equals_golden
from trano.data_models.conversion import convert_model, convert_network
from trano.data_models.converter import converter
from trano.scripts.schema import create_final_schema
from trano.simulate.simulate import SimulationOptions, simulate
from trano.utils.utils import is_success
import yaml
from jsf import JSF
from linkml.generators.jsonschemagen import JsonSchemaGenerator
from linkml.generators.owlgen import OwlSchemaGenerator
from linkml.generators.pydanticgen import PydanticGenerator
from linkml.generators.pythongen import PythonGenerator
from linkml.linter.linter import Linter


@pytest.fixture
def house() -> Path:
    return Path(__file__).parents[1].joinpath("tests", "models", "house.yaml")


def test_validate_schema() -> None:
    house = Path(__file__).parents[1].joinpath("tests", "models", "house.yaml")
    data_model_path = Path(__file__).parents[1].joinpath("trano", "data_models", "trano.yaml")
    report = validate_file(house, data_model_path, "Building")
    assert report.results == []


@pytest.mark.skip(reason="Create the new schema")
def test_create_new_schema(schema: Path, schema_original: Path, parameters_path: Path) -> None:
    trano_path = schema_original
    trano_final_path = schema
    create_final_schema(parameters_path, trano_final_path, trano_path)
    assert trano_final_path.exists()


def test_convert_to_json(schema: Path, house: Path) -> None:
    for target in ["ttl", "json", "rdf"]:
        # TODO: why doesn't this work with json-ld
        assert converter(
            input=str(house),
            target_class="Building",
            schema=schema,
            output_format=target,
        )


def test_create_model_json(schema: Path, house: Path) -> None:
    model_name = "house"
    with tempfile.NamedTemporaryFile(suffix=".json") as temp:
        data = converter(
            input=str(house),
            target_class="Building",
            schema=schema,
            output_format="json",
        )
        Path(temp.name).write_text(json.dumps(data))
        model_ = convert_model(model_name, Path(temp.name))
        assert_model_equals_golden(model_, model_name)


def test_create_json_schema() -> None:
    data_model_path = Path(__file__).parents[1].joinpath("trano", "data_models", "trano.yaml")
    a = JsonSchemaGenerator(data_model_path).serialize()
    faker = JSF(json.loads(a))
    fake_json = faker.generate()  # noqa: F841
    # TODO: needs validation


def test_create_model_yaml(schema: Path, house: Path) -> None:
    model_name = "house"
    model_ = convert_model(model_name, house)
    assert_model_equals_golden(model_, model_name)


@pytest.mark.simulate
def test_simulate_model_yaml(house: Path) -> None:
    model_name = "house"
    network = convert_network(model_name, house)
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as project_path:
        results = simulate(
            Path(project_path),
            network,
            options=SimulationOptions(end_time=3600),
        )
        assert is_success(results)


@pytest.mark.simulate
def test_simulate_simplified_yaml() -> None:
    model_path = Path(__file__).parents[1].joinpath("tests", "models", "simplified_house.yaml")
    network = convert_network("simplified_yaml", model_path)
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as project_path:
        results = simulate(
            Path(project_path),
            network,
            options=SimulationOptions(end_time=3600),
        )
        assert is_success(results)


SCHEMA_DIRECTORY = Path(__file__).parents[1].joinpath("trano", "data_models")
FINAL_SCHEMA = SCHEMA_DIRECTORY.joinpath("trano_final.yaml")


def test_final_schema_is_generated_from_its_sources(tmp_path: Path) -> None:
    """trano_final.yaml is trano.yaml and parameters.yaml merged: regenerate it after changing them."""
    generated = tmp_path / "trano_final.yaml"
    create_final_schema(SCHEMA_DIRECTORY / "parameters.yaml", generated, SCHEMA_DIRECTORY / "trano.yaml")
    assert yaml.safe_load(generated.read_text()) == yaml.safe_load(FINAL_SCHEMA.read_text())


def test_final_schema_lints_without_errors() -> None:
    """The remaining warnings are names fixed by Modelica (k, r, Kv, Cv, Av) or by existing models."""
    problems = list(Linter({"extends": "recommended"}).lint(str(FINAL_SCHEMA)))
    assert [problem.message for problem in problems if str(problem.level) == "error"] == []
    assert {problem.rule_name for problem in problems} <= {"standard_naming"}


@pytest.mark.parametrize("generator", [JsonSchemaGenerator, PydanticGenerator, PythonGenerator, OwlSchemaGenerator])
def test_final_schema_generates(generator: type) -> None:
    assert generator(str(FINAL_SCHEMA)).serialize()
