"""A model takes the name of its YAML file: its generated package must not hide itself behind one of its classes."""

import shutil
from pathlib import Path

import pytest
from typer.testing import CliRunner

from trano.data_models.conversion import convert_network
from trano.exceptions import InvalidModelNameError
from trano.main import app
from trano.topology import RESERVED_MODEL_NAMES, Network

MODELS = Path(__file__).parent / "models"


@pytest.mark.parametrize("name", sorted(RESERVED_MODEL_NAMES))
def test_a_model_cannot_take_the_name_of_a_class_of_its_package(name: str) -> None:
    with pytest.raises(InvalidModelNameError, match=f"Rename the YAML file, e.g. 'my_{name}.yaml'"):
        Network(name=name)


def test_a_name_that_contains_a_reserved_one_is_accepted() -> None:
    assert Network(name="my_building").name == "my_building"


def test_a_yaml_file_named_building_is_refused_before_generating(tmp_path: Path) -> None:
    """``building.yaml`` generated ``package building`` holding ``model building``: OpenModelica then reported
    ``Class building.Components.BaseClasses.PumpPump_001 not found in scope distribution``."""
    model = tmp_path / "building.yaml"
    shutil.copy(MODELS / "three_zones_hydronic.yaml", model)

    with pytest.raises(InvalidModelNameError):
        convert_network(model.stem, model)

    result = CliRunner().invoke(app, ["create-model", str(model), "IDEAS"])
    assert isinstance(result.exception, InvalidModelNameError)
    assert not model.with_suffix(".mo").exists()
