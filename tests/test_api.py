"""The one-call API: a YAML model in, a ``SimulationResult`` out."""

from pathlib import Path
from typing import Any

import pytest
import yaml

import trano
from tests.fixtures.fake_omc import FakeOmc
from trano.api import check_yaml, modelica_name, simulate_yaml
from trano.simulate.result import SimulationErrorKind
from trano.simulate.simulate import SimulationOptions

FIRST_MODEL = Path(__file__).parent / "tutorials" / "first_model.yaml"


def first_model() -> dict[str, Any]:
    return dict(yaml.safe_load(FIRST_MODEL.read_text()))


def test_the_api_is_exported_from_the_package() -> None:
    assert trano.simulate_yaml is simulate_yaml
    assert trano.check_yaml is check_yaml


def test_a_yaml_file_is_simulated(tmp_path: Path, fake_omc: FakeOmc) -> None:
    phases: list[str] = []

    result = simulate_yaml(
        FIRST_MODEL,
        options=SimulationOptions(end_time=3600),
        project_path=tmp_path,
        runtime="local",
        on_phase=phases.append,
    )

    assert result.success, result.error_message
    assert result.result_path == tmp_path.resolve() / "results" / "first_model.building_res.mat"
    assert phases == ["generating", "compiling", "simulating", "done"]
    assert result.duration_s > 0
    assert "stopTime = 3600" in str(fake_omc.record["script"])


def test_a_parsed_model_is_written_into_the_project(tmp_path: Path, fake_omc: FakeOmc) -> None:
    result = simulate_yaml(first_model(), project_path=tmp_path, runtime="local", name="my-house")

    assert result.success, result.error_message
    assert yaml.safe_load((tmp_path / "my_house.yaml").read_text()) == first_model()
    assert result.result_path == tmp_path.resolve() / "results" / "my_house.building_res.mat"


def test_a_temporary_project_is_created_when_none_is_given(fake_omc: FakeOmc) -> None:
    result = simulate_yaml(first_model(), runtime="local")

    assert result.result_path is not None
    assert result.result_path.parent.parent.name.startswith("trano-")


def test_a_check_does_not_simulate(tmp_path: Path, fake_omc: FakeOmc) -> None:
    result = check_yaml(FIRST_MODEL, project_path=tmp_path, runtime="local")

    assert result.success and result.result_path is None
    assert "simulate(" not in str(fake_omc.record["script"])


def test_an_invalid_model_is_reported_not_raised(tmp_path: Path) -> None:
    model = first_model()
    model["spaces"][0]["parameters"]["floor_area"] = -100.0
    phases: list[str] = []

    result = simulate_yaml(model, project_path=tmp_path, runtime="local", on_phase=phases.append)

    assert not result.success
    assert result.error_kind is SimulationErrorKind.validation
    assert result.error_message == "spaces[0].parameters.floor_area: must be >= 0 (got -100.0) (Floor area [m2])"
    assert phases == ["generating", "done"]


def test_a_missing_model_file_is_a_validation_error(tmp_path: Path) -> None:
    result = simulate_yaml(tmp_path / "nowhere.yaml", project_path=tmp_path, runtime="local")

    assert result.error_kind is SimulationErrorKind.validation
    assert "nowhere.yaml" in str(result.error_message)


def test_a_missing_openmodelica_is_a_runtime_error(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("PATH", str(tmp_path))

    result = simulate_yaml(first_model(), project_path=tmp_path / "project", runtime="local")

    assert result.error_kind is SimulationErrorKind.runtime
    assert "TRANO_SIMULATION_RUNTIME" in str(result.error_message)


def test_an_unknown_runtime_is_a_runtime_error(tmp_path: Path) -> None:
    result = simulate_yaml(first_model(), project_path=tmp_path, runtime="podman")

    assert result.error_kind is SimulationErrorKind.runtime


def test_solver_failures_are_returned(tmp_path: Path, fake_omc: FakeOmc) -> None:
    fake_omc.replay("solver_missing_weather.log")

    result = simulate_yaml(FIRST_MODEL, project_path=tmp_path, runtime="local")

    assert result.error_kind is SimulationErrorKind.solver
    assert str(result.error_message).startswith("Not possible to open file")


@pytest.mark.parametrize(("name", "identifier"), [("house", "house"), ("my-house 2", "my_house_2"), ("3d", "_3d")])
def test_package_names_are_modelica_identifiers(name: str, identifier: str) -> None:
    assert modelica_name(name) == identifier
