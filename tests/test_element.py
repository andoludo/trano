from pathlib import Path

import pytest
import yaml
from linkml.validator import validate_file

from trano.elements import (
    AhuControl,
    Boiler,
    BoilerControl,
    ExternalWall,
    Pump,
    Window,
)
from trano.elements.library.parameters import param_from_config
from trano.elements.types import Azimuth, Tilt


@pytest.mark.skip(reason="This test is not relevant")
def test_validate_schema() -> None:
    for name in ["boiler", "valve", "window"]:
        element = (
            Path(__file__)
            .parents[1]
            .joinpath("trano", "elements", "library", "data", "models", "default", f"{name}.yaml")
        )
        data_model_path = Path(__file__).parents[1].joinpath("trano", "data_models", "trano_final.yaml")
        report = validate_file(element, data_model_path, "Building")
        assert report.results == []


def test_external_wall_basic_construction() -> None:
    from tests.constructions.constructions import Constructions

    wall = ExternalWall(
        name="w_test",
        surface=12.0,
        azimuth=Azimuth.south,
        tilt=Tilt.wall,
        construction=Constructions.external_wall,
    )
    assert wall.name == "w_test"
    assert wall.surface == 12.0
    assert wall.azimuth == Azimuth.south


def test_window_carries_geometry() -> None:
    from tests.constructions.constructions import Glasses

    win = Window(
        name="win_test",
        surface=2.0,
        azimuth=Azimuth.east,
        tilt=Tilt.wall,
        width=2.0,
        height=1.0,
        construction=Glasses.simple_glazing,
    )
    assert win.width * win.height == win.surface


def test_boiler_default_name_increments() -> None:
    Boiler.name_counter = 0
    a = Boiler(name="boiler_a", control=BoilerControl(name="ctrl_a"))
    b = Boiler(control=BoilerControl(name="ctrl_b"))
    assert a.name == "boiler_a"
    # default name follows lowercased class name + counter
    assert b.name.startswith("boiler_")


def test_pump_accepts_typed_parameters() -> None:
    parameters_cls = param_from_config("Pump")
    assert parameters_cls is not None
    pump = Pump(
        name="pump_test",
        parameters=parameters_cls(dp_nominal=10000, m_flow_nominal=0.5),
    )
    assert pump.parameters is not None


def test_ahu_control_can_be_constructed() -> None:
    control = AhuControl(name="ahu_ctrl")
    assert control.name == "ahu_ctrl"


def test_default_library_models_are_valid_yaml() -> None:
    """Every shipped per-component yaml file must be loadable."""
    models_dir = Path(__file__).parents[1].joinpath("trano", "elements", "library", "data", "models", "default")
    yaml_files = sorted(models_dir.glob("*.yaml"))
    assert yaml_files, "no library models under data/models/default"
    for f in yaml_files:
        loaded = yaml.safe_load(f.read_text())
        assert loaded, f"{f} loaded as empty"
