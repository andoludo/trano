import shutil
from pathlib import Path

import numpy as np
import pandas as pd
import pytest
from pydantic import ValidationError
from typer.testing import CliRunner

from trano.data_models.conversion import convert_network
from trano.elements.library.library import Library
from trano.main import app
from trano.mpc import (
    EstimationSettings,
    ISO13790Parameters,
    Orientation,
    R3C2Parameters,
    RCBuilding,
    RCModelType,
    RCZone,
    ZoneCoupling,
    rc_building_from_yaml,
)
from trano.mpc.estimation import reference_zone_parameters
from trano.topology import Network

MODELS = Path(__file__).parent / "models"
THREE_ZONES = MODELS / "three_zones_mpc.yaml"
OCCUPANCY_FROM_DATA = MODELS / "single_zone_hydronic_occupancy_from_data.yaml"


def mpc_library(model_type: RCModelType | None = None) -> Library:
    library = Library.from_configuration("mpc")
    return library.model_copy(update={"rc_model_type": model_type}) if model_type else library


def mpc_network(path: Path = THREE_ZONES, model_type: RCModelType | None = None, name: str = "house") -> Network:
    return convert_network(name, path, library=mpc_library(model_type))


def model_section(source: str, name: str) -> str:
    start = source.index(f"model {name} ")
    return source[start : source.index(f"end {name};", start)]


@pytest.fixture(scope="module")
def three_zones() -> dict[RCModelType, RCBuilding]:
    return {model_type: rc_building_from_yaml(THREE_ZONES, model_type=model_type) for model_type in RCModelType}


def test_modelica_parameters_follow_declaration_order() -> None:
    parameters = reference_zone_parameters(RCModelType.r3c2)
    assert [p.name for p in parameters.modelica_parameters()] == ["Ci", "Ce", "Ria", "Rie", "Rea", "Reg"]


def test_zone_parameters_discriminated_by_model_type() -> None:
    parameters = reference_zone_parameters(RCModelType.iso13790)
    zone = RCZone.model_validate({"name": "zone", "parameters": parameters.model_dump()})
    assert isinstance(zone.parameters, ISO13790Parameters)


@pytest.mark.parametrize(
    ("zones", "couplings"),
    [
        (["zone", "zone"], []),
        (["zone_a", "zone_b"], [("zone_a", "unknown")]),
        (["zone_a", "zone_b"], [("zone_a", "zone_a")]),
        (["1zone"], []),
    ],
)
def test_building_validation(zones: list[str], couplings: list[tuple[str, str]]) -> None:
    parameters = reference_zone_parameters(RCModelType.r3c2)
    with pytest.raises(ValidationError):
        RCBuilding(
            zones=[RCZone(name=name, parameters=parameters) for name in zones],
            couplings=[ZoneCoupling(zone_a=a, zone_b=b, conductance=10) for a, b in couplings],
        )


def test_negative_resistance_rejected() -> None:
    values = reference_zone_parameters(RCModelType.r3c2).model_dump()
    with pytest.raises(ValidationError):
        R3C2Parameters(**{**values, "indoor_envelope_resistance": -1})


@pytest.mark.parametrize(
    ("azimuth", "tilt", "name"), [(0, 90, "azi0_til90"), (-90, 90, "azim90_til90"), (22.5, 0, "azi22p5_til0")]
)
def test_orientation_names_are_modelica_identifiers(azimuth: float, tilt: float, name: str) -> None:
    orientation = Orientation(azimuth=azimuth, tilt=tilt)
    assert orientation.name == name
    assert orientation.irradiance == f"HSol_{name}"


def test_rc_building_from_yaml(three_zones: dict[RCModelType, RCBuilding]) -> None:
    building = three_zones[RCModelType.r3c2]
    assert [zone.name for zone in building.zones] == ["space_001", "space_002", "space_003"]
    couplings = {(c.zone_a, c.zone_b): c.conductance for c in building.couplings}
    assert couplings == pytest.approx(
        {("space_001", "space_002"): 7.0847, ("space_002", "space_003"): 4.5035}, rel=1e-4
    )
    zone = building.zones[0]
    assert isinstance(zone.parameters, R3C2Parameters)
    # 250 m3 at 0.5 1/h (41.875 W/K) in parallel with the windows.
    assert 1 / zone.parameters.indoor_outdoor_resistance == pytest.approx(41.875 + 10.521, rel=1e-3)
    assert zone.parameters.air_capacitance == pytest.approx(1.2 * 1005 * 250 * 5)
    assert zone.floor_area == 100
    # Windows facing south (0) and north (180), opaque walls facing south, west (90) and north.
    apertures = {aperture.orientation.name: aperture for aperture in zone.solar_apertures}
    assert set(apertures) == {"azi0_til90", "azi90_til90", "azi180_til90"}
    assert apertures["azi0_til90"].window > 0
    assert apertures["azi90_til90"].window == 0
    assert apertures["azi90_til90"].opaque > 0
    assert [o.name for o in building.orientations] == ["azi0_til90", "azi90_til90", "azi180_til90"]


def test_estimation_settings_are_used() -> None:
    building = rc_building_from_yaml(THREE_ZONES, settings=EstimationSettings(air_capacity_multiplier=1))
    assert building.zones[0].parameters.air_capacitance == pytest.approx(1.2 * 1005 * 250)  # type: ignore[union-attr]


@pytest.mark.parametrize("model_type", list(RCModelType))
def test_building_mpc_is_flat_and_library_free(
    three_zones: dict[RCModelType, RCBuilding], model_type: RCModelType
) -> None:
    source = three_zones[model_type].to_modelica("MyPackage")
    assert source.startswith("package MyPackage")
    assert source.rstrip().endswith("end MyPackage;")
    building = model_section(source, "building_mpc")
    assert "Modelica." not in building  # no dependency, not even on the MSL
    assert "Buildings." not in building
    assert "connect(" not in building
    assert "der(space_003_Ti)" in building
    assert "H_space_001_space_002*(space_002_Ti - space_001_Ti)" in building
    assert "space_001_gA_azi0_til90*HSol_azi0_til90" in building


def test_mpc_library_is_registered() -> None:
    assert Library.from_configuration("mpc").rc_model_type == RCModelType.r3c2
    assert not Library.from_configuration("Buildings").is_rc


def test_trano_library_embeds_mpc_zones() -> None:
    source = convert_network("house", THREE_ZONES).model()
    trano_package = source[source.index("package Trano") : source.index("end Trano;")]
    for model_type in RCModelType:
        assert f"model {model_type.value} " in trano_package
    assert "end MPC;" in trano_package


@pytest.mark.parametrize("model_type", [None, *RCModelType])
def test_mpc_library_generates_mpc_and_runnable_models(model_type: RCModelType | None) -> None:
    source = mpc_network(model_type=model_type).model()
    assert source.startswith("package house")
    assert "package Trano" in source
    assert f"({(model_type or RCModelType.r3c2).value})" in model_section(source, "building_mpc")
    runnable = model_section(source, "building")
    assert "building_mpc rc" in runnable


def test_runnable_model_uses_the_weather_file_of_the_other_libraries() -> None:
    mpc = model_section(mpc_network().model(), "building")
    buildings = convert_network("house", THREE_ZONES).model()
    assert "filNam=../tests/resources/BEL_VLG_Uccle.064470_TMYx.2007-2021.mos" in " ".join(buildings.split())
    assert 'ReaderTMY3 weather(filNam="../tests/resources/BEL_VLG_Uccle.064470_TMYx.2007-2021.mos");' in mpc
    assert "connect(weather.weaBus, weaBus);" in mpc
    assert "connect(weaBus.TDryBul, rc.TOut);" in mpc


def test_runnable_model_connects_solar_gains_of_each_orientation() -> None:
    runnable = model_section(mpc_network().model(), "building")
    for orientation in ("azi0_til90", "azi90_til90", "azi180_til90"):
        assert f"DirectTiltedSurface HDir_{orientation}(" in runnable
        assert f"DiffuseIsotropic HDif_{orientation}(" in runnable
        assert f"connect(weather.weaBus, HDir_{orientation}.weaBus);" in runnable
        assert f"connect(HSol_{orientation}.y, rc.HSol_{orientation});" in runnable
    assert "til=1.570796327, azi=3.141592654" in runnable


def test_runnable_model_connects_occupancy_heating_and_outputs() -> None:
    runnable = model_section(mpc_network().model(), "building")
    for zone, floor_area in (("space_001", 100), ("space_002", 70), ("space_003", 50)):
        assert f"Trano.Occupancy.SimpleOccupancy occupancy_{zone}(" in runnable
        assert f"QInt_{zone}(nin=2, k={{{floor_area}, {floor_area}}})" in runnable
        assert f"connect(QInt_{zone}.y, rc.{zone}_QInt);" in runnable
        assert f"Modelica.Blocks.Interfaces.RealInput {zone}_QHea" in runnable
        assert f"connect({zone}_QHea, rc.{zone}_QHea);" in runnable
        assert f"{zone}_TZon = rc.{zone}_Ti;" in runnable


def _external_data(path: Path) -> Path:
    index = pd.date_range("2024-01-01", periods=25, freq="h")
    hour = np.arange(25)
    data = pd.DataFrame(
        {"co2_01": np.where((hour >= 9) & (hour < 17), 900.0, 420.0), "space_001_QHea": 1500.0}, index=index
    )
    data.to_csv(path)
    return path


def test_runnable_model_reads_external_data(tmp_path: Path) -> None:
    network = mpc_network(OCCUPANCY_FROM_DATA, name="house_data")
    network.external_data = _external_data(tmp_path / "data.csv")
    runnable = model_section(network.model(), "building")
    assert "CombiTimeTable externalData(" in runnable
    assert "columns: co2_01, space_001_QHea" in runnable
    assert "Trano.Occupancy.OccupancyCo2 occupancy_space_001(" in runnable
    assert "co2(u=externalData.y[1])" in runnable
    # The heating power is replayed from the data instead of being a free input.
    assert "connect(externalData.y[2], rc.space_001_QHea);" in runnable
    assert "RealInput space_001_QHea" not in runnable


def test_occupancy_data_requires_external_data() -> None:
    with pytest.raises(ValueError, match="co2_01"):
        mpc_network(OCCUPANCY_FROM_DATA).model()


def test_cli_create_model_with_mpc_library(tmp_path: Path) -> None:
    model_path = tmp_path / "house.yaml"
    shutil.copy(THREE_ZONES, model_path)
    result = CliRunner().invoke(app, ["create-model", str(model_path), "mpc", "--rc-model-type", "R4C3"])
    assert result.exit_code == 0, result.output
    source = model_path.with_suffix(".mo").read_text()
    assert "der(space_001_Th)" in model_section(source, "building_mpc")
    assert "building_mpc rc" in model_section(source, "building")
