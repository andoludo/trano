from pathlib import Path

import pytest
from pydantic import ValidationError

from trano.mpc import (
    EstimationSettings,
    ISO13790Parameters,
    R3C2Parameters,
    RCBuilding,
    RCModelType,
    RCZone,
    ZoneCoupling,
    rc_building_from_yaml,
)
from trano.mpc.estimation import reference_zone_parameters

THREE_ZONES = Path(__file__).parent / "models" / "three_zones_ideal_heaters.yaml"
KELVIN = 273.15


@pytest.fixture(scope="module")
def three_zones() -> dict[RCModelType, RCBuilding]:
    return {model_type: rc_building_from_yaml(THREE_ZONES, model_type=model_type) for model_type in RCModelType}


def _single_zone(model_type: RCModelType) -> RCBuilding:
    return RCBuilding(zones=[RCZone(name="zone", parameters=reference_zone_parameters(model_type))])


def test_modelica_parameters_follow_declaration_order() -> None:
    parameters = reference_zone_parameters(RCModelType.r3c2)
    assert [p.name for p in parameters.modelica_parameters()] == ["Ci", "Ce", "Ria", "Rie", "Rea", "Reg", "gA", "aE"]


def test_zone_parameters_discriminated_by_model_type() -> None:
    parameters = reference_zone_parameters(RCModelType.iso13790)
    zone = RCZone.model_validate({"name": "zone", "parameters": parameters.model_dump()})
    assert isinstance(zone.parameters, ISO13790Parameters)


@pytest.mark.parametrize(
    "zones, couplings",
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


def test_rc_building_from_yaml(three_zones: dict[RCModelType, RCBuilding]) -> None:
    building = three_zones[RCModelType.r3c2]
    assert [zone.name for zone in building.zones] == ["space_001", "space_002", "space_003"]
    couplings = {(c.zone_a, c.zone_b): c.conductance for c in building.couplings}
    assert couplings == pytest.approx(
        {("space_001", "space_002"): 7.0847, ("space_002", "space_003"): 4.5035}, rel=1e-4
    )
    zone = building.zones[0].parameters
    assert isinstance(zone, R3C2Parameters)
    # 250 m3 at 0.5 1/h (41.875 W/K) in parallel with the windows.
    assert 1 / zone.indoor_outdoor_resistance == pytest.approx(41.875 + 10.521, rel=1e-3)
    assert zone.air_capacitance == pytest.approx(1.2 * 1005 * 250 * 5)


def test_estimation_settings_are_used() -> None:
    building = rc_building_from_yaml(THREE_ZONES, settings=EstimationSettings(air_capacity_multiplier=1))
    assert building.zones[0].parameters.air_capacitance == pytest.approx(1.2 * 1005 * 250)  # type: ignore[union-attr]


@pytest.mark.parametrize("model_type", list(RCModelType))
def test_to_modelica(three_zones: dict[RCModelType, RCBuilding], model_type: RCModelType) -> None:
    source = three_zones[model_type].to_modelica("MyPackage")
    assert source.startswith("package MyPackage")
    assert source.rstrip().endswith("end MyPackage;")
    assert "Modelica." not in source  # no dependency, not even on the MSL
    for library_model in RCModelType:
        assert f"model {library_model.value} " in source
    assert "der(space_003_Ti)" in source
    assert "H_space_001_space_002*(space_002_Ti - space_001_Ti)" in source
