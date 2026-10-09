"""Hydronic component parameters derived by Trano must match the library conventions."""

import re

import pytest

from tests.golden import remove_trano_package
from tests.test_yaml_values import NUMBER

RADIATOR_POWER = 5000.0  # default Q_flow_nominal of the fixture radiators [W]
BOILER_POWER = 20000.0  # default Q_flow_nominal of the fixture boiler [W]


@pytest.fixture
def hydronic_model(request: pytest.FixtureRequest) -> str:
    return request.getfixturevalue("buildings_simple_hydronic").model()


def scalar(model: str, parameter: str) -> float:
    """Value of `parameter` in the generated model, outside the embedded Trano package."""
    match = re.search(rf"(?<![\w.]){re.escape(parameter)}\s*=\s*({NUMBER})", remove_trano_package(model))
    assert match, f"{parameter} not rendered"
    return float(match.group(1))


def test_radiator_water_volume_and_dry_mass_follow_radiator_en442(hydronic_model: str) -> None:
    # RadiatorEN442_2: VWat = 5.8e-6 * |Q_flow_nominal| and mDry = 0.0263 * |Q_flow_nominal|.
    assert scalar(hydronic_model, "VWat") == pytest.approx(5.8e-6 * RADIATOR_POWER)
    assert scalar(hydronic_model, "mDry") == pytest.approx(0.0263 * RADIATOR_POWER)


def test_boiler_loop_flow_uses_the_boiler_temperature_difference(hydronic_model: str) -> None:
    sca_fac_rad, dt_rad, dt_boi = 1.5, 10.0, 20.0
    assert scalar(hydronic_model, "nominal_mass_flow_radiator_loop") == pytest.approx(
        sca_fac_rad * BOILER_POWER / dt_rad / 4200
    )
    assert scalar(hydronic_model, "nominal_mass_flow_rate_boiler") == pytest.approx(
        sca_fac_rad * BOILER_POWER / dt_boi / 4200
    )


def test_sensors_get_their_own_nominal_flow(hydronic_model: str) -> None:
    assert "mRad_flow_nominal" not in hydronic_model
    sensor = re.search(r"TemperatureTwoPort\s+temperature_sensor\s*\((.*?)\)", hydronic_model, re.DOTALL)
    assert sensor and scalar(sensor.group(1), "m_flow_nominal") == pytest.approx(0.15)


def test_boiler_control_starts_on_supply_set_point_and_stops_on_tank_bottom(hydronic_model: str) -> None:
    assert "greThr(t=\n        threshold_to_switch_off_boiler)" in hydronic_model
    assert "dTThr1(k=\n              TSup_nominal)" in hydronic_model
    assert scalar(hydronic_model, "threshold_to_switch_off_boiler") == pytest.approx(358.15)
    assert scalar(hydronic_model, "TSup_nominal") == pytest.approx(353.15)


def test_gas_usage_integrates_the_fuel_volume_flow_of_the_boiler(hydronic_model: str) -> None:
    assert "fuelVolumeFlow(y=boi.VFue_flow)" in hydronic_model
    assert "connect(fuelVolumeFlow.y, GasUsage.u)" in hydronic_model
    assert "gain2" not in hydronic_model
