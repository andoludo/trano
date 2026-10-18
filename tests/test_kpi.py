"""Heating KPIs read from result files of ``tests/tutorials/first_model.yaml`` (see tests/data/kpi)."""

from pathlib import Path

import numpy as np
import pytest
from buildingspy.io.outputfile import Reader  # type: ignore[import-untyped]

from tests.fixtures.result_files import reduce_result
from trano.data_models.conversion import convert_network
from trano.elements.library.library import Library
from trano.kpi import building_kpis

TESTS = Path(__file__).parent
KPI_DATA = TESTS / "data" / "kpi"
FIRST_MODEL = TESTS / "tutorials" / "first_model.yaml"
HEATING = "building.emission1.heating_001"


def result_file(library: str) -> Path:
    return KPI_DATA / f"first_model_{library.lower()}_1day.mat"


@pytest.mark.parametrize("library", ["Buildings", "IDEAS"])
def test_heating_kpis_of_one_day(library: str) -> None:
    network = convert_network("first_model", FIRST_MODEL, library=Library.from_configuration(library))
    time, power = Reader(str(result_file(library)), "openmodelica").values(f"{HEATING}.QHea_flow")

    kpi = building_kpis(result_file(library), network)

    energy_kwh = np.trapezoid(power, time) / 3.6e6
    assert (kpi.period_start_s, kpi.period_end_s) == (0.0, 86400.0)
    assert kpi.heated_floor_area_m2 == 100.0
    assert kpi.heating_energy_kwh == pytest.approx(energy_kwh)
    assert kpi.heating_energy_kwh_per_m2 == pytest.approx(energy_kwh / 100.0)
    assert kpi.peak_heating_w == pytest.approx(power.max())
    assert 10 < kpi.heating_energy_kwh < 30, "a 100 m2 house on a January day in Uccle"
    (zone,) = kpi.zones
    assert zone.zone_id == "living_001"
    assert zone.heating_energy_kwh == pytest.approx(kpi.heating_energy_kwh)
    assert zone.hours_below_setpoint is not None and 0 <= zone.hours_below_setpoint < 1
    assert kpi.source_variables == {
        "living_001.heating_power": f"{HEATING}.QHea_flow",
        "living_001.air_temperature": f"{HEATING}.TAir.T",
        "living_001.heating_setpoint": f"{HEATING}.heaSch.y[1]",
    }


def test_hours_below_setpoint_need_the_setpoint(tmp_path: Path) -> None:
    network = convert_network("first_model", FIRST_MODEL)
    power_only = tmp_path / "power_only.mat"
    reduce_result(result_file("Buildings"), power_only, [f"{HEATING}.QHea_flow"])

    kpi = building_kpis(power_only, network)

    assert kpi.zones[0].hours_below_setpoint is None
    assert list(kpi.source_variables) == ["living_001.heating_power"]


def test_a_missing_heating_power_is_reported(tmp_path: Path) -> None:
    network = convert_network("first_model", FIRST_MODEL)
    temperature_only = tmp_path / "temperature_only.mat"
    reduce_result(result_file("Buildings"), temperature_only, [f"{HEATING}.TAir.T"])

    with pytest.raises(ValueError, match=r"heating_001\.QHea_flow"):
        building_kpis(temperature_only, network)


def test_only_ideal_heating_is_supported() -> None:
    network = convert_network("hydronic", TESTS / "tutorials" / "three_zones_hydronic_heating.yaml")

    with pytest.raises(NotImplementedError, match="kpi: only ideal heating in phase 1"):
        building_kpis(result_file("Buildings"), network)
