"""The FastModel-against-Buildings study (validation/fast_model): cases, perturbations, series and KPIs, without
simulating anything."""

from pathlib import Path

import numpy as np
import pandas as pd
import pytest
import yaml

from validation.fast_model.__main__ import plan
from validation.fast_model.cases import BASE, CASES, offset_schedule
from validation.fast_model.harness import (
    BUILDINGS,
    FAST_MODEL,
    compare_series,
    compute_kpis,
    network_for,
    variant_yaml,
    zone_names,
)


def test_the_cases_are_the_complex_bestest_cases_and_the_house_after_and_before_renovation() -> None:
    assert list(CASES) == ["950FF", "940", "960", "residential_after", "residential_before"]
    assert all(case.path.exists() for case in CASES.values())
    assert CASES["950FF"].variants == [
        "base",
        "insulation_x1.5",
        "infiltration_x2",
        "mass_x0.5",
        "glazing_tau_x0.75",
        "absorptance_0.9",
        "gains_x2",
    ]
    assert {"setpoint_+2K", "gap_k_x0.5", "insulation_x0.5", "infiltration_x3"} <= set(CASES["940"].variants)
    assert len(CASES["960"].perturbations) == 4
    assert len(CASES["residential_after"].perturbations) == 12
    assert len(CASES["residential_before"].perturbations) == 9
    assert {perturbation.group for case in CASES.values() for perturbation in case.perturbations} == {
        "envelope",
        "system",
        "use",
    }
    for case in CASES.values():
        assert len(set(case.variants)) == len(case.variants), case.id


def test_the_sweeps_scale_one_parameter_by_several_factors() -> None:
    from validation.fast_model.report import sweeps

    steps = sweeps(CASES["940"])
    assert {name: [step.factor for step in group] for name, group in steps.items()} == {
        "insulation": [0.5, 0.75, 1.5, 2.0],
        "infiltration": [0.5, 1.5, 2.0, 3.0],
    }
    assert [step.id for step in steps["insulation"]] == [
        "insulation_x0.5",
        "insulation_x0.75",
        "insulation_x1.5",
        "insulation_x2",
    ]
    assert list(sweeps(CASES["residential_after"])) == ["eps"]
    assert list(sweeps(CASES["residential_before"])) == ["n50"]
    assert sweeps(CASES["960"]) == {}


def test_every_perturbation_changes_its_case(tmp_path: Path) -> None:
    for case in CASES.values():
        base = yaml.safe_load(case.path.read_text())
        assert yaml.safe_load(variant_yaml(case, BASE, tmp_path / case.id / BASE).read_text()) == base
        for perturbation in case.perturbations:
            changed = yaml.safe_load(
                variant_yaml(case, perturbation.id, tmp_path / case.id / perturbation.id).read_text()
            )
            assert changed != base, perturbation.id
            assert set(changed) == set(base), perturbation.id


def test_the_perturbations_change_what_they_say(tmp_path: Path) -> None:
    def document(case_id: str, variant: str) -> dict:  # type: ignore[type-arg]
        return yaml.safe_load(variant_yaml(CASES[case_id], variant, tmp_path / case_id / variant).read_text())  # type: ignore[no-any-return]

    insulation = document("950FF", "insulation_x1.5")
    assert next(m for m in insulation["material"] if m["id"] == "FOAM_INSULATION:001")["thermal_conductivity"] == 0.06
    assert document("950FF", "infiltration_x2")["spaces"][0]["parameters"]["ach"] == pytest.approx(0.828)
    setback = document("940", "setpoint_+2K")["spaces"][0]["emissions"][0]["ideal_heating_cooling"]
    assert setback["parameters"]["heating_setpoint_schedule"].startswith("[0, 285.15; 25200, 285.15; 28800, 295.15")
    house = document("residential_after", "heat_pump_2kW")
    assert house["systems"][0]["boiler"]["parameters"]["nominal_heating_power"] == 2000
    radiators = document("residential_after", "radiators_x0.7")
    living = radiators["spaces"][0]["emissions"][0]["radiator"]["parameters"]
    assert living["nominal_heating_power_positive_for_heating"] == pytest.approx(3900 * 0.7)
    setpoint = document("residential_after", "setpoint_+1K")
    controls = {
        space["id"]: space["emissions"][1]["valve"]["control"]["emission_control"]["parameters"][
            "temperature_heating_setpoint"
        ]
        for space in setpoint["spaces"]
    }
    assert controls["LIVING:001"] == controls["KITCHEN:001"] == 295.15
    assert controls["BEDROOM_1:001"] == 291.15

    sweep_step = document("940", "insulation_x0.75")
    assert next(m for m in sweep_step["material"] if m["id"] == "FOAM_INSULATION:001")["thermal_conductivity"] == 0.03
    glass = next(g for g in document("950FF", "glazing_tau_x0.75")["glass_material"] if g["id"] == "CLEAR_GLASS:001")
    assert glass["solar_transmittance"] == [pytest.approx(0.6255)]
    absorptance = {m["id"]: m["shortwave_emissivity"] for m in document("940", "absorptance_0.9")["material"]}
    assert absorptance["WOOD_SIDING:001"] == absorptance["ROOF_DECK:001"] == 0.9
    assert absorptance["PLASTERBOARD:001"] == 0.6
    assert document("940", "gains_x2")["spaces"][0]["occupancy"]["parameters"]["gain"] == "[5; 3.33333; 0]"
    assert document("940", "gap_k_x0.5")["gas"][0]["thermal_conductivity"] == 0.0125
    sunspace = document("960", "sunspace_infiltration_x2")["spaces"]
    assert {space["id"]: space["parameters"]["ach"] for space in sunspace} == {"ZONE:001": 0.414, "SUNSPACE:001": 0.828}
    ventilation = document("residential_after", "ventilation_x1.5")
    unit = next(s["air_handling_unit"] for s in ventilation["systems"] if "air_handling_unit" in s)
    assert unit["parameters"]["m_flow_nominal"] == pytest.approx(0.075)
    assert ventilation["spaces"][0]["ventilation_inlets"][0]["duct"]["parameters"]["nominal_mass_flow_rate"] == (
        pytest.approx(0.01333 * 1.5)
    )
    exponent = document("residential_after", "radiator_n_1.33")["spaces"][0]["emissions"][0]["radiator"]
    assert exponent["parameters"]["exponent_for_heat_transfer"] == 1.33

    cavity = document("residential_before", "cavity_insulated")["material"]
    assert next(m for m in cavity if m["id"] == "CAVITY_AIR_5CM:001")["thermal_conductivity"] == 0.035
    boiler = document("residential_before", "boiler_10kW")["systems"][0]["boiler"]["parameters"]
    assert boiler["nominal_heating_power"] == 10000
    supply = document("residential_before", "supply_-10K")["systems"][0]["boiler"]["parameters"]
    assert supply["supply_temperature_setpoint"] == 338.15
    assert document("residential_before", "n50_x0.5")["spaces"][0]["parameters"]["n50"] == 6.0


def test_offset_schedule() -> None:
    assert (
        offset_schedule("[0, 283.15; 25200, 283.15; 28800, 293.15]", 2.0) == "[0, 285.15; 25200, 285.15; 28800, 295.15]"
    )
    assert offset_schedule("[0, 293.15]", -1.5) == "[0, 291.65]"


@pytest.mark.parametrize("case_id", list(CASES))
def test_every_variant_generates_a_fast_model(case_id: str, tmp_path: Path) -> None:
    """The perturbed YAML files are valid building descriptions: trano generates their FastModel models."""
    case = CASES[case_id]
    for variant in case.variants:
        directory = tmp_path / variant
        model = network_for(variant_yaml(case, variant, directory), FAST_MODEL, directory).model(
            include_container=False
        )
        assert "FastModel.Zones." in model, variant


def test_the_base_cases_generate_the_buildings_model(tmp_path: Path) -> None:
    for case in CASES.values():
        directory = tmp_path / case.id
        model = network_for(variant_yaml(case, BASE, directory), BUILDINGS, directory).model(include_container=False)
        assert "Buildings.ThermalZones" in model or "Trano." in model, case.id


def test_the_night_ventilation_of_950ff_reaches_both_libraries(tmp_path: Path) -> None:
    case = CASES["950FF"]
    for library in (BUILDINGS, FAST_MODEL):
        directory = tmp_path / library
        model = network_for(variant_yaml(case, BASE, directory), library, directory).model(include_container=False)
        assert "ventilationSchedule=[0, 0.391389; 25200, 0.391389; 25200, 0; 64800, 0; 64800, 0.391389;" in model, (
            library
        )


def _series(offset: float = 0.0, heating_factor: float = 1.0) -> pd.DataFrame:
    hours = np.arange(48, dtype=float)
    heating = np.where(hours % 24 < 8, 2000.0, 500.0) * heating_factor
    return pd.DataFrame(
        {
            "TOut": np.full(48, 2.0),
            "zone_001.T": 20.0 + np.sin(hours / 24 * 2 * np.pi) + offset,
            "zone_001.heating": heating,
            "zone_001.cooling": np.zeros(48),
            "heating": heating,
            "cooling": np.zeros(48),
            "electricity": heating / 3,
        },
        index=pd.Index(hours * 3600, name="time"),
    )


def test_kpis_of_a_series() -> None:
    kpis = compute_kpis(CASES["940"], BASE, BUILDINGS, _series(), wall_time=12.0, model_hash="abc")
    assert zone_names(_series()) == ["zone_001"]
    assert kpis.zone("zone_001").mean_temperature == pytest.approx(20.0)
    assert kpis.zone("zone_001").max_temperature == pytest.approx(21.0, abs=0.01)
    assert kpis.heating_kwh == pytest.approx((16 * 2000 + 32 * 500) / 1000)
    assert kpis.peak_heating_kw == 2.0
    assert kpis.cooling_kwh == 0.0
    assert kpis.electricity_kwh == pytest.approx(kpis.heating_kwh / 3)
    assert kpis.fuel_kwh is None
    assert kpis.value("mean_temperature") == pytest.approx(20.0)
    assert kpis.value("heating_kwh") == pytest.approx(kpis.heating_kwh)


def test_fidelity_of_a_shifted_series() -> None:
    [fidelity] = compare_series(_series(), _series(offset=0.5, heating_factor=1.1))
    assert fidelity.zone == "zone_001"
    assert fidelity.rmse == pytest.approx(0.5)
    assert fidelity.bias == pytest.approx(0.5)
    assert fidelity.max_abs == pytest.approx(0.5)
    assert fidelity.heating_deviation == pytest.approx(0.1)
    assert fidelity.heating_nmbe == pytest.approx(0.1)
    assert fidelity.heating_cv_rmse is not None and fidelity.heating_cv_rmse > 0.1


def test_the_plan_lists_every_variant() -> None:
    rows = plan()
    assert len(rows) == sum(len(case.variants) for case in CASES.values())
    assert plan(["950FF"], ["base"]) == [{"case": "950FF", "variant": "base"}]
    assert {row["variant"] for row in plan(["residential_after"], ["base", "eps_x1.5"])} == {"base", "eps_x1.5"}


def test_a_commit_message_selects_the_runs() -> None:
    from validation.fast_model.__main__ import plan_from_message

    assert plan_from_message("no tag") == []
    assert len(plan_from_message("Fix [study]")) == len(plan())
    assert plan_from_message("Retry [study 960 residential_after | base]") == [
        {"case": "960", "variant": "base"},
        {"case": "residential_after", "variant": "base"},
    ]
    assert {row["case"] for row in plan_from_message("[study 940]")} == {"940"}


def test_the_verdict_takes_the_ratio_or_the_difference_of_the_changes() -> None:
    from validation.fast_model.report import agreement

    def row(unit: str, base: float, buildings: float, fast_model: float) -> dict[str, object]:
        return {
            "unit": unit,
            "buildings_base": base,
            "delta_buildings": buildings,
            "delta_fast_model": fast_model,
            "ratio": fast_model / buildings,
        }

    assert agreement(row("degC", 20.0, 0.10, 0.13)) == 0  # a ratio of 1.3 on a tenth of a kelvin: 0.03 K apart
    assert agreement(row("degC", 20.0, 1.0, 1.08)) == 0
    assert agreement(row("degC", 20.0, 1.0, 1.2)) == 1
    assert agreement(row("kWh", 1000.0, 100.0, 120.0)) == 1  # ratio 1.2, 2 % of the base case apart
    assert agreement(row("kWh", 1000.0, 100.0, 150.0)) == 2
    assert agreement(row("kWh", 1000.0, 20.0, 28.0)) == 1  # ratio 1.4, 0.8 % of the base case apart
