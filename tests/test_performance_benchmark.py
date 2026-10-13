"""The performance benchmark tooling: case selection, log parsing, signal selection and report (no Docker)."""

import re

import pytest

from tests.golden import remove_trano_package
from trano.data_models.conversion import convert_network
from trano.performance import PerformanceOptions
from validation.performance.accuracy import Comparison, SignalComparison, select_signals
from validation.performance.cases import MODELS, PROFILES, select_cases
from validation.performance.report import accuracy_table, timing_table
from validation.performance.runner import (
    CaseRun,
    SolverStatistics,
    TranslationStatistics,
    parse_solver,
    parse_translation,
)

BUILD_LOG = """
Notification: Model statistics after passing the back-end for initialization:
 * Number of states: 0 ('-d=stateselection' for list of states)
Notification: Model statistics after passing the back-end for simulation:
 * Number of states: 60 ('-d=stateselection' for list of states)
Notification: Strong component statistics for simulation (1020):
 * Single equations (assignments): 950
Notification: Torn system details for strict tearing set:
 * Linear torn systems (#iteration vars, #inner vars, density): 3 systems
   {(1,4,100.0%), (13,63,66.9%), (3,9,100.0%)}
 * Non-linear torn systems (#iteration vars, #inner vars): 2 systems
   {(1,3), (3,17)}
"""

SIMULATION_LOG = """
LOG_STATS         | info    | ### STATISTICS ###
|                 | |       | | events
|                 | |       | | |   336 state events
|                 | |       | | |     0 time events
|                 | |       | | solver: dassl
|                 | |       | | | 22122 steps taken
|                 | |       | | | 35157 calls of functionODE
|                 | |       | | |  8440 evaluations of jacobian
|                 | |       | | |  1352 error test failures
|                 | |       | | |   669 convergence test failures
|                 | |       | | | 9.86084s time of jacobian evaluation
"""


def test_translation_statistics_are_those_of_the_simulation_system() -> None:
    assert parse_translation(BUILD_LOG) == TranslationStatistics(
        states=60, linear_systems=[13, 3, 1], nonlinear_systems=[3, 1]
    )


def test_solver_statistics() -> None:
    assert parse_solver(SIMULATION_LOG) == SolverStatistics(
        steps=22122,
        rhs_evaluations=35157,
        jacobian_evaluations=8440,
        error_test_failures=1352,
        convergence_test_failures=669,
        state_events=336,
        time_events=0,
        jacobian_time=9.86084,
    )


def test_missing_statistics_stay_unknown() -> None:
    assert parse_solver("") == SolverStatistics()
    assert parse_translation("") == TranslationStatistics()


def test_default_selection_leaves_out_slow_models_and_optimizes_hydronic_models_only() -> None:
    cases = select_cases(None, None, None)
    assert {case.model.name for case in cases} == {model.name for model in MODELS if not model.slow}
    assert {case.profile.name for case in cases} == {"baseline", "fast"}
    assert all(case.profile.name == "baseline" for case in cases if case.model.envelope_only)


def test_unknown_model_is_refused() -> None:
    with pytest.raises(ValueError, match="Unknown models"):
        select_cases(["nope"], None, None)


def test_hysteresis_profile_sets_the_boiler_controls_only() -> None:
    (case,) = select_cases(["three_zones_hydronic"], ["Buildings"], ["hysteresis"])
    assert case.profile == PROFILES["hysteresis"]
    assert re.search(r"dTHys=1\.0", case.network().model())
    (heat_pump,) = select_cases(["residential_after_renovation"], ["Buildings"], ["hysteresis"])
    assert "dTHys=" not in remove_trano_package(heat_pump.network().model())


def test_profiles_use_the_performance_options() -> None:
    (case,) = select_cases(["single_zone_hydronic"], ["IDEAS"], ["fast"])
    network = case.network()
    assert network.performance == PerformanceOptions(ideal_actuators=True, steady_state_sensors=True)


def test_signals_are_selected_by_kind() -> None:
    names = [
        "building.envelope1.space_001.heaPorAir.T",
        "building.envelope1.living_001.TSensor",
        "building.envelope1.space_001.heaPorAir.Q_flow",
        "building.distribution1.pump_001.temSup.T",
        "building.production1.generator_001.retTemp.T",
        "building.emission1.radiator_003.Q_flow",
        "building.production1.boiler_001.boi.Q_flow",
        "building.production1.generator_001.airToWaterTableData2D.QCon_flow",
    ]
    assert select_signals(names) == {
        "zone_temperature": ["building.envelope1.living_001.TSensor", "building.envelope1.space_001.heaPorAir.T"],
        "supply_temperature": ["building.distribution1.pump_001.temSup.T"],
        "return_temperature": ["building.production1.generator_001.retTemp.T"],
        "heating_power": [
            "building.emission1.radiator_003.Q_flow",
            "building.production1.boiler_001.boi.Q_flow",
            "building.production1.generator_001.airToWaterTableData2D.QCon_flow",
        ],
    }


def _signal(kind: str, name: str, rmse: float, maximum: float, mean: tuple[float, float]) -> SignalComparison:
    return SignalComparison(
        kind=kind, name=name, rmse=rmse, max_deviation=maximum, mean_baseline=mean[0], mean_case=mean[1]
    )


def test_deviation_beyond_tolerance_is_flagged() -> None:
    zone = "building.envelope1.space_001.heaPorAir.T"
    radiator = "building.emission1.radiator_001.Q_flow"
    within = Comparison(
        case_id="a",
        signals=[
            _signal("zone_temperature", zone, 0.05, 0.3, (293, 293)),
            _signal("heating_power", radiator, 10, 300, (-1000, -1010)),
        ],
    )
    beyond = Comparison(case_id="b", signals=[_signal("zone_temperature", zone, 0.3, 3.0, (293, 293))])
    assert within.within_tolerance
    assert within.heating_energy_relative() == pytest.approx(0.01)
    assert not beyond.within_tolerance
    table = accuracy_table([within, beyond])
    assert "| a | 0.050 | 0.30 | - | - | +1.0% | yes |" in table
    assert "| b | 0.300 | 3.00 | - | - | - | **no** |" in table


def test_timing_table_reports_the_speed_up_against_the_baseline() -> None:
    baseline_case, fast_case = select_cases(["single_zone_hydronic"], ["Buildings"], ["baseline", "fast"])
    runs = [
        CaseRun(case_id=baseline_case.id, model_hash="x", success=True, translation_time=30, simulation_time=16),
        CaseRun(case_id=fast_case.id, model_hash="y", success=True, translation_time=30, simulation_time=8),
    ]
    table = timing_table([baseline_case, fast_case], runs)
    assert "| single_zone_hydronic | Buildings | fast | 30.0 | 8.0 | 2.00x |" in table


def test_boiler_loop_sensors_are_oriented_with_the_flow() -> None:
    """The sensors of the boiler with storage are connected port_a upstream (they read negative flows before)."""
    model = convert_network("single_zone_hydronic", MODELS[1].path).model()
    package = model[model.index("partial model PartialBoilerWithStorage") : model.index("end PartialBoilerWithStorage")]
    for connection in [
        "connect(port_a, senMasFlo4.port_a)",
        "connect(temperature_sensor2.port_a, senMasFlo4.port_b)",
        "connect(splVal3.port_2, temperature_sensor2.port_b)",
        "connect(splVal3.port_1, temperature_sensor4.port_a)",
        "connect(temperature_sensor4.port_b, senMasFlo2.port_a)",
        "connect(senMasFlo1.port_a, splVal4.port_2)",
        "connect(senMasFlo1.port_b, port_b)",
    ]:
        assert connection in package


def test_energy_tolerance_applies_to_the_total_not_to_each_emitter() -> None:
    zone = _signal("zone_temperature", "building.envelope1.space_001.heaPorAir.T", 0.01, 0.1, (293, 293))
    large = _signal("heating_power", "building.emission1.radiator_001.Q_flow", 5, 50, (-1000, -1000))
    small = _signal("heating_power", "building.emission1.radiator_002.Q_flow", 1, 10, (-6.0, -5.6))  # -7 %
    assert Comparison(case_id="a", signals=[zone, large, small]).within_tolerance
    drifting = _signal("heating_power", "building.emission1.radiator_001.Q_flow", 50, 500, (-1000, -950))
    assert not Comparison(case_id="b", signals=[zone, drifting]).within_tolerance
