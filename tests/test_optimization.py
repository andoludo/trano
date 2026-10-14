"""The standalone optimization package on a small Modelica model, independent of the rest of trano."""

import numpy as np
import pytest

from trano.optimization import (
    KELVIN,
    CasadiModel,
    MovingHorizonEstimation,
    OptimalControl,
    OptimalControlSettings,
    Roles,
    receding_horizon,
)

HOUSE = """
package House
  connector RealInput = input Real;
  model Room "Room heated by a boiler with a fuel meter and an ideal heater, sun through the window"
    RealInput TOut(unit="K") "Outdoor temperature";
    RealInput HGloHor(unit="W/m2") "Global horizontal irradiance";
    RealInput y(min=0, max=1, unit="1") "Boiler part load ratio";
    RealInput QHea(min=0, max=2000, unit="W") "Ideal heater";
    parameter Real C(unit="J/K") = 5e6 "Heat capacity of the room";
    parameter Real UA(unit="W/K") = 100 "Heat loss coefficient";
    parameter Real QNom(unit="W") = 5000 "Boiler power";
    parameter Real eta = 0.9 "Boiler efficiency";
    parameter Real gA(unit="m2") = 2 "Solar aperture";
    Real TAir(start=291.15, fixed=true, unit="K") "Zone air temperature";
    Real EFue(start=0, fixed=true, unit="kWh") "Fuel meter";
  equation
    C*der(TAir) = UA*(TOut - TAir) + y*QNom + QHea + gA*HGloHor*(0.5 + 0.5*cos(2*3.141592653589793*time/86400));
    der(EFue) = y*QNom/eta/3.6e6;
  end Room;
end House;
"""


@pytest.fixture(scope="module")
def room() -> CasadiModel:
    return CasadiModel.from_modelica(HOUSE, "House.Room")


def test_the_description_comes_from_the_modelica_model(room: CasadiModel) -> None:
    description = room.description
    assert description.state_names == ["TAir", "EFue"]
    assert description.input_names == ["TOut", "HGloHor", "y", "QHea"]
    assert description.input("y").maximum == 1
    assert description.state("EFue").unit == "kWh"
    assert description.initial_state == pytest.approx([291.15, 0.0])
    assert dict(zip(description.parameter_names, description.parameter_values, strict=True))["UA"] == 100


def test_roles_are_inferred_from_the_metadata(room: CasadiModel) -> None:
    roles = Roles.infer(room.description)
    assert roles.controls == ("y", "QHea")
    assert roles.disturbances == ("TOut", "HGloHor")
    assert roles.comfort == ("TAir",)
    assert roles.meters == {"EFue": "fuel"}
    with pytest.raises(ValueError, match="without a role"):
        Roles(controls=("y",), disturbances=("TOut",), comfort=("TAir",)).check(room.description)


def test_the_model_depends_on_time(room: CasadiModel) -> None:
    x, u = room.description.initial_state, np.array([273.15, 500.0, 0.0, 0.0])
    assert room.derivatives(0.0, x, u)[0] > room.derivatives(43200.0, x, u)[0]


def _forecast(hours: int) -> np.ndarray:
    return np.vstack([np.full(hours, KELVIN), np.full(hours, 100.0)])


def test_the_optimal_control_meets_comfort_at_least_cost(room: CasadiModel) -> None:
    ocp = OptimalControl(room, settings=OptimalControlSettings(time_step=3600))
    hours = 12
    comfort = np.where(np.arange(hours) >= 6, KELVIN + 20, KELVIN + 15)
    result = ocp.solve(
        room.description.initial_state,
        _forecast(hours),
        comfort,
        price=np.full(hours, 0.3),
        fuel_price=np.full(hours, 0.1),
    )
    assert result.success, result.status
    assert result.slack.max() < 0.05
    assert result.states[0, 7:].min() > KELVIN + 20 - 0.05
    y, heater = result.controls
    assert y.sum() > 0.5
    assert heater.max() < 50  # fuel at 0.1/0.9 is cheaper than electric heat at 0.3: the boiler heats
    simulated = ocp.simulate(room.description.initial_state, result.controls, _forecast(hours))
    assert simulated[0] == pytest.approx(result.states[0], abs=0.05)


def test_estimation_and_receding_horizon(room: CasadiModel) -> None:
    roles = Roles.infer(room.description)
    hours = 8
    controls = np.vstack([np.full(hours, 0.5), np.zeros(hours)])
    truth = OptimalControl(room, roles).simulate(np.array([292.0, 0.0]), controls, _forecast(hours))
    estimate = MovingHorizonEstimation(room, roles).estimate(
        np.array([288.0, 0.0]), controls, _forecast(hours), truth[:1]
    )
    assert estimate.success
    assert estimate.current[0] == pytest.approx(truth[0, -1], abs=0.1)

    ocp = OptimalControl(room, roles, OptimalControlSettings(time_step=3600))
    loop = receding_horizon(
        ocp, room.description.initial_state, _forecast(8), np.full(8, KELVIN + 19), np.full(8, 0.3), horizon=4
    )
    assert all(status in ("Solve_Succeeded", "Solved_To_Acceptable_Level") for status in loop.statuses)
    assert loop.states[0, -1] > KELVIN + 19 - 0.1
