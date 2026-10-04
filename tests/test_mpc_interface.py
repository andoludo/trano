"""The ``mpc`` library output can be plugged into an external MPC runtime.

A runtime receives the Modelica file and the interface JSON written by
``trano create-model house.yaml mpc``. ``ReferenceRuntime`` below is a minimal consumer
that never imports trano internals: it only relies on the interface contract.
"""

import shutil
from pathlib import Path

import casadi as ca
import numpy as np
import pytest
import rumoca
from typer.testing import CliRunner

from trano.data_models.conversion import convert_network
from trano.elements.library.library import Library
from trano.main import app
from trano.mpc import InputRole, MPCModelInterface, RCModelType, network_interface

MODELS = Path(__file__).parent / "models"
THREE_ZONES = MODELS / "three_zones_mpc.yaml"
KELVIN = 273.15


def mpc_network(model_type: RCModelType = RCModelType.r3c2, path: Path = THREE_ZONES):  # noqa: ANN201
    library = Library.from_configuration("mpc").model_copy(update={"rc_model_type": model_type})
    return convert_network("house", path, library=library)


@pytest.mark.parametrize("model_type", list(RCModelType))
def test_interface_matches_the_casadi_export(model_type: RCModelType) -> None:
    network = mpc_network(model_type)
    interface = network_interface(network)
    export = rumoca.Session().loads(network.model(), model=interface.model).to_casadi()
    assert interface.state_names == list(export.state_names)
    assert interface.input_names == list(export.input_names)
    assert interface.parameter_names == list(export.parameter_names)
    assert interface.parameter_values == pytest.approx(list(export.default_parameters), rel=1e-9)
    assert interface.initial_state == pytest.approx(list(export.default_states))


def test_interface_describes_controls_and_disturbance_sources() -> None:
    interface = network_interface(mpc_network())
    assert interface.model == "house.building_mpc"
    assert interface.runnable_model == "house.building"
    assert interface.weather_file == "../tests/resources/BEL_VLG_Uccle.064470_TMYx.2007-2021.mos"
    assert [control.name for control in interface.controls] == [f"space_00{i}_QHea" for i in (1, 2, 3)]
    sources = {signal.name: signal.source for signal in interface.disturbances}
    assert sources["TOut"].kind == "weather"  # type: ignore[union-attr]
    south = sources["HSol_azi0_til90"]
    assert south.kind == "irradiance"  # type: ignore[union-attr]
    assert (south.azimuth, south.tilt) == (0, 90)  # type: ignore[union-attr]
    occupancy = sources["space_002_QInt"]
    assert occupancy.kind == "occupancy"  # type: ignore[union-attr]
    assert occupancy.floor_area == 70  # type: ignore[union-attr]
    assert occupancy.model == "SimpleOccupancy"  # type: ignore[union-attr]
    zone = interface.zones[0]
    assert zone.indoor_temperature == "space_001_Ti"
    assert zone.design_heating_power is not None
    assert zone.design_heating_power > 0


def test_cli_writes_the_interface(tmp_path: Path) -> None:
    model_path = tmp_path / "house.yaml"
    shutil.copy(THREE_ZONES, model_path)
    result = CliRunner().invoke(app, ["create-model", str(model_path), "mpc"])
    assert result.exit_code == 0, result.output
    interface = MPCModelInterface.read(tmp_path / "house.mpc.json")
    assert interface.model == "house.building_mpc"
    assert (tmp_path / "house.mo").exists()


class ReferenceRuntime:
    """Minimal MPC runtime built from the Modelica file and the interface only."""

    def __init__(self, modelica: str, interface: MPCModelInterface, time_step: float, substeps: int = 4) -> None:
        self.interface = interface
        export = rumoca.Session().loads(modelica, model=interface.model).to_casadi()
        controls = [i for i, signal in enumerate(interface.inputs) if signal.role == InputRole.control]
        disturbances = [i for i, signal in enumerate(interface.inputs) if signal.role == InputRole.disturbance]
        x = ca.MX.sym("x", len(interface.states))
        u = ca.MX.sym("u", len(controls))
        d = ca.MX.sym("d", len(disturbances))
        p = ca.DM(interface.parameter_values)
        signals = [None] * len(interface.inputs)
        for vector, indices in ((u, controls), (d, disturbances)):
            for position, index in enumerate(indices):
                signals[index] = vector[position]
        f = ca.Function("f", [x, u, d], [export.rhs(0, x, ca.vertcat(*signals), p)])
        h, x_next = time_step / substeps, x
        for _ in range(substeps):
            k1 = f(x_next, u, d)
            k2 = f(x_next + h / 2 * k1, u, d)
            k3 = f(x_next + h / 2 * k2, u, d)
            k4 = f(x_next + h * k3, u, d)
            x_next = x_next + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        self.step = ca.Function("F", [x, u, d], [x_next])
        self.time_step = time_step
        self.indoor = [interface.state_names.index(zone.indoor_temperature) for zone in interface.zones]

    def solve(self, disturbances: np.ndarray, lower: np.ndarray, price: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        horizon = disturbances.shape[1]
        n_zones = len(self.interface.zones)
        max_power = np.array([zone.design_heating_power for zone in self.interface.zones])
        opti = ca.Opti()
        states = opti.variable(len(self.interface.states), horizon + 1)
        power = opti.variable(n_zones, horizon)
        slack = opti.variable(n_zones, horizon)
        opti.subject_to(states[:, 0] == self.interface.initial_state)
        for k in range(horizon):
            opti.subject_to(states[:, k + 1] == self.step(states[:, k], power[:, k], disturbances[:, k]))
            for z, index in enumerate(self.indoor):
                opti.subject_to(states[index, k + 1] + slack[z, k] >= lower[k])
        opti.subject_to(opti.bounded(0, ca.vec(power), np.tile(max_power, horizon)))
        opti.subject_to(ca.vec(slack) >= 0)
        energy = ca.sum2(ca.DM(price).T * ca.sum1(power)) * self.time_step / 3.6e6
        opti.minimize(energy + 100 * ca.sum1(ca.sum2(slack)) + 10 * ca.sumsqr(slack))
        opti.solver("ipopt", {"print_time": False}, {"print_level": 0, "sb": "yes"})
        solution = opti.solve()
        return np.array(solution.value(power)), np.array(solution.value(states))


def _forecast(interface: MPCModelInterface, horizon: int) -> np.ndarray:
    """Disturbance forecasts built from the interface sources (synthetic weather)."""
    hour = np.arange(horizon) % 24
    rows = []
    for signal in interface.disturbances:
        source = signal.source
        assert source is not None
        if source.kind == "weather":
            rows.append(KELVIN + 2 + 4 * np.sin(2 * np.pi * (hour - 9) / 24))
        elif source.kind == "irradiance":
            rows.append(np.clip(400 * np.sin(np.pi * (hour - 7) / 10), 0, None) * (hour <= 17))
        else:
            occupied = (hour >= 7) & (hour < 19)
            rows.append(np.where(occupied, 105 / 24 * source.floor_area, 0.0))
    return np.vstack(rows)


@pytest.mark.parametrize("model_type", list(RCModelType))
def test_reference_runtime_solves_an_mpc_problem(tmp_path: Path, model_type: RCModelType) -> None:
    network = mpc_network(model_type)
    modelica_path = tmp_path / "house.mo"
    modelica_path.write_text(network.model())
    interface_path = network_interface(network).write(tmp_path / "house.mpc.json")

    interface = MPCModelInterface.read(interface_path)
    runtime = ReferenceRuntime(modelica_path.read_text(), interface, time_step=3600)
    horizon = 24
    hour = np.arange(horizon)
    lower = np.where((hour >= 7) & (hour < 22), KELVIN + 20, KELVIN + 16)
    power, states = runtime.solve(_forecast(interface, horizon), lower, price=np.full(horizon, 0.25))

    assert power.shape == (len(interface.zones), horizon)
    assert np.all(power >= -1e-3)
    indoor = states[runtime.indoor]
    assert np.all(indoor[:, 9:] >= lower[8:] - 0.05)  # comfort met once the zones had time to heat up
    assert power.sum() > 0
