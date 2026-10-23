"""The residential examples as MPC models: rumoca exports, roles, and an optimal control solved by IPOPT.

For each house (``examples/residential_after_renovation.yaml``, heat pump; ``residential_before_renovation.yaml``, gas
boiler) the FastModelMPC model is exported to CasADi by rumoca and an optimal control of 12 hours is solved by IPOPT
(trano-optimization) on a cold, sunless night: 0 degC outside, every zone kept at or above 20 degC, energy at 0.25
per kWh. The closed-loop FastModel model is exported too, and its states are checked against those of the MPC model:
mpctrano starts the MPC model from the closed loop by copying the states that share a name.

Needs trano-optimization with rumoca >= 0.10.3 (not a dependency of trano).

    python -m analysis.fast_model.scripts.mpc_houses

Writes ``mpc_<house>.json``.
"""

from __future__ import annotations

import argparse
import math
import time
import warnings
from pathlib import Path
from typing import Any

import numpy as np

from analysis.fast_model.scripts._common import EXAMPLES, KELVIN, configure_logging, logger, write_json
from trano.data_models.conversion import convert_network
from trano.elements.library.library import Library

HOUSES = ("residential_after_renovation", "residential_before_renovation")
WEATHER = {
    "weather_inputs.TDryBul": KELVIN,
    "weather_inputs.TBlaSky": KELVIN - 10,
    "weather_inputs.winSpe": 3.0,
    "weather_inputs.winDir": math.pi,
    "weather_inputs.HDirNor": 0.0,
    "weather_inputs.HDifHor": 0.0,
    "weather_inputs.HGloHor": 0.0,
}
HOURS = 12
COMFORT = KELVIN + 20
PRICE = 0.25


def model_text(path: Path, library: str) -> str:
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        return convert_network(path.stem, path, library=Library.from_configuration(library)).model(
            include_container=True
        )


def export(path: Path, library: str) -> tuple[Any, float]:
    from trano_optimization import CasadiModel

    start = time.perf_counter()
    model = CasadiModel.from_modelica(model_text(path, library), f"{path.stem}.building_mpc")
    return model, time.perf_counter() - start


def shared_states(mpc: Any, closed_loop: Any) -> dict[str, Any]:  # noqa: ANN401
    """The wall states of the MPC model are states of the closed loop under the same names; the closed loop adds the
    nodes inside the layers (``TSub``)."""
    mpc_states, closed_states = set(mpc.description.state_names), set(closed_loop.description.state_names)
    walls = {name for name in mpc_states if ".T[" in name}
    return {
        "mpc_states": len(mpc_states),
        "closed_loop_states": len(closed_states),
        "mpc_wall_states": len(walls),
        "mpc_wall_states_missing_in_closed_loop": sorted(walls - closed_states),
        "closed_loop_inner_layer_states": sum(".TSub[" in name for name in closed_states),
        "mpc_states_missing_in_closed_loop": sorted(mpc_states - closed_states),
    }


def solve(model: Any) -> dict[str, Any]:  # noqa: ANN401
    from trano_optimization import OptimalControl, OptimalControlSettings

    ocp = OptimalControl(model, settings=OptimalControlSettings(time_step=3600))
    disturbances = np.array([[WEATHER.get(name, 0.0)] * HOURS for name in ocp.roles.disturbances])
    result = ocp.solve(
        model.description.initial_state, disturbances, np.full(HOURS, COMFORT), price=np.full(HOURS, PRICE)
    )
    names = model.description.state_names
    zones = {name: names.index(name) for name in ocp.roles.comfort if name in names}
    meters = {
        name: float(result.states[names.index(name), -1] - result.states[names.index(name), 0])
        for name in ocp.roles.meters
        if name in names
    }
    return {
        "roles": {
            "controls": list(ocp.roles.controls),
            "comfort": list(ocp.roles.comfort),
            "meters": ocp.roles.meters,
            "disturbances": len(ocp.roles.disturbances),
        },
        "status": result.status,
        "success": result.success,
        "solve_time_s": result.solve_time,
        "energy_cost": result.energy_cost,
        "comfort_slack_max_k": float(np.max(result.slack)) if result.slack.size else 0.0,
        "lowest_zone_temperature_after_3_h_degc": min(float(result.states[index, 3:].min()) for index in zones.values())
        - KELVIN,
        "metered_kwh": meters,
        "zone_temperatures_degc": {
            name: [round(float(value) - KELVIN, 3) for value in result.states[index]] for name, index in zones.items()
        },
        "controls": {
            name: [round(float(value), 4) for value in result.controls[row]]
            for row, name in enumerate(ocp.roles.controls)
        },
    }


def house(stem: str) -> dict[str, Any]:
    path = EXAMPLES / f"{stem}.yaml"
    mpc, mpc_time = export(path, "FastModelMPC")
    logger.info("%s: FastModelMPC exported in %.1f s", stem, mpc_time)
    closed_loop, closed_time = export(path, "FastModel")
    logger.info("%s: FastModel exported in %.1f s", stem, closed_time)
    return {
        "yaml": f"examples/{stem}.yaml",
        "scenario": {"hours": HOURS, "time_step_s": 3600, "comfort_degc": COMFORT - KELVIN, "price": PRICE}
        | {"weather": WEATHER},
        "mpc_export": {
            "seconds": mpc_time,
            "states": len(mpc.description.states),
            "inputs": len(mpc.description.input_names),
            "parameters": len(mpc.description.parameter_names),
        },
        "closed_loop_export": {
            "seconds": closed_time,
            "states": len(closed_loop.description.states),
            "inputs": len(closed_loop.description.input_names),
        },
        "state_names": shared_states(mpc, closed_loop),
        "optimal_control": solve(mpc),
    }


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("houses", nargs="*", choices=HOUSES, help="both by default")
    arguments = parser.parse_args(argv)
    configure_logging()
    for stem in arguments.houses or HOUSES:
        write_json(f"mpc_{stem}.json", house(stem))


if __name__ == "__main__":
    main()
