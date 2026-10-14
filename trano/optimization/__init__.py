"""Simulation, state estimation and optimal control of Modelica models with rumoca, CasADi and IPOPT.

A self-contained package: it depends on rumoca, CasADi, NumPy and pydantic only (no other module of trano), so
that it can be moved to a package of its own. It works on any Modelica model that rumoca translates into an
explicit ODE; the roles of the variables (controls, disturbances, comfort temperatures, energy meters) are read
from the model metadata (:class:`Roles`) or given explicitly.

    model = CasadiModel.from_modelica(source, "house.building_mpc")
    ocp = OptimalControl(model, settings=OptimalControlSettings(time_step=3600))
    result = ocp.solve(model.description.initial_state, disturbances, comfort_lower, price, start_time=t0)
"""

from .collocation import KELVIN, SOLVED, CollocationSettings, default_ipopt_options, linear_solver
from .control import (
    ClosedLoopResult,
    OptimalControl,
    OptimalControlResult,
    OptimalControlSettings,
    Plant,
    receding_horizon,
)
from .estimation import EstimationResult, EstimationSettings, MovingHorizonEstimation
from .model import CasadiModel, ModelDescription, Variable
from .roles import Carrier, Roles

__all__ = [
    "KELVIN",
    "SOLVED",
    "Carrier",
    "CasadiModel",
    "ClosedLoopResult",
    "CollocationSettings",
    "EstimationResult",
    "EstimationSettings",
    "ModelDescription",
    "MovingHorizonEstimation",
    "OptimalControl",
    "OptimalControlResult",
    "OptimalControlSettings",
    "Plant",
    "Roles",
    "Variable",
    "default_ipopt_options",
    "linear_solver",
    "receding_horizon",
]
