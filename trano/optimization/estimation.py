"""Moving horizon estimation: the trajectory of the model closest to the measurements over a past window."""

import time
from dataclasses import dataclass

import casadi as ca
import numpy as np
from pydantic import Field

from .collocation import SOLVED, Collocated, CollocationSettings
from .model import CasadiModel
from .roles import Roles


class EstimationSettings(CollocationSettings):
    measurement_noise: float = Field(0.1, gt=0, description="Standard deviation of a measurement [K]")
    process_noise: float = Field(0.05, gt=0, description="Standard deviation of the model error over a step [K]")
    arrival_noise: float = Field(2.0, gt=0, description="Standard deviation of the prior of the first state [K]")


@dataclass(frozen=True)
class EstimationResult:
    states: np.ndarray  # (states, N + 1) over the window; the last column is the current estimate
    process_noise: np.ndarray  # (states, N) estimated model errors
    status: str
    solve_time: float

    @property
    def current(self) -> np.ndarray:
        return self.states[:, -1]

    @property
    def success(self) -> bool:
        return self.status in SOLVED


class MovingHorizonEstimation(Collocated):
    """State estimation over a past window.

    Minimizes, over the states of the window, the weighted squares of the measurement errors, of the model errors
    over each step (process noise added to the end state of the step) and of the deviation of the first state
    from its prior (arrival cost). The unmeasured states (walls, water, buffers) follow from the dynamics.
    """

    settings: EstimationSettings

    def __init__(
        self,
        model: CasadiModel,
        roles: Roles | None = None,
        settings: EstimationSettings | None = None,
        parameters: np.ndarray | None = None,
    ) -> None:
        super().__init__(model, roles, settings or EstimationSettings(), parameters)

    def estimate(  # noqa: PLR0913
        self,
        prior: np.ndarray,
        controls: np.ndarray,
        disturbances: np.ndarray,
        measurements: np.ndarray,
        start_time: float = 0.0,
        measured: list[str] | None = None,
    ) -> EstimationResult:
        """States over the window of ``controls`` (controls, N) as applied and ``disturbances`` (disturbances, N).

        ``measurements`` (measured states, N + 1) are the values of the states ``measured`` (the comfort
        temperatures by default) at the step boundaries; NaN marks a missing measurement.
        """
        settings = self.settings
        rows = [self.description.state_names.index(name) for name in (measured or self.roles.comfort)]
        n_x, horizon, degree = self.n_states, controls.shape[1], settings.degree
        opti = ca.Opti()
        z = opti.variable(n_x, horizon + 1)
        zc = opti.variable(n_x * degree, horizon)
        noise = opti.variable(n_x, horizon)
        residuals, ends = self.steps(z, zc, ca.DM(self.scaled(controls)), disturbances, start_time)
        opti.subject_to(ca.vec(residuals) == 0)
        opti.subject_to(z[:, 1:] == ends + noise)
        self.bound_temperatures(opti, z)

        offsets = self.offset[rows][:, None]
        observed = ~np.isnan(measurements)
        errors = (z[rows, :] - ca.DM(np.where(observed, measurements - offsets, 0.0))) * ca.DM(observed.astype(float))
        objective = ca.sumsqr(errors) / settings.measurement_noise**2
        objective += ca.sumsqr(noise) / settings.process_noise**2
        objective += ca.sumsqr(z[:, 0] - (np.asarray(prior, dtype=float) - self.offset)) / settings.arrival_noise**2
        opti.minimize(objective)

        guess = self.simulate(prior, controls, disturbances, start_time) - self.offset[:, None]
        opti.set_initial(z, guess)
        opti.set_initial(zc, np.tile(guess[:, 1:], (degree, 1)))
        started = time.monotonic()
        values, status = self.solve_nlp(opti)
        return EstimationResult(
            states=np.atleast_2d(values(z)).reshape(n_x, horizon + 1) + self.offset[:, None],
            process_noise=np.atleast_2d(values(noise)).reshape(n_x, horizon),
            status=status,
            solve_time=time.monotonic() - started,
        )
