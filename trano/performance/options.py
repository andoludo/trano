"""Options trading the fidelity of short transients for simulation speed.

Every option is off by default: a model generated without them is the model trano always generated.
See the "Simulation performance" documentation page for the measurements behind each option.
"""

from pydantic import BaseModel, ConfigDict, Field


class PerformanceOptions(BaseModel):
    """Opt-in simplifications of the generated model, set under the ``performance`` key of the YAML."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    ideal_actuators: bool = Field(
        default=False,
        description=(
            "Valves, dampers, pumps and fans follow their control signal instantly: the filter approximating "
            "the stroke time of actuators and the rise time of movers is removed. Removes two states per "
            "actuator and the fast closed-loop dynamics they create with the controllers."
        ),
    )
    steady_state_sensors: bool = Field(
        default=False,
        description=(
            "Two-port sensors of temperature and other fluid properties measure instantly (time constant 0) "
            "instead of through a first-order lag. At zero flow they report the incoming fluid, not the "
            "last value measured."
        ),
    )

    @property
    def enabled(self) -> bool:
        return self.ideal_actuators or self.steady_state_sensors
