"""Options trading the fidelity of short transients for simulation speed.

Every option is on by default; ``PerformanceOptions.disabled()`` (or setting them to false under the
``performance`` key of the YAML) gives the detailed model, with the actuator and sensor dynamics of the
libraries. See the "Simulation performance" documentation page for the measurements behind each option.
"""

from pydantic import BaseModel, ConfigDict, Field


class PerformanceOptions(BaseModel):
    """Simplifications of the generated model, set under the ``performance`` key of the YAML (on by default)."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    ideal_actuators: bool = Field(
        default=True,
        description=(
            "Valves, dampers, pumps and fans follow their control signal instantly: the filter approximating "
            "the stroke time of actuators and the rise time of movers is removed. Removes two states per "
            "actuator and the fast closed-loop dynamics they create with the controllers."
        ),
    )
    steady_state_sensors: bool = Field(
        default=True,
        description=(
            "Two-port sensors of temperature and other fluid properties measure instantly (time constant 0) "
            "instead of through a first-order lag. At zero flow they report the incoming fluid, not the "
            "last value measured."
        ),
    )

    @classmethod
    def disabled(cls) -> "PerformanceOptions":
        """No simplification: the actuator and sensor dynamics of the libraries are kept."""
        return cls(ideal_actuators=False, steady_state_sensors=False)

    @property
    def enabled(self) -> bool:
        return self.ideal_actuators or self.steady_state_sensors
