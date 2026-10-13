"""Simplifications of the generated models that make them simulate faster, on by default.

``PerformanceOptions`` is read from the ``performance`` key of the YAML (or given to ``convert_network``);
``apply_performance_options`` rewrites the rendered model accordingly. With ``PerformanceOptions.disabled()``
the model is returned unchanged.
"""

from trano.performance.declarations import (
    ACTUATOR_FILTER,
    MOVER_FILTER,
    SENSOR_DYNAMICS,
    DeclarationRule,
    apply_rule,
)
from trano.performance.options import PerformanceOptions

__all__ = ["PerformanceOptions", "apply_performance_options", "rules"]


def rules(options: PerformanceOptions) -> list[DeclarationRule]:
    """The declaration rules the options call for."""
    selected: list[DeclarationRule] = []
    if options.ideal_actuators:
        selected += [ACTUATOR_FILTER, MOVER_FILTER]
    if options.steady_state_sensors:
        selected.append(SENSOR_DYNAMICS)
    return selected


def apply_performance_options(model: str, options: PerformanceOptions) -> str:
    for rule in rules(options):
        model, _ = apply_rule(model, rule)
    return model
