"""CasADi-compatible RC building models and model predictive control.

Typical workflow::

    from trano.mpc import RCModelType, rc_building_from_yaml
    from trano.mpc.controller import Forecast, ModelPredictiveController

    building = rc_building_from_yaml("house.yaml", model_type=RCModelType.r3c2)
    modelica_source = building.to_modelica()  # stand-alone Modelica package
    model = building.to_casadi()  # Modelica -> CasADi (via rumoca)
    controller = ModelPredictiveController(model)
    solution = controller.solve(forecast)

The CasADi dependent parts need the optional dependencies: ``pip install 'trano[mpc]'``.
"""

from trano.mpc.building import RCBuilding, RCZone, ZoneCoupling
from trano.mpc.estimation import (
    EstimationSettings,
    estimate_zone_parameters,
    rc_building_from_network,
    rc_building_from_yaml,
    zone_envelope,
)
from trano.mpc.parameters import (
    ISO13790Parameters,
    R1C1Parameters,
    R3C2Parameters,
    R4C3Parameters,
    RCModelType,
)

__all__ = [
    "EstimationSettings",
    "ISO13790Parameters",
    "R1C1Parameters",
    "R3C2Parameters",
    "R4C3Parameters",
    "RCBuilding",
    "RCModelType",
    "RCZone",
    "ZoneCoupling",
    "estimate_zone_parameters",
    "rc_building_from_network",
    "rc_building_from_yaml",
    "zone_envelope",
]
