"""CasADi-compatible RC building models: the ``mpc`` Trano library.

Generate the model like with any other Trano library::

    trano create-model house.yaml mpc --rc-model-type R3C2

The generated package contains:

* ``Trano.MPC.Zones``: the single-zone RC models (R1C1, R3C2, R4C3, ISO13790),
* ``building_mpc``: the flat multi-zone RC model, directly translatable into a CasADi ODE
  (e.g. with rumoca) for model predictive control with IPOPT,
* ``building``: the runnable model connecting the weather file, the solar irradiance of each
  orientation, the occupancy and the external data to ``building_mpc``.
"""

from trano.mpc.building import Orientation, RCBuilding, RCZone, SolarAperture, ZoneCoupling
from trano.mpc.estimation import (
    EstimationSettings,
    estimate_zone_parameters,
    rc_building_from_network,
    rc_building_from_yaml,
    sanitize_name,
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
    "Orientation",
    "R1C1Parameters",
    "R3C2Parameters",
    "R4C3Parameters",
    "RCBuilding",
    "RCModelType",
    "RCZone",
    "SolarAperture",
    "ZoneCoupling",
    "estimate_zone_parameters",
    "rc_building_from_network",
    "rc_building_from_yaml",
    "sanitize_name",
    "zone_envelope",
]
