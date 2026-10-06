"""CasADi-compatible RC building models: the ``mpc`` Trano library.

Generate the model like with any other Trano library::

    trano create-model house.yaml mpc --rc-model-type R3C2

The generated package contains:

* ``Trano.MPC.Zones``: the single-zone RC models (R1C1, R3C2, R4C3, ISO13790),
* ``building_mpc``: the flat multi-zone RC model, directly translatable into a CasADi ODE
  (e.g. with rumoca) for model predictive control with IPOPT,
* ``building``: the runnable model connecting the weather file, the solar irradiance of each
  orientation, the occupancy and the external data to ``building_mpc``.

``trano create-model house.yaml mpc`` also writes ``house.mpc.json``: the :class:`MPCModelInterface`
describing the states, inputs (controls and disturbances with their origin) and parameters of
``building_mpc`` in the order of the CasADi vectors, to plug the model into an MPC runtime.
"""

from trano.mpc.building import Orientation, RCBuilding, RCZone, SolarAperture, ZoneCoupling
from trano.mpc.estimation import (
    EstimationSettings,
    estimate_zone_parameters,
    rc_building_from_network,
    rc_building_from_yaml,
    sanitize_name,
    systems_from_network,
    zone_envelope,
)
from trano.mpc.interface import (
    AnySystemSpec,
    BatterySpec,
    BoilerSpec,
    Carrier,
    ChillerSpec,
    EVChargerSpec,
    HeatPumpSpec,
    InputRole,
    InputSpec,
    MPCModelInterface,
    ParameterSpec,
    PhotovoltaicSpec,
    StateSpec,
    StorageTankSpec,
    ZoneSpec,
)
from trano.mpc.modelica import build_interface, network_interface
from trano.mpc.parameters import (
    ISO13790Parameters,
    R1C1Parameters,
    R3C2Parameters,
    R4C3Parameters,
    RCModelType,
)
from trano.mpc.systems import (
    AnySystem,
    Battery,
    Chiller,
    DHWTank,
    DrawOffProfile,
    EVCharger,
    EVSessions,
    GasBoiler,
    HeatPump,
    Photovoltaic,
    SystemKind,
)

__all__ = [
    "AnySystem",
    "AnySystemSpec",
    "Battery",
    "BatterySpec",
    "BoilerSpec",
    "Carrier",
    "Chiller",
    "ChillerSpec",
    "DHWTank",
    "DrawOffProfile",
    "EVCharger",
    "EVChargerSpec",
    "EVSessions",
    "EstimationSettings",
    "GasBoiler",
    "HeatPump",
    "HeatPumpSpec",
    "ISO13790Parameters",
    "InputRole",
    "InputSpec",
    "MPCModelInterface",
    "Orientation",
    "ParameterSpec",
    "Photovoltaic",
    "PhotovoltaicSpec",
    "R1C1Parameters",
    "R3C2Parameters",
    "R4C3Parameters",
    "RCBuilding",
    "RCModelType",
    "RCZone",
    "SolarAperture",
    "StateSpec",
    "StorageTankSpec",
    "SystemKind",
    "ZoneCoupling",
    "ZoneSpec",
    "build_interface",
    "estimate_zone_parameters",
    "network_interface",
    "rc_building_from_network",
    "rc_building_from_yaml",
    "sanitize_name",
    "systems_from_network",
    "zone_envelope",
]
