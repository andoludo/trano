"""PV arrays of the ``mpc`` library, after trano's Buildings PV (``PVSimpleOriented``).

``P = A*fAct*eta*eta_DCAC*(HDir + HDif)`` with the direct and Perez diffuse irradiance on the array, the inputs of
its orientation; ``Electrical.Photovoltaic`` integrates the production into a meter, which the optimal control
counts as electricity that is not bought.
"""

import math
from typing import TYPE_CHECKING

from trano.mpc.components.model import Instance
from trano.mpc.estimation import sanitize_name

if TYPE_CHECKING:
    from trano.elements.solar import Photovoltaic
    from trano.mpc.components.envelope import Orientation
    from trano.topology import Network

ACTIVE_AREA_FRACTION = 0.9  # fAct of Buildings PartialPvBase
INVERTER_EFFICIENCY = 0.9  # eta_DCAC of Buildings PartialAcDcParameters


def photovoltaic_arrays(network: "Network") -> list["Photovoltaic"]:
    from trano.elements.solar import Photovoltaic

    return sorted((node for node in network.graph.nodes if isinstance(node, Photovoltaic)), key=lambda pv: str(pv.name))


def array_orientation(array: "Photovoltaic") -> "Orientation":
    from trano.mpc.components.envelope import Orientation

    parameters = array.parameters
    azimuth = getattr(parameters, "azimuth_radians", None)
    tilt = getattr(parameters, "tilt_radians", None)
    return Orientation(
        azimuth=float(azimuth) if azimuth is not None else math.radians(float(parameters.azimuth)),  # type: ignore[union-attr]
        tilt=float(tilt) if tilt is not None else math.radians(float(parameters.tilt)),  # type: ignore[union-attr]
    )


def photovoltaic_instance(array: "Photovoltaic") -> Instance:
    parameters = array.parameters
    orientation = array_orientation(array)
    fraction = getattr(parameters, "active_area_fraction", None)
    inverter = getattr(parameters, "inverter_efficiency", None)
    return Instance(
        name=sanitize_name(str(array.name)),
        component="Electrical.Photovoltaic",
        description=f"PV array {array.name}",
        system=sanitize_name(str(array.name)),
        parameters={
            "A": float(parameters.area),  # type: ignore[union-attr]
            "eta": float(parameters.efficiency),  # type: ignore[union-attr]
            "fAct": float(fraction) if fraction is not None else ACTIVE_AREA_FRACTION,
            "eta_DCAC": float(inverter) if inverter is not None else INVERTER_EFFICIENCY,
        },
        inputs={"HDir": orientation.direct, "HDif": orientation.diffuse},
    )
