"""Interface of a ``FastModel`` model: the ``mpc`` interface with the inputs of the multilayer zones.

The file format is the one of :mod:`trano.mpc.interface` (``<package>.mpc.json``), so that an MPC runtime reads both
libraries alike. ``FastModel`` adds two things to the inputs:

* the parts of the solar irradiance on an orientation (``component``): the multilayer zones take the beam and Perez
  diffuse irradiance and the cosine of the incidence angle, not their total;
* the bounds of a control (``minimum``, ``maximum``): branch flows, valve positions, part load ratios, fan speeds.
"""

from pathlib import Path
from typing import Annotated, Literal

from pydantic import Field

from trano.mpc import interface as mpc

IrradianceComponent = Literal["total", "direct", "diffuse_perez", "incidence_cosine"]


class IrradianceSource(mpc.IrradianceSource):
    """Solar irradiance on a tilted surface: total (direct + isotropic diffuse) by default, or one of its parts.

    ``direct`` is the beam irradiance on the surface, ``diffuse_perez`` the Perez diffuse irradiance on the surface
    and ``incidence_cosine`` the cosine of the incidence angle of the beam.
    """

    component: IrradianceComponent = "total"


DisturbanceSource = Annotated[
    mpc.OutdoorTemperatureSource
    | IrradianceSource
    | mpc.OccupancySource
    | mpc.PhotovoltaicSource
    | mpc.DrawOffSource
    | mpc.DrivingSource,
    Field(discriminator="kind"),
]


class InputSpec(mpc.InputSpec):
    source: DisturbanceSource | None = Field(None, description="Origin of a disturbance, None for controls")
    minimum: float | None = Field(None, description="Lower bound of a control, if it has one")
    maximum: float | None = Field(None, description="Upper bound of a control, if it has one")


class FastModelInterface(mpc.MPCModelInterface):
    """Description of ``<package>.building_mpc`` of the ``FastModel`` library for an MPC runtime."""

    # The inputs carry the FastModel specs; a list is invariant, hence the ignores on the overrides.
    inputs: list[InputSpec]  # type: ignore[assignment]

    @property
    def controls(self) -> list[InputSpec]:  # type: ignore[override]
        return [signal for signal in self.inputs if signal.role == mpc.InputRole.control]

    @property
    def disturbances(self) -> list[InputSpec]:  # type: ignore[override]
        return [signal for signal in self.inputs if signal.role == mpc.InputRole.disturbance]

    @classmethod
    def read(cls, path: Path | str) -> "FastModelInterface":
        return cls.model_validate_json(Path(path).read_text())
