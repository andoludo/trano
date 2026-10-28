from collections.abc import Sequence

# the simulation errors, defined with their backends and re-exported here
from trano_simulate.exceptions import DockerClientError, DockerNotInstalledError, OpenModelicaNotFoundError

__all__ = [
    "ConnectionLimitReached",
    "ContainerNotFoundError",
    "ControllerBusPortError",
    "DockerClientError",
    "DockerNotInstalledError",
    "IncompatiblePortsError",
    "InvalidBuildingStructureError",
    "InvalidModelError",
    "InvalidModelNameError",
    "InvalidSensorInletError",
    "NoConnectionFoundError",
    "OpenModelicaNotFoundError",
    "SystemsNotConnectedError",
    "UnknownComponentVariantError",
    "UnknownLibraryError",
    "WrongSystemFlowError",
]


class WrongSystemFlowError(Exception):
    """Exception raised for wrong system flow."""


class ControllerBusPortError(Exception):
    """Exception raised for controller bus port error."""


class IncompatiblePortsError(Exception): ...


class ConnectionLimitReached(Exception): ...  # noqa: N818


class NoConnectionFoundError(Exception): ...


class ContainerNotFoundError(Exception): ...


class SystemsNotConnectedError(Exception): ...


class UnknownLibraryError(Exception): ...


class UnknownComponentVariantError(Exception): ...


class InvalidBuildingStructureError(Exception): ...


class InvalidSensorInletError(Exception): ...


class InvalidModelNameError(ValueError):
    """The name of the model is the name of a class its generated package declares (``building``, ``Components``...).

    Inside the package, ``<name>.Components.BaseClasses...`` would resolve ``<name>`` to that class instead of the
    package, and the model would not compile.
    """


class InvalidModelError(ValueError):
    """The YAML model does not satisfy the Trano schema.

    ``errors`` holds one readable line per violation, e.g.
    ``spaces[0].parameters.floor_area: must be >= 0 (Floor area [m2])``.
    """

    def __init__(self, message: str, errors: Sequence[str] = ()) -> None:
        super().__init__(message)
        self.errors: tuple[str, ...] = tuple(errors)
