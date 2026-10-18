from collections.abc import Sequence


class WrongSystemFlowError(Exception):
    """Exception raised for wrong system flow."""


class ControllerBusPortError(Exception):
    """Exception raised for controller bus port error."""


class IncompatiblePortsError(Exception): ...


class DockerNotInstalledError(Exception): ...


class DockerClientError(Exception): ...


class ConnectionLimitReached(Exception): ...  # noqa: N818


class NoConnectionFoundError(Exception): ...


class ContainerNotFoundError(Exception): ...


class SystemsNotConnectedError(Exception): ...


class UnknownLibraryError(Exception): ...


class UnknownComponentVariantError(Exception): ...


class InvalidBuildingStructureError(Exception): ...


class InvalidSensorInletError(Exception): ...


class InvalidModelError(ValueError):
    """The YAML model does not satisfy the Trano schema.

    ``errors`` holds one readable line per violation, e.g.
    ``spaces[0].parameters.floor_area: must be >= 0 (Floor area [m2])``.
    """

    def __init__(self, message: str, errors: Sequence[str] = ()) -> None:
        super().__init__(message)
        self.errors: tuple[str, ...] = tuple(errors)


class OpenModelicaNotFoundError(Exception):
    """The local simulation runtime is selected but OpenModelica's ``omc`` is not on the PATH."""
