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
