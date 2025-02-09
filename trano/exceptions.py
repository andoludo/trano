class WrongSystemFlowError(Exception):
    """Exception raised for wrong system flow."""


class ControllerBusPortError(Exception):
    """Exception raised for controller bus port error."""


class IncompatiblePortsError(Exception): ...


class DockerNotInstalledError(Exception): ...


class DockerClientError(Exception): ...

class ConnectionLimitReached(Exception): ...
