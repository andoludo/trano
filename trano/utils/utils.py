import logging
from typing import Optional

import docker  # type: ignore


from trano.simulate.simulate import SimulationOptions

logger = logging.getLogger(__name__)


def is_success(
    results: docker.models.containers.ExecResult,
    options: Optional[SimulationOptions] = None,
) -> bool:
    logger.debug(f"Simulation results: {results.output.decode()}")
    if options and options.check_only:
        return "true" in results.output.decode()
    return "The simulation finished successfully" in results.output.decode()
