import logging

import docker  # type: ignore


from trano_simulate.result import SimulationResult
from trano_simulate.simulate import SimulationOptions

logger = logging.getLogger(__name__)


def is_success(
    results: docker.models.containers.ExecResult | SimulationResult,
    options: SimulationOptions | None = None,
) -> bool:
    if isinstance(results, SimulationResult):
        logger.debug(f"Simulation results: {results.log}")
        return results.success
    logger.debug(f"Simulation results: {results.output.decode()}")
    if options and options.check_only:
        # every loadModel prints "true": only the checkModel verdict tells whether the model is valid
        return "completed successfully" in results.output.decode()
    return "The simulation finished successfully" in results.output.decode()
