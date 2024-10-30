from typing import Optional

import docker  # type: ignore

from trano.simulate.simulate import SimulationOptions


def is_success(
    results: docker.models.containers.ExecResult,
    options: Optional[SimulationOptions] = None,
) -> bool:
    if options and options.check_only:
        return "true" in results.output.decode()
    return "The simulation finished successfully" in results.output.decode()
