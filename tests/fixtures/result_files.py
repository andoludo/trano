"""Reduce an OpenModelica result file to a few of its variables, for a small test fixture.

The fixtures of ``tests/data/kpi`` were made from one simulated day of ``tests/tutorials/first_model.yaml``::

    simulate_yaml(Path("tests/tutorials/first_model.yaml"), library=library, runtime="local",
                  options=SimulationOptions(end_time=86400, number_of_intervals=96))

then ``python -m tests.fixtures.result_files <result.mat> <fixture.mat> <variable> ...``. The variables
keep their values on the time grid of the simulation; the parameters (constant data) are dropped.
"""

import sys
from pathlib import Path

import numpy as np
import scipy.io  # type: ignore[import-untyped]
from buildingspy.io.outputfile import Reader  # type: ignore[import-untyped]


def reduce_result(source: Path, target: Path, variables: list[str]) -> None:
    reader = Reader(str(source), "openmodelica")
    time = reader.values(variables[0])[0]
    names = ["time", *variables]
    values = np.vstack([time, *(reader.values(name)[1] for name in variables)])
    width = max(map(len, names))
    scipy.io.savemat(
        str(target),
        {
            "Aclass": np.array(["Atrajectory", "1.1        ", "           ", "binTrans   "]),
            # character matrices, one column per variable
            "name": np.array(["".join(name.ljust(width)[row] for name in names) for row in range(width)]),
            "description": np.array([" " * len(names)]),
            # each variable is a row of data_2 (positive: no sign change), linearly interpolated
            "dataInfo": np.array(
                [[0, 1, 0, -1]] + [[2, row, 0, -1] for row in range(2, len(names) + 1)], dtype=np.int32
            ).T,
            "data_1": np.array([[time[0]], [time[-1]]]),
            "data_2": values,
        },
        format="4",
    )


if __name__ == "__main__":
    reduce_result(Path(sys.argv[1]), Path(sys.argv[2]), sys.argv[3:])
