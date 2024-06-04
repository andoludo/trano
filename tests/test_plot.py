import pandas as pd
import scipy
from buildingspy.io.outputfile import Reader


def test_read():
    mat = Reader(
        "/home/aan/Documents/neosim/results/buildings_simple_hydronic_two_zones.building_res.mat",
        "openmodelica",
    )
    a = 12
    r1 = mat.values("data_bus.dataBus.TZonSpace_1")
    r2 = mat.values("data_bus.dataBus.TZonSpace_2")
