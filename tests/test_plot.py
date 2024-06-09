from buildingspy.io.outputfile import Reader


def test_read() -> None:
    mat = Reader(
        "/home/aan/Documents/neosim/results/buildings_simple_hydronic_two_zones.building_res.mat",
        "openmodelica",
    )
    assert mat
