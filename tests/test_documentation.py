
from neosim.topology import Network


def test_documentation_buildings_two_rooms_with_storage(
    buildings_two_rooms_with_storage: Network,
) -> None:
    model_ = buildings_two_rooms_with_storage.model()
    a = 12