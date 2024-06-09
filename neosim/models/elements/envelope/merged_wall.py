from neosim.models.elements.base import AvailableLibraries
from neosim.models.elements.envelope.base import MergedBaseExternalWall
from neosim.models.elements.envelope.external_wall import IdeasMergedExternalWall


class MergedExternalWall(MergedBaseExternalWall):
    libraries_data: AvailableLibraries = AvailableLibraries(
        ideas=[IdeasMergedExternalWall]
    )
