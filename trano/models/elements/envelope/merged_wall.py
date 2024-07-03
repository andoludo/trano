from trano.models.elements.base import AvailableLibraries
from trano.models.elements.envelope.base import MergedBaseExternalWall
from trano.models.elements.envelope.external_wall import IdeasMergedExternalWall


class MergedExternalWall(MergedBaseExternalWall):
    libraries_data: AvailableLibraries = AvailableLibraries(
        ideas=[IdeasMergedExternalWall]
    )
