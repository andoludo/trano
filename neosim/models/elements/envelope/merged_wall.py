from typing import TYPE_CHECKING

from neosim.models.elements.base import AvailableLibraries
from neosim.models.elements.envelope.base import MergedBaseWall
from neosim.models.elements.envelope.external_wall import (
    ideas_merged_external_wall_factory,
)

if TYPE_CHECKING:
    pass


class MergedExternalWall(MergedBaseWall):
    libraries_data: AvailableLibraries = AvailableLibraries(
        ideas=[ideas_merged_external_wall_factory]
    )
