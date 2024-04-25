from typing import List

from neosim.construction import Construction
from neosim.glass import Glass
from neosim.models.constants import Tilt
from neosim.models.elements.wall import BaseWall, ExternalWall, FloorOnGround, Window


class MergedBaseWall(BaseWall):
    surfaces: List[float | int]
    azimuths: List[float | int]
    tilts: List[Tilt]
    constructions: List[Construction | Glass]

    @classmethod
    def from_base_elements(
        cls, base_walls: List[ExternalWall | Window | FloorOnGround]
    ) -> "MergedBaseWall":
        surfaces = [base_wall.surface for base_wall in base_walls]
        azimuths = [base_wall.azimuth for base_wall in base_walls]
        tilts = [base_wall.tilt for base_wall in base_walls]
        constructions = [base_wall.construction for base_wall in base_walls]
        names = [base_wall.name for base_wall in base_walls]
        return cls(
            name=f"merged_{'_'.join(names)}",
            surfaces=surfaces,
            azimuths=azimuths,
            tilts=tilts,
            constructions=constructions,
        )


class MergedExternalWall(MergedBaseWall):
    ...


class MergedFloor(MergedBaseWall):
    ...


class MergedWindows(MergedBaseWall):
    widths: List[float | int]
    heights: List[float | int]

    @classmethod
    def from_base_windows(cls, base_walls: List[Window]) -> "MergedWindows":
        surfaces = [base_wall.surface for base_wall in base_walls]
        azimuths = [base_wall.azimuth for base_wall in base_walls]
        tilts = [base_wall.tilt for base_wall in base_walls]
        constructions = [base_wall.construction for base_wall in base_walls]
        names = [base_wall.name for base_wall in base_walls]
        widths = [base_wall.width for base_wall in base_walls]
        heights = [base_wall.height for base_wall in base_walls]
        return cls(
            name=f"merged_{'_'.join(names)}",
            surfaces=surfaces,
            azimuths=azimuths,
            tilts=tilts,
            constructions=constructions,
            heights=heights,
            widths=widths,
        )
