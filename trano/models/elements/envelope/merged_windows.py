from typing import Dict, List, Union

from trano.models.elements.base import AvailableLibraries
from trano.models.elements.envelope.base import (
    BaseWindow,
    MergedBaseWindow,
    _get_element,
)
from trano.models.elements.envelope.external_wall import ExternalWall
from trano.models.elements.envelope.floor_on_ground import FloorOnGround
from trano.models.elements.envelope.window import IdeasMergedWindows


class MergedWindows(MergedBaseWindow):
    widths: List[float | int]
    heights: List[float | int]
    libraries_data: AvailableLibraries = AvailableLibraries(ideas=[IdeasMergedWindows])

    @classmethod
    def from_base_windows(cls, base_walls: List["BaseWindow"]) -> List["MergedWindows"]:
        merged_windows = []
        unique_constructions = {base_wall.construction for base_wall in base_walls}

        for construction in unique_constructions:
            data: Dict[
                str, List[Union["ExternalWall", "FloorOnGround", "BaseWindow", str]]
            ] = {
                "azimuth": [],
                "tilt": [],
                "name": [],
                "surface": [],
                "width": [],
                "height": [],
            }
            for construction_type in data:
                data[construction_type] = _get_element(
                    construction_type, base_walls, construction  # type: ignore
                )
            merged_window = cls(
                name=f"merged_{'_'.join(data['name'])}",  # type: ignore
                surfaces=data["surface"],
                azimuths=data["azimuth"],
                tilts=data["tilt"],
                constructions=[construction],
                heights=data["height"],
                widths=data["width"],
            )
            merged_windows.append(merged_window)
        return sorted(merged_windows, key=lambda x: x.name)  # type: ignore
