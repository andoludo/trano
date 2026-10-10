import logging
import math
from math import sqrt
from collections.abc import Callable
from typing import TYPE_CHECKING, Any, Type

from pydantic import BaseModel, ConfigDict, field_validator, model_validator, Field

from trano.elements.base import BaseElement
from trano.elements.construction import Construction, Glass
from trano.elements.types import Azimuth, Tilt, ContainerTypes
from trano.exceptions import InvalidBuildingStructureError

if TYPE_CHECKING:
    pass

logger = logging.getLogger(__name__)


class BaseWall(BaseElement):
    container_type: ContainerTypes = "envelope"

    # @computed_field  # type: ignore
    @property
    def length(self) -> int:
        if hasattr(self, "surfaces"):
            return len(self.surfaces)
        return 1


FULL_TURN = 2 * math.pi
WINDOW_AREA_TOLERANCE = 0.01
"""Relative tolerance between a window's surface and its width times height."""
GROUND_TEMPERATURE = 283.15
"""[K] Temperature at the outer surface of floors on ground (10 degC, the AixLib reduced-order default)."""


def check_azimuth_in_radians(azimuth: float | int) -> float | int:
    """Azimuths are radians (0 south, pi/2 west, pi north, -pi/2 east).

    A magnitude above one full turn can only be degrees, which every library would silently
    wrap into a wrong orientation.
    """
    if abs(azimuth) > FULL_TURN:
        raise InvalidBuildingStructureError(
            f"Azimuth {azimuth} is outside [-2*pi, 2*pi]: azimuths must be given in radians "
            "(0 south, pi/2 west, pi north, -pi/2 east), not in degrees."
        )
    return azimuth


class BaseSimpleWall(BaseWall):
    surface: float | int
    azimuth: float | int
    tilt: Tilt
    construction: Construction | Glass

    _azimuth_in_radians = field_validator("azimuth")(check_azimuth_in_radians)

    def get_tilt(self, space_name: str) -> Tilt:
        return self.tilt

    @property
    def opaque_surface(self) -> float:
        """Area of the element without the windows cut out of it [m2]; the surface itself unless it hosts windows."""
        return float(self.surface)


class BaseInternalElement(BaseSimpleWall):
    # An internal element receives no solar radiation: its orientation is irrelevant.
    azimuth: float | int = Azimuth.south


class BaseFloorOnGround(BaseSimpleWall):
    ground_temperature: float = GROUND_TEMPERATURE  # [K] at the outer surface of the floor construction


class BaseExternalWall(BaseSimpleWall):
    """External wall whose ``surface`` is the gross facade area, the windows it hosts included.

    The windows of a space are cut out of the walls facing the same way (see
    :func:`assign_windows_to_walls`): libraries that take the opaque wall on its own (IDEAS,
    AixLib, ISO 13790, MPC) get ``opaque_surface``, Buildings gets the gross area and cuts the
    window out itself.
    """

    hosted_window_area: float = 0.0  # [m2] windows cut out of this wall, assigned by its space

    @property
    def opaque_surface(self) -> float:
        if not self.hosted_window_area:
            return self.surface  # rendered as written (10, not 10.0)
        return _ten_digits(float(self.surface) - self.hosted_window_area)


class Overhang(BaseModel):
    """Horizontal projection above a window, measured in metres."""

    model_config = ConfigDict(frozen=True)
    depth: float = Field(..., gt=0, description="Projection perpendicular to the wall [m]")
    gap: float = Field(0.0, ge=0, description="Distance between the top of the window and the overhang [m]")
    width_left: float = Field(0.0, ge=0, description="Extension beyond the left edge of the window [m]")
    width_right: float = Field(0.0, ge=0, description="Extension beyond the right edge of the window [m]")


class SideFins(BaseModel):
    """Vertical projections on both sides of a window, measured in metres."""

    model_config = ConfigDict(frozen=True)
    depth: float = Field(..., gt=0, description="Projection perpendicular to the wall [m]")
    gap: float = Field(0.0, ge=0, description="Distance between the edges of the window and the fins [m]")
    height: float = Field(0.0, ge=0, description="Extension of the fins above the top of the window [m]")


class BaseWindow(BaseSimpleWall):
    width: float | None = None
    height: float | None = None
    frame_fraction: float = 0.1  # [1] share of the window area taken by the frame (the Buildings default)
    overhang: Overhang | None = None
    side_fins: SideFins | None = None

    @property
    def shaded(self) -> bool:
        return self.overhang is not None or self.side_fins is not None

    @model_validator(mode="after")
    def width_validator(self) -> "BaseWindow":
        if self.width is None and self.height is None:
            self.width = sqrt(self.surface)
            self.height = sqrt(self.surface)
        elif self.width is not None and self.height is None:
            self.height = self.surface / self.width
        elif self.width is None and self.height is not None:
            self.width = self.surface / self.height
        elif (
            self.width is not None
            and self.height is not None
            and not math.isclose(self.width * self.height, self.surface, rel_tol=WINDOW_AREA_TOLERANCE)
        ):
            raise InvalidBuildingStructureError(
                f"The surface of window {self.name} ({self.surface} m2) does not match its width * height "
                f"({self.width} m * {self.height} m)."
            )
        else:
            ...

        return self


def _get_element(
    construction_type: str,
    base_walls: list[BaseExternalWall | BaseWindow | BaseFloorOnGround],
    construction: Construction | Glass,
) -> list[BaseExternalWall | BaseWindow | BaseFloorOnGround]:
    return [getattr(base_wall, construction_type) for base_wall in base_walls if base_wall.construction == construction]


class MergedBaseWall(BaseWall):
    surfaces: list[float | int]
    azimuths: list[float | int]
    tilts: list[Tilt]
    constructions: list[Construction | Glass]
    include_in_layout: bool = False
    component_size: float = 3

    @classmethod
    def from_base_elements(
        cls, base_walls: list[BaseExternalWall | BaseWindow | BaseFloorOnGround]
    ) -> list["MergedBaseWall"]:
        merged_walls = []
        unique_constructions = {base_wall.construction for base_wall in base_walls}

        for construction in unique_constructions:
            data: dict[
                str,
                list[BaseExternalWall | BaseWindow | BaseFloorOnGround],
            ] = {
                "azimuth": [],
                "tilt": [],
                "name": [],
                "opaque_surface": [],
            }
            for construction_type in data:
                data[construction_type] = _get_element(construction_type, base_walls, construction)
            merged_wall = cls(
                name=f"merged_{'_'.join(data['name'])}",  # type: ignore
                surfaces=data["opaque_surface"],
                azimuths=data["azimuth"],
                tilts=data["tilt"],
                constructions=[construction],
            )
            merged_walls.append(merged_wall)
        return sorted(merged_walls, key=lambda x: x.name)  # type: ignore #TODO: what is the issue with this!!!


class MergedBaseWindow(MergedBaseWall): ...


class MergedBaseExternalWall(MergedBaseWall): ...


class ExternalDoor(BaseExternalWall): ...


class ExternalWall(ExternalDoor): ...


class FloorOnGround(BaseFloorOnGround):
    azimuth: float | int = Azimuth.south
    tilt: Tilt = Tilt.floor
    include_in_layout: bool = False
    component_size: float = 3


class SpaceTilt(BaseModel):
    space_name: str
    tilt: Tilt | None = None


class InternalElement(BaseInternalElement):
    space_tilts: list[SpaceTilt] = Field(default_factory=list)

    def get_tilt(self, space_name: str) -> Tilt:
        for space_tilt in self.space_tilts:
            if space_tilt.tilt and space_tilt.space_name == space_name:
                return space_tilt.tilt
        return self.tilt


class MergedFloor(MergedBaseWall): ...


class MergedExternalWall(MergedBaseExternalWall): ...


class MergedWindows(MergedBaseWindow):
    widths: list[float | int]
    heights: list[float | int]

    @classmethod
    def from_base_windows(cls, base_walls: list["BaseWindow"]) -> list["MergedWindows"]:
        merged_windows = []
        unique_constructions = {base_wall.construction for base_wall in base_walls}

        for construction in unique_constructions:
            data: dict[str, list[ExternalWall | FloorOnGround | BaseWindow | str]] = {
                "azimuth": [],
                "tilt": [],
                "name": [],
                "surface": [],
                "width": [],
                "height": [],
            }
            for construction_type in data:
                data[construction_type] = _get_element(
                    construction_type,
                    base_walls,  # type: ignore
                    construction,
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


class Window(BaseWindow): ...


class WindowedWall(BaseSimpleWall): ...


ANGLE_TOLERANCE = 1e-2
"""[rad] Azimuths closer than this face the same way: accepts azimuths rounded to two decimals (1.57 for pi/2)."""


def same_angle(first: float, second: float) -> bool:
    """Whether two azimuths [rad] face the same way, also across a full turn (0 and 2*pi)."""
    difference = (first - second) % FULL_TURN
    return min(difference, FULL_TURN - difference) < ANGLE_TOLERANCE


def same_orientation(first: BaseSimpleWall, second: BaseSimpleWall) -> bool:
    return same_angle(first.azimuth, second.azimuth) and first.tilt == second.tilt


class WallParameters(BaseModel):
    """Constructions of one kind, rendered as the arrays of a Buildings MixedAir zone."""

    number: int
    surfaces: list[float]
    azimuths: list[float]
    layers: list[str]
    tilts: list[Tilt]
    type: str

    @classmethod
    def from_neighbors(
        cls,
        space_name: str,
        neighbors: list["BaseElement"],
        wall: Type["BaseSimpleWall"],
        filter: list[str] | None = None,
    ) -> "WallParameters":
        constructions = [
            neighbor for neighbor in neighbors if isinstance(neighbor, wall) if neighbor.name not in (filter or [])
        ]
        return cls(
            number=len(constructions),
            surfaces=[construction.surface for construction in constructions],
            azimuths=[construction.azimuth for construction in constructions],
            layers=[construction.construction.name for construction in constructions],
            tilts=[construction.get_tilt(space_name) for construction in constructions],
            type=wall.__name__,
        )


def _ten_digits(value: float) -> float:
    """Value rounded to ten significant digits: exact for the model, without float noise (7.200000000000001)."""
    return float(f"{value:.10g}")


class HostedWindows(BaseModel):
    """The windows of one orientation and the walls of that orientation they are cut out of."""

    model_config = ConfigDict(arbitrary_types_allowed=True)
    host_walls: list[ExternalWall]
    windows: list[BaseWindow]

    @property
    def gross_area(self) -> float:
        """Area of the host walls, windows included (Buildings ``datConExtWin.A``)."""
        return float(sum(wall.surface for wall in self.host_walls))

    @property
    def window_area(self) -> float:
        return float(sum(window.surface for window in self.windows))


def hosted_windows(boundaries: list["BaseElement"]) -> list[HostedWindows]:
    """Group the windows by orientation with their host walls, checking that they fit in them.

    The host walls are the walls with the same azimuth and tilt whose construction covers the
    largest area.
    """
    windows = [boundary for boundary in boundaries if isinstance(boundary, BaseWindow)]
    walls = [boundary for boundary in boundaries if isinstance(boundary, ExternalWall)]
    groups = [
        HostedWindows(host_walls=_host_walls(walls, orientation_windows[0]), windows=orientation_windows)
        for orientation_windows in _group(windows, same_orientation)
    ]
    for group in groups:
        if group.window_area > group.gross_area * (1 + 1e-9):
            raise InvalidBuildingStructureError(
                f"The windows {[window.name for window in group.windows]} ({group.window_area} m2) are larger "
                f"than the walls {[wall.name for wall in group.host_walls]} ({group.gross_area} m2) of the same "
                "orientation: the wall surface is the gross area, windows included."
            )
    return groups


def assign_windows_to_walls(boundaries: list["BaseElement"]) -> None:
    """Cut the windows out of their host walls: sets ``hosted_window_area`` on every external wall.

    The window area of an orientation is spread over its host walls in proportion to their size.
    """
    for boundary in boundaries:
        if isinstance(boundary, BaseExternalWall):
            boundary.hosted_window_area = 0.0
    for group in hosted_windows(boundaries):
        for wall in group.host_walls:
            wall.hosted_window_area += group.window_area * wall.surface / group.gross_area


class WindowedWallParameters(WallParameters):
    """Walls with windows (Buildings ``datConExtWin``): one entry per orientation and glazing.

    The host walls of the windows are excluded from the opaque walls (``datConExt``). With several
    glazings on one orientation, the gross wall area is split between the entries in proportion to
    their window areas, so that it is counted once.
    """

    window_layers: list[str]
    window_width: list[float]
    window_height: list[float]
    window_frame_fraction: list[float]
    # Overhang and side fins per entry (Buildings ``ove`` and ``sidFin``), zero depth when absent.
    overhang_width_left: list[float]
    overhang_width_right: list[float]
    overhang_depth: list[float]
    overhang_gap: list[float]
    side_fin_height: list[float]
    side_fin_depth: list[float]
    side_fin_gap: list[float]
    included_external_walls: list[str]

    @property
    def has_shading(self) -> bool:
        return any(depth > 0 for depth in self.overhang_depth + self.side_fin_depth)

    @classmethod
    def from_neighbors(cls, neighbors: list["BaseElement"]) -> "WindowedWallParameters":  # type: ignore[override]
        entries: dict[str, list[Any]] = {
            key: []
            for key in (
                "surfaces",
                "azimuths",
                "layers",
                "tilts",
                "window_layers",
                "window_width",
                "window_height",
                "window_frame_fraction",
                "overhang_width_left",
                "overhang_width_right",
                "overhang_depth",
                "overhang_gap",
                "side_fin_height",
                "side_fin_depth",
                "side_fin_gap",
            )
        }
        included_external_walls: list[str] = []
        for group in hosted_windows(neighbors):
            host_walls = group.host_walls
            included_external_walls += [wall.name for wall in host_walls if wall.name is not None]
            gross_area, window_area = group.gross_area, group.window_area
            for glazing_windows in _group(group.windows, lambda a, b: a.construction == b.construction):
                area = sum(window.surface for window in glazing_windows)
                height = sum(window.surface * window.height for window in glazing_windows) / area
                entries["surfaces"].append(_ten_digits(gross_area * area / window_area))
                entries["azimuths"].append(host_walls[0].azimuth)
                entries["layers"].append(host_walls[0].construction.name)
                entries["tilts"].append(host_walls[0].tilt)
                entries["window_layers"].append(glazing_windows[0].construction.name)
                entries["window_height"].append(_ten_digits(height))
                entries["window_width"].append(_ten_digits(area / height))
                entries["window_frame_fraction"].append(
                    _ten_digits(sum(window.surface * window.frame_fraction for window in glazing_windows) / area)
                )
                overhang, side_fins = _shading_of(glazing_windows)
                entries["overhang_width_left"].append(overhang.width_left if overhang else 0.0)
                entries["overhang_width_right"].append(overhang.width_right if overhang else 0.0)
                entries["overhang_depth"].append(overhang.depth if overhang else 0.0)
                entries["overhang_gap"].append(overhang.gap if overhang else 0.0)
                entries["side_fin_height"].append(side_fins.height if side_fins else 0.0)
                entries["side_fin_depth"].append(side_fins.depth if side_fins else 0.0)
                entries["side_fin_gap"].append(side_fins.gap if side_fins else 0.0)
        return cls(
            number=len(entries["surfaces"]),
            type="WindowedWall",
            included_external_walls=included_external_walls,
            **entries,
        )


def _shading_of(windows: list[BaseWindow]) -> tuple[Overhang | None, SideFins | None]:
    """The shading shared by windows merged into one entry; they must agree on it."""
    overhangs = {window.overhang for window in windows}
    side_fins = {window.side_fins for window in windows}
    if len(overhangs) > 1 or len(side_fins) > 1:
        raise InvalidBuildingStructureError(
            f"The windows {[window.name for window in windows]} share one orientation and glazing but differ in "
            "their overhang or side fins: give them the same shading or different glazings."
        )
    return overhangs.pop(), side_fins.pop()


def _group(elements: list[Any], same: Callable[[Any, Any], bool]) -> list[list[Any]]:
    """Group elements with an equivalence test, keeping the order of first appearance."""
    groups: list[list[Any]] = []
    for element in elements:
        group = next((group for group in groups if same(group[0], element)), None)
        if group is None:
            groups.append([element])
        else:
            group.append(element)
    return groups


def _host_walls(walls: list[ExternalWall], window: BaseWindow) -> list[ExternalWall]:
    """Walls hosting the windows of an orientation: same azimuth and tilt, construction with the largest area."""
    candidates = [wall for wall in walls if same_orientation(wall, window)]
    if not candidates:
        raise InvalidBuildingStructureError(
            f"No wall found with the same azimuth and tilt as the window {window.name}."
        )
    by_construction = _group(candidates, lambda a, b: a.construction == b.construction)
    if len(by_construction) > 1:
        logger.warning(
            "The walls facing the window %s have different constructions; the windows are placed in the "
            "construction with the largest area.",
            window.name,
        )
    return max(by_construction, key=lambda group: sum(wall.surface for wall in group))
