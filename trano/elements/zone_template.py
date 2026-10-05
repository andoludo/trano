"""Mapping of a space's envelope onto IDEAS' ``RectangularZoneTemplate``.

The template bundles a zone, its four vertical walls (faces A to D, at 90° from each
other), a floor and a ceiling, with one construction and one optional window per face.
Surfaces that do not fit this shape (pitched roofs, a second construction or glazing on
the same face, extra orientations, ...) stay separate IDEAS components connected to the
template through its ``proBusExt`` bus, so no surface of the space is lost.
"""

import math
from collections import defaultdict
from collections.abc import Iterable, Sequence
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field

from trano.elements.construction import Construction, Glass
from trano.elements.envelope import (
    BaseExternalWall,
    BaseFloorOnGround,
    BaseSimpleWall,
    BaseWindow,
)
from trano.elements.types import Tilt

FaceKey = Literal["A", "B", "C", "D", "Flo", "Cei"]
BoundaryType = Literal["OuterWall", "SlabOnGround", "None"]
VERTICAL_FACES: tuple[FaceKey, ...] = ("A", "B", "C", "D")
ANGLE_TOLERANCE = 1e-2  # [rad] accepts azimuths rounded to two decimals (1.57 for pi/2)
MINIMUM_WINDOW_HEIGHT = 0.1  # [m] lower bound enforced by IDEAS on h_win


def to_radians(azimuth: float) -> float:
    """Normalize an azimuth to [0, 2pi).

    Trano does not enforce a unit for azimuths: the YAML models document degrees while
    the Python API has historically been used with radians. A magnitude above 2pi can
    only be degrees, anything else is taken as radians.
    """
    radians = math.radians(azimuth) if abs(azimuth) > 2 * math.pi else azimuth
    return radians % (2 * math.pi)


def same_angle(first: float, second: float) -> bool:
    difference = (first - second) % (2 * math.pi)
    return min(difference, 2 * math.pi - difference) < ANGLE_TOLERANCE


class TemplateWindow(BaseModel):
    """The single window IDEAS models on a face: windows sharing a glazing are lumped."""

    model_config = ConfigDict(arbitrary_types_allowed=True)
    glazing: Glass
    area: float
    height: float

    @classmethod
    def from_windows(cls, windows: Sequence[BaseWindow]) -> "TemplateWindow":
        area = sum(window.surface for window in windows)
        height = sum((window.height or 0.0) * window.surface for window in windows) / area
        return cls(glazing=windows[0].construction, area=area, height=max(MINIMUM_WINDOW_HEIGHT, height))


class Face(BaseModel):
    model_config = ConfigDict(arbitrary_types_allowed=True)
    key: FaceKey
    boundary_type: BoundaryType = "None"
    construction: Construction | None = None
    area: float = 0.0
    window: TemplateWindow | None = None

    @property
    def length(self) -> float | None:
        """Horizontal length of a vertical face, derived from its gross area and the zone height."""
        return None

    @property
    def gross_area(self) -> float:
        return self.area + (self.window.area if self.window else 0.0)


class VerticalFace(Face):
    height: float

    @property
    def length(self) -> float | None:
        if self.boundary_type == "None":
            return None
        return self.gross_area / self.height


def _largest_group(elements: Iterable[BaseSimpleWall]) -> tuple[list[BaseSimpleWall], list[BaseSimpleWall]]:
    """Split elements into the largest group sharing a construction and the rest."""
    groups: dict[Construction | Glass, list[BaseSimpleWall]] = defaultdict(list)
    for element in elements:
        groups[element.construction].append(element)
    if not groups:
        return [], []
    largest = max(groups.values(), key=lambda group: (sum(e.surface for e in group), group[0].name))
    rest = [element for group in groups.values() if group is not largest for element in group]
    return largest, rest


class RectangularZone(BaseModel):
    """Parameters of ``IDEAS.Buildings.Components.RectangularZoneTemplate`` for one space."""

    model_config = ConfigDict(arbitrary_types_allowed=True)
    azimuth: float = Field(description="Azimuth of face A [rad]")
    height: float
    floor_area: float
    faces: list[Face]
    external_surfaces: list[BaseExternalWall | BaseWindow | BaseFloorOnGround] = Field(
        default_factory=list,
        description="Surfaces that do not fit the template and stay separate components.",
    )

    @classmethod
    def from_boundaries(
        cls,
        boundaries: Sequence[BaseExternalWall | BaseWindow | BaseFloorOnGround],
        height: float,
        floor_area: float,
    ) -> "RectangularZone":
        walls = [b for b in boundaries if isinstance(b, BaseExternalWall)]
        windows = [b for b in boundaries if isinstance(b, BaseWindow)]
        floors = [b for b in boundaries if isinstance(b, BaseFloorOnGround)]
        vertical_walls = [wall for wall in walls if wall.tilt == Tilt.wall]
        azimuth = _face_a_azimuth(vertical_walls, [w for w in windows if w.tilt == Tilt.wall])
        leftovers: list[BaseSimpleWall] = []
        faces: list[Face] = []
        vertical_windows = [window for window in windows if window.tilt == Tilt.wall]
        unassigned: list[BaseSimpleWall] = [*vertical_walls, *vertical_windows]
        for index, key in enumerate(VERTICAL_FACES):
            face_azimuth = azimuth + index * math.pi / 2
            face, rest, assigned = _vertical_face(key, face_azimuth, height, vertical_walls, vertical_windows)
            faces.append(face)
            leftovers += rest
            unassigned = [element for element in unassigned if element not in assigned]
        leftovers += unassigned
        floor, rest = _horizontal_face("Flo", floors, [])
        leftovers += rest
        ceiling, rest = _horizontal_face(
            "Cei",
            [wall for wall in walls if wall.tilt == Tilt.ceiling],
            [window for window in windows if window.tilt == Tilt.ceiling],
        )
        leftovers += rest
        leftovers += [wall for wall in walls if wall.tilt not in (Tilt.wall, Tilt.ceiling)]
        leftovers += [window for window in windows if window.tilt not in (Tilt.wall, Tilt.ceiling)]
        return cls(
            azimuth=azimuth,
            height=height,
            floor_area=floor_area,
            faces=[*faces, floor, ceiling],
            external_surfaces=sorted(leftovers, key=lambda element: element.name),
        )

    @property
    def length(self) -> float:
        """Length of faces A and C; falls back to a square zone when neither exists."""
        for key in ("A", "C"):
            length = self.face(key).length
            if length:
                return length
        return math.sqrt(self.floor_area)

    @property
    def width(self) -> float:
        return self.floor_area / self.length

    @property
    def ceiling_area(self) -> float:
        ceiling = self.face("Cei")
        return ceiling.gross_area if ceiling.boundary_type != "None" else self.floor_area

    def face(self, key: FaceKey) -> Face:
        return next(face for face in self.faces if face.key == key)

    def constructions(self) -> set[Construction | Glass]:
        """Constructions and glazings the template refers to, which must be in the data package."""
        result: set[Construction | Glass] = set()
        for face in self.faces:
            if face.construction:
                result.add(face.construction)
            if face.window:
                result.add(face.window.glazing)
        return result


def _face_a_azimuth(walls: Sequence[BaseExternalWall], windows: Sequence[BaseWindow]) -> float:
    """Azimuth of face A: the candidate that puts the most surfaces on the four faces."""
    surfaces = [to_radians(element.azimuth) for element in [*walls, *windows]]
    candidates = sorted({to_radians(wall.azimuth) for wall in walls})
    if not candidates:
        return 0.0

    def score(candidate: float) -> int:
        faces = [candidate + index * math.pi / 2 for index in range(4)]
        return sum(any(same_angle(surface, face) for face in faces) for surface in surfaces)

    return max(candidates, key=lambda candidate: (score(candidate), -candidate))


def _vertical_face(
    key: FaceKey,
    azimuth: float,
    height: float,
    walls: Sequence[BaseExternalWall],
    windows: Sequence[BaseWindow],
) -> tuple[Face, list[BaseSimpleWall], list[BaseSimpleWall]]:
    """Face for this azimuth, the surfaces of that orientation it cannot hold, and all it looked at."""
    face_walls: list[BaseSimpleWall] = [wall for wall in walls if same_angle(to_radians(wall.azimuth), azimuth)]
    face_windows: list[BaseSimpleWall] = [
        window for window in windows if same_angle(to_radians(window.azimuth), azimuth)
    ]
    kept_walls, rest = _largest_group(face_walls)
    if not kept_walls:
        return VerticalFace(key=key, height=height), [*rest, *face_windows], [*face_walls, *face_windows]
    kept_windows, rest_windows = _largest_group(face_windows)
    face = VerticalFace(
        key=key,
        height=height,
        boundary_type="OuterWall",
        construction=kept_walls[0].construction,
        area=sum(wall.surface for wall in kept_walls),
        window=TemplateWindow.from_windows(kept_windows) if kept_windows else None,  # type: ignore[arg-type]
    )
    return face, [*rest, *rest_windows], [*face_walls, *face_windows]


def _horizontal_face(
    key: FaceKey,
    walls: Sequence[BaseSimpleWall],
    windows: Sequence[BaseWindow],
) -> tuple[Face, list[BaseSimpleWall]]:
    kept_walls, rest = _largest_group(walls)
    if not kept_walls:
        return Face(key=key), [*rest, *windows]
    kept_windows, rest_windows = _largest_group(windows)
    face = Face(
        key=key,
        boundary_type="SlabOnGround" if key == "Flo" else "OuterWall",
        construction=kept_walls[0].construction,
        area=sum(wall.surface for wall in kept_walls),
        window=TemplateWindow.from_windows(kept_windows) if kept_windows else None,  # type: ignore[arg-type]
    )
    return face, [*rest, *rest_windows]
