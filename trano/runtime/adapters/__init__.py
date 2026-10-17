"""Library adapters: the run-time names of the parameters trano writes for each library.

Only Buildings has an adapter so far; IDEAS gets its own module next to it and an entry in ``ADAPTERS``.
"""

from trano.runtime.adapters.base import LibraryAdapter, Snapshot, SnapshotEntry
from trano.runtime.adapters.buildings import BuildingsAdapter

ADAPTERS: dict[str, type[LibraryAdapter]] = {"buildings": BuildingsAdapter}


def get_adapter(library: str) -> LibraryAdapter:
    """The adapter of a library, by its name (case-insensitive)."""
    try:
        return ADAPTERS[library.lower()]()
    except KeyError:
        raise NotImplementedError(
            f"No run-time adapter for the {library} library; available: {', '.join(sorted(ADAPTERS))}"
        ) from None


__all__ = ["ADAPTERS", "BuildingsAdapter", "LibraryAdapter", "Snapshot", "SnapshotEntry", "get_adapter"]
