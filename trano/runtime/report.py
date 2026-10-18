"""Summary tables of a parameter inventory, in Markdown (RUNTIME_PARAMETERS.md)."""

from collections import Counter
from collections.abc import Sequence

from trano.runtime.inventory import GROUPS, InventoryRow, Status, group_of

STATUSES = tuple(Status)
RUNTIME_CHANGEABLE = (Status.changeable, Status.derived, Status.no_impact)


def _table(header: Sequence[str], rows: Sequence[Sequence[object]]) -> str:
    lines = ["| " + " | ".join(header) + " |", "|" + "---|" * len(header)]
    lines += ["| " + " | ".join(str(cell) for cell in row) + " |" for row in rows]
    return "\n".join(lines)


def status_by_group(rows: Sequence[InventoryRow]) -> str:
    """The number of fields of each status, by parameter group."""
    counts = Counter((group_of(row.yaml_path)[0], row.status) for row in rows)
    groups = [group.name for group in GROUPS if any(group_of(row.yaml_path)[0] == group.name for row in rows)]
    table = [
        [group, *[counts[(group, status)] or "" for status in STATUSES], sum(counts[(group, s)] for s in STATUSES)]
        for group in groups
    ]
    totals = Counter(row.status for row in rows)
    table.append(["**all**", *[f"**{totals[status]}**" for status in STATUSES], f"**{len(rows)}**"])
    return _table(["group", *STATUSES, "fields"], table)


def changeable_flag_agreement(rows: Sequence[InventoryRow]) -> str:
    """How the ``isValueChangeable`` attributes of a field's run-time names compare to the run-time verdict.

    Only the fields that reach the executable are counted (not ``not_used`` or ``missing``).
    """
    counts: Counter[tuple[str, str]] = Counter()
    for row in rows:
        if row.status in (Status.not_used, Status.missing) or not row.is_value_changeable:
            continue
        verdict = "taken at run time" if row.status in RUNTIME_CHANGEABLE else "frozen"
        counts[(row.is_value_changeable, verdict)] += 1
    flags = ["true", "mixed", "false"]
    verdicts = ["taken at run time", "frozen"]
    return _table(
        ["isValueChangeable of the run-time names", *verdicts],
        [[flag, *[counts[(flag, verdict)] for verdict in verdicts]] for flag in flags],
    )


__all__ = ["changeable_flag_agreement", "status_by_group"]
