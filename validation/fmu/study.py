"""The inventory study: which fields are checked, how the checks are sampled, and the CSV written.

Every numeric field is resolved and classified (cheap). The simulation checks are sampled:

- **readback** runs for every candidate (a few seconds each, no simulation);
- **impact** runs once per *field family*: the fields of the same name on the same kind of component
  (the six radiators' ``Q_flow_nominal``) share one simulation, on the first instance in the YAML; the
  other instances carry the metric of their representative, marked as such;
- **equivalence** rebuilds the FMU, which takes as long as the baseline export: it runs on a sample of
  one high-priority field per parameter group (``EQUIVALENCE_SAMPLE``), or on the fields given.
"""

from __future__ import annotations

import json
import logging
import re
from collections.abc import Iterable
from pathlib import Path

from pydantic import BaseModel, Field

from trano.fmu.calibration import CheckResults, Inventory, InventoryBuilder, InventoryCase, apply_results
from trano.fmu.inventory import InventoryRow, write_inventory

logger = logging.getLogger(__name__)

EQUIVALENCE_SAMPLE = (
    "material[EPS:001].thermal_conductivity",
    "constructions[EXTERNAL_WALL:001].layers[0].thickness",
    "glazings[WINDOW:001].u_value_frame",
    "glass_material[GLASS:001].solar_transmittance",
    "spaces[LIVING:001].parameters.n50",
    "spaces[LIVING:001].external_boundaries.floor_on_grounds[0].ground_temperature",
    "spaces[LIVING:001].external_boundaries.external_walls[0].surface",
    "spaces[LIVING:001].emissions[0].radiator.parameters.nominal_heating_power_positive_for_heating",
    "spaces[LIVING:001].emissions[1].valve.control.emission_control.parameters.temperature_heating_setpoint",
    "systems[0].boiler.parameters.nominal_heating_power",
    "systems[0].boiler.parameters.temperature_supply_setpoint_heat_pump",
    "systems[1].pump.parameters.m_flow_nominal",
    "systems[5].air_handling_unit.parameters.heat_exchanger_effectiveness",
    "spaces[LIVING:001].ventilation_inlets[0].duct.parameters.nominal_mass_flow_rate",
    "solar[0].photovoltaic.parameters.area",
)
"""One high-priority field per parameter group of examples/residential_after_renovation.yaml."""
_INDEX = re.compile(r"\[[^\]]*\]")


def family(yaml_path: str) -> str:
    """The field family: the path with every index and id removed (``spaces[].emissions[].radiator.parameters.x``)."""
    return _INDEX.sub("[]", yaml_path)


class StudyResults(BaseModel):
    """What the checks found, keyed by YAML path, to resume a study without running them again."""

    readback: dict[str, CheckResults] = Field(default_factory=dict)
    impact: dict[str, CheckResults] = Field(default_factory=dict)
    equivalence: dict[str, CheckResults] = Field(default_factory=dict)

    @classmethod
    def load(cls, path: Path) -> StudyResults:
        return cls.model_validate_json(path.read_text()) if path.exists() else cls()

    def save(self, path: Path) -> None:
        path.write_text(self.model_dump_json(indent=1))


def representatives(cases: Iterable[InventoryCase]) -> dict[str, str]:
    """Family → YAML path of the first candidate of the family (the one simulated for the impact)."""
    chosen: dict[str, str] = {}
    for case in cases:
        if case.candidate and case.start_values:
            chosen.setdefault(family(case.resolved.yaml_path), case.resolved.yaml_path)
    return chosen


def run_study(
    builder: InventoryBuilder,
    results_path: Path,
    equivalence: Iterable[str] = EQUIVALENCE_SAMPLE,
    impact: bool = True,
    rebuild_directory: Path | None = None,
) -> Inventory:
    """Resolve, classify and check; results are stored after each check so that a stopped study resumes."""
    results = StudyResults.load(results_path)
    cases = builder.resolve()
    chosen = representatives(cases)
    wanted_equivalence = set(equivalence)
    override_runs = {}
    for case in cases:
        path = case.resolved.yaml_path
        if not case.candidate or not case.start_values:
            continue
        if path not in results.readback:
            logger.info("readback %s", path)
            results.readback[path] = builder.readback(case)
            results.save(results_path)
        if impact and chosen.get(family(path)) == path and path not in results.impact:
            logger.info("impact %s", path)
            results.impact[path], run = builder.impact(case)
            if run is not None:
                override_runs[path] = run
            results.save(results_path)
        if path in wanted_equivalence and path not in results.equivalence:
            logger.info("equivalence %s (rebuilding the FMU)", path)
            run = override_runs.get(path)
            if run is None:
                _, run = builder.impact(case)
            if run is None:
                results.equivalence[path] = CheckResults(equivalence_ok=False, equivalence_detail="override run failed")
            else:
                directory = (rebuild_directory or builder.work_directory) / _slug(path)
                results.equivalence[path] = builder.equivalence(case, run, directory)
            results.save(results_path)
    rows = [_finish(case, results, chosen) for case in cases]
    return Inventory(
        library=builder.library.name,
        yaml_path=builder.yaml_path,
        fmu=builder.fmu,
        outputs=builder.outputs,
        cases=[case.model_copy(update={"row": row}) for case, row in zip(cases, rows, strict=True)],
    )


def _finish(case: InventoryCase, results: StudyResults, chosen: dict[str, str]) -> InventoryRow:
    path = case.resolved.yaml_path
    found = [results.readback.get(path), results.impact.get(path), results.equivalence.get(path)]
    representative = chosen.get(family(path))
    if path not in results.impact and representative and representative in results.impact:
        shared = results.impact[representative]
        found.append(shared.model_copy(update={"impact": f"{shared.impact} [from {representative}]"}))
    return apply_results(case.row, [result for result in found if result is not None])


def _slug(yaml_path: str) -> str:
    return re.sub(r"[^A-Za-z0-9]+", "_", yaml_path).strip("_")[-80:]


def write_outputs(inventory: Inventory, directory: Path) -> tuple[Path, Path]:
    """The CSV of the inventory and a JSON with the outputs and the FMU it was checked against."""
    csv_path = write_inventory(inventory.rows(), directory / f"parameter_inventory_{inventory.library.lower()}.csv")
    meta = {
        "library": inventory.library,
        "yaml": str(inventory.yaml_path),
        "fmu": str(inventory.fmu),
        "outputs": inventory.outputs.model_dump(),
        "status_counts": {
            status: sum(row.status == status for row in inventory.rows())
            for status in sorted({row.status for row in inventory.rows()})
        },
    }
    meta_path = directory / f"parameter_inventory_{inventory.library.lower()}.json"
    meta_path.write_text(json.dumps(meta, indent=1))
    return csv_path, meta_path
