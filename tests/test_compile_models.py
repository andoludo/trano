"""Every model of the repository compiles in OpenModelica, not only translates.

The other tests compare the generated Modelica with golden files, which proves that the text did not
change, not that it is valid Modelica. Here OpenModelica checks the model of every YAML file of the
repository (test models, tutorials, examples and the model of `trano verify`) with Buildings and IDEAS.

The `compile` tests need Docker and the OpenModelica image (see `TRANO_MODELICA_LIBRARIES`); they run in
the `compile` workflow on every push. `test_every_model_file_is_accounted_for` runs with the other tests:
a new YAML file must be compiled here, or listed below with the reason why not.
"""

import os
import tempfile
import warnings
from pathlib import Path

import pytest
import yaml

from trano.data.include import Loader
from trano.data_models.conversion import convert_network
from trano.elements.library.library import Library
from trano.exceptions import InvalidModelError
from trano.simulate.simulate import SimulationOptions, simulate
from trano.utils.utils import is_success
from trano.validation.issues import ModelWarning

ROOT = Path(__file__).parents[1]
MODEL_FILES = sorted(
    [
        *ROOT.joinpath("tests", "models").glob("*.yaml"),
        *ROOT.joinpath("tests", "tutorials").glob("*.yaml"),
        *ROOT.joinpath("examples").glob("*.yaml"),
        *ROOT.joinpath("validation", "systems", "cases").glob("*.yaml"),  # the systems validation, every variant
        ROOT.joinpath("trano", "verification.yaml"),
    ]
)
LIBRARIES = ("Buildings", "IDEAS")

# files of tests/models that are not building models
NOT_MODELS = {
    "ahu.yaml": "component definitions of the air handling unit, not a building",
    "boiler.yaml": "component definitions of the boiler, not a building",
    "radiator.yaml": "component definitions of the radiator, not a building",
    "test.yaml": "uses the unknown tag !include_constructions",
}
# models that must be rejected (tests/test_validity.py checks the messages)
INVALID = {
    "hello_world_missing_space_parameters.yaml",
    "single_zone_air_handling_unit.yaml",
    "single_zone_air_handling_unit_without_vav.yaml",
    "single_zone_air_handling_unit_without_vav_duct_only.yaml",
    "single_zone_air_handling_unit_wrong_flow.yaml",
    "single_zone_hydronic_incomplete_system.yaml",
    "single_zone_hydronic_no_junction.yaml",
    "single_zone_hydronic_unidentified_paramer.yaml",
    "single_zone_hydronic_unknown_id.yaml",
    "single_zone_hydronic_unknown_system.yaml",
    "single_zone_hydronic_unknown_variant.yaml",
}
# models written for some libraries only
ONLY = {
    "two_zones_ideas_rectangular_zone.yaml": ("IDEAS",),  # rectangular_zone: an IDEAS zone
}
# models whose weather file does not exist (they are only translated by the other tests): checked with the
# weather file of the repository instead
WEATHER_STAND_IN = {"house_values.yaml", "realistic.yaml"}
WEATHER_FILE = ROOT.joinpath("tests", "resources", "BEL_VLG_Uccle.064470_TMYx.2007-2021.mos")
# (file, library) that do not compile today, with the reason; strict, so that a fix is noticed
KNOWN_FAILURES: dict[tuple[str, str], str] = {}

COMPILED = [
    (path, library)
    for path in MODEL_FILES
    if path.name not in NOT_MODELS and path.name not in INVALID
    for library in ONLY.get(path.name, LIBRARIES)
]


def _shard(cases: list[tuple[Path, str]]) -> list[tuple[Path, str]]:
    """The cases of the shard `TRANO_COMPILE_SHARD=<k>/<n>` (all of them when unset), to split the checks over jobs."""
    shard = os.environ.get("TRANO_COMPILE_SHARD")
    if not shard:
        return cases
    index, count = (int(part) for part in shard.split("/"))
    return [case for position, case in enumerate(cases) if position % count == index - 1]


def _id(case: tuple[Path, str]) -> str:
    path, library = case
    return f"{path.parent.name}/{path.stem}-{library}"


def _marks(path: Path, library: str) -> list[pytest.MarkDecorator]:
    reason = KNOWN_FAILURES.get((path.name, library))
    return [pytest.mark.xfail(reason=reason, strict=True)] if reason else []


def test_every_model_file_is_accounted_for() -> None:
    """Each YAML file is compiled, a negative fixture that is rejected, or not a building model."""
    names = {path.name for path in MODEL_FILES}
    assert set(NOT_MODELS) | INVALID | set(ONLY) | WEATHER_STAND_IN <= names
    for path in MODEL_FILES:
        if path.name in INVALID:
            with pytest.raises(InvalidModelError):
                convert_network(path.stem, path)


@pytest.mark.compile
@pytest.mark.parametrize(
    ("path", "library"),
    [pytest.param(path, library, marks=_marks(path, library)) for path, library in _shard(COMPILED)],
    ids=[_id(case) for case in _shard(COMPILED)],
)
def test_model_compiles(path: Path, library: str) -> None:
    options = SimulationOptions(check_only=True)
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as project:
        if path.name in WEATHER_STAND_IN:
            model = yaml.load(path.read_text(), Loader=Loader)  # noqa: S506
            model["weather"]["parameters"]["path"] = str(WEATHER_FILE)
            path = Path(project, path.name)
            path.write_text(yaml.safe_dump(model, sort_keys=False))
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", ModelWarning)
            network = convert_network(path.stem, path, library=Library.from_configuration(library))
        results = simulate(Path(project), network, options=options, container_name=f"compile_{path.stem}_{library}")
    output = results.output.decode(errors="replace")
    assert is_success(results, options=options), output[-3000:]
