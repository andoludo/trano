"""Tests for the component variant system, including loading variants from
external folders outside trano."""

import os
from pathlib import Path
import textwrap

import pytest
import yaml

from trano.elements.library import components as components_module
from trano.elements.library.components import (
    COMPONENTS,
    Components,
    clear_variants_folders,
    register_variants_folder,
    registered_variants_folders,
)
from trano.elements.system import Radiator
from trano.exceptions import UnknownComponentVariantError


CUSTOM_RADIATOR_VARIANT_YAML = """
- classes:
  - Radiator
  figures: []
  library: default
  parameter_processing:
    function: default_parameters
  ports:
  - flow: convective
    medium: heat
    names:
    - heatPortCon
    targets:
    - Space
  - flow: radiative
    medium: heat
    names:
    - heatPortRad
    targets:
    - Space
  - flow: inlet
    medium: fluid
    names:
    - port_a
  - flow: outlet
    medium: fluid
    names:
    - port_b
  template: |2-

        Custom.Radiator {{ element.name }}
  variant: my_custom_radiator
"""


@pytest.fixture(autouse=True)
def _reset_variants_after_test() -> None:
    """Ensure each test starts and ends with a clean external folder list."""
    clear_variants_folders()
    yield
    clear_variants_folders()


def _write_variant(tmp_path: Path, file_name: str, content: str) -> Path:
    folder = tmp_path / "custom_variants"
    folder.mkdir(exist_ok=True)
    (folder / file_name).write_text(content.strip())
    return folder


def test_components_load_includes_builtin_variants() -> None:
    components = Components.load()
    classes = {c["classes"][0] for c in components.components}
    # A few well-known components must be loaded out of the box.
    assert {"Radiator", "Space", "EmissionControl", "AirHandlingUnit"}.issubset(classes)


def test_register_variants_folder_adds_variants_to_components(tmp_path: Path) -> None:
    folder = _write_variant(tmp_path, "radiator.yaml", CUSTOM_RADIATOR_VARIANT_YAML)

    initial_count = len(COMPONENTS.components)
    register_variants_folder(folder)

    assert len(COMPONENTS.components) == initial_count + 1
    assert folder.resolve() in registered_variants_folders()

    radiator = Radiator(name="radiator_custom", variant="my_custom_radiator")
    available_variants = {ld.variant for ld in radiator.libraries_data}
    assert {"default", "ideal", "my_custom_radiator"}.issubset(available_variants)


def test_clear_variants_folders_removes_external_variants(tmp_path: Path) -> None:
    folder = _write_variant(tmp_path, "radiator.yaml", CUSTOM_RADIATOR_VARIANT_YAML)
    register_variants_folder(folder)
    assert any(c.get("variant") == "my_custom_radiator" for c in COMPONENTS.components)

    clear_variants_folders()
    assert registered_variants_folders() == []
    assert not any(c.get("variant") == "my_custom_radiator" for c in COMPONENTS.components)


def test_register_variants_folder_missing_path_raises(tmp_path: Path) -> None:
    missing = tmp_path / "does_not_exist"
    with pytest.raises(FileNotFoundError):
        register_variants_folder(missing)


def test_register_variants_folder_path_is_file_raises(tmp_path: Path) -> None:
    file_path = tmp_path / "not_a_folder.yaml"
    file_path.write_text("- {}")
    with pytest.raises(NotADirectoryError):
        register_variants_folder(file_path)


def test_invalid_variant_yaml_top_level_must_be_list(tmp_path: Path) -> None:
    folder = tmp_path / "broken"
    folder.mkdir()
    (folder / "broken.yaml").write_text("classes:\n- Radiator\n")
    with pytest.raises(ValueError, match="must contain a list"):
        register_variants_folder(folder)


def test_register_same_folder_twice_is_idempotent(tmp_path: Path) -> None:
    folder = _write_variant(tmp_path, "radiator.yaml", CUSTOM_RADIATOR_VARIANT_YAML)
    register_variants_folder(folder)
    count_after_first = len(COMPONENTS.components)
    register_variants_folder(folder)
    assert len(COMPONENTS.components) == count_after_first


def test_unknown_variant_raises_explicit_error() -> None:
    radiator = Radiator(name="r1", variant="this_variant_does_not_exist")
    from trano.elements.library.library import Library

    library = Library.load_default()
    with pytest.raises(UnknownComponentVariantError):
        radiator.get_library_data(library)


def test_external_variant_can_be_used_in_yaml_model(tmp_path: Path) -> None:
    """A custom variant registered from a folder outside trano must be usable
    by name from a YAML model file."""
    folder = _write_variant(tmp_path, "radiator.yaml", CUSTOM_RADIATOR_VARIANT_YAML)
    register_variants_folder(folder)

    radiator = Radiator(name="rad_yaml", variant="my_custom_radiator")
    from trano.elements.library.library import Library

    library = Library.load_default()
    library_data = radiator.get_library_data(library)
    assert library_data is not None
    assert library_data.variant == "my_custom_radiator"
    assert "Custom.Radiator" in library_data.template


def test_environment_variable_registers_folders(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """``TRANO_VARIANTS_PATH`` must register one or more folders when read at
    import time. Here we exercise the helper that parses the variable."""
    folder_a = tmp_path / "a"
    folder_a.mkdir()
    (folder_a / "rad.yaml").write_text(CUSTOM_RADIATOR_VARIANT_YAML.strip())
    folder_b = tmp_path / "b"
    folder_b.mkdir()

    monkeypatch.setenv(
        components_module._TRANO_VARIANTS_PATH_ENV,
        os.pathsep.join([str(folder_a), str(folder_b)]),
    )

    parsed = components_module._initial_extra_paths()
    assert folder_a.resolve() in parsed
    assert folder_b.resolve() in parsed


def test_variant_yaml_in_subfolder_is_picked_up(tmp_path: Path) -> None:
    """The folder is scanned recursively, so subfolders work too."""
    nested = tmp_path / "library" / "custom" / "subfolder"
    nested.mkdir(parents=True)
    (nested / "radiator.yaml").write_text(CUSTOM_RADIATOR_VARIANT_YAML.strip())

    register_variants_folder(tmp_path / "library")

    radiator = Radiator(name="radiator_nested", variant="my_custom_radiator")
    assert any(ld.variant == "my_custom_radiator" for ld in radiator.libraries_data)


def test_components_get_components_detects_duplicate_variants(tmp_path: Path) -> None:
    folder = tmp_path / "dup"
    folder.mkdir()
    duplicate = yaml.safe_load(CUSTOM_RADIATOR_VARIANT_YAML)
    (folder / "rad_a.yaml").write_text(yaml.safe_dump(duplicate))
    (folder / "rad_b.yaml").write_text(yaml.safe_dump(duplicate))

    register_variants_folder(folder)

    with pytest.raises(ValueError, match="Duplicate variant"):
        COMPONENTS.get_components("Radiator")


def test_create_model_yaml_using_custom_variant(tmp_path: Path) -> None:
    """End-to-end: register a custom variant folder, then build a Trano network
    from a YAML model that references the new variant."""
    radiator_folder = _write_variant(
        tmp_path,
        "radiator.yaml",
        CUSTOM_RADIATOR_VARIANT_YAML,
    )
    register_variants_folder(radiator_folder)

    model_yaml = tmp_path / "single_zone_custom_variant.yaml"
    model_yaml.write_text(
        textwrap.dedent(
            """
            default: !include_default
            spaces:
              - parameters:
                  floor_area: 50.0
                  average_room_height: 2.5
                id: SPACE:001
                external_boundaries:
                  external_walls:
                    - surface: 20
                      azimuth: 0
                      tilt: wall
                      construction: CAVITYWALL:001
                emissions:
                  - radiator:
                      id: RADIATOR:001
                      variant: my_custom_radiator
                      parameters:
                        nominal_heating_power_positive_for_heating: 2500
                      control:
                        emission_control:
                          id: EMISSION_CONTROL:001
            systems: []
            """
        ).strip()
    )

    from trano.data_models.conversion import convert_network
    from trano.elements.library.library import Library

    network = convert_network("custom_variant_model", model_yaml, library=Library.load_default())

    radiators = [n for n in network.graph.nodes if isinstance(n, Radiator)]
    assert any(r.variant == "my_custom_radiator" for r in radiators)
