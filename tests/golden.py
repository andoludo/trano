"""Compare generated Modelica models against the golden files in tests/data.

The comparison ignores annotations, documentation, whitespace and the embedded
Trano package: a model matches its golden file when both reduce to the same set
of statements. Set the environment variable ``TRANO_OVERWRITE_GOLDEN=1`` to
regenerate the golden files instead of comparing against them.
"""

import os
import re
from pathlib import Path

DATA_DIRECTORY = Path(__file__).parent.joinpath("data")


def remove_annotation(model: str) -> str:
    for documentation in re.findall(r"Documentation(.*?)</html>", model, re.DOTALL):
        model = model.replace(documentation, "").replace("Documentation", "")

    model = model.replace(" ", "").replace("\n", "")
    for annotation in re.findall(r"annotation(.*?);", model):
        model = model.replace(annotation, "").replace("annotation", "")

    return model


def remove_trano_package(model: str) -> str:
    for annotation in re.findall(r"package Trano(.*?)end Trano;", model, re.DOTALL):
        model = model.replace(annotation, "").replace("package Trano", "").replace("end Trano;", "")
    return model


def _to_statements(model: str, model_name: str) -> set:
    model = remove_annotation(remove_trano_package(model))
    return {
        line
        for line in set(model.replace("record", ";").replace(f"model{model_name}", "").split(";"))
        if "ReaderTMY3weather" not in line
    }


def clean_model(model: str, model_name: str) -> set:
    if os.environ.get("TRANO_OVERWRITE_GOLDEN") == "1":
        DATA_DIRECTORY.joinpath(f"{model_name}.mo").write_text(model)
    return _to_statements(model, model_name)


def read_golden(file_name: str) -> set:
    return _to_statements(DATA_DIRECTORY.joinpath(f"{file_name}.mo").read_text(), file_name)


def assert_model_equals_golden(model: str, golden_name: str) -> None:
    assert clean_model(model, golden_name) == read_golden(golden_name)
