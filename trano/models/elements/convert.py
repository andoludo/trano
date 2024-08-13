import os
import subprocess
import tempfile
from collections import namedtuple
from functools import partial
from pathlib import Path
from types import SimpleNamespace
from typing import Callable, List, Dict, Any

import yaml


from trano.models.elements.base import (
    DynamicComponentTemplate,
    Port,
    LibraryData,
    BaseParameter,
    exclude_parameters,
)
from pydantic import create_model


def convert_copy(schema: Path, input_file: Path, target: str, output: Path) -> bool:
    root_path = Path(__file__).parents[3]
    os.chdir(root_path)
    command = [
        "poetry",
        "run",
        "linkml-convert",
        "-o",
        f"{output}",
        "-t",
        target,
        "-C",
        "Building",
        "-s",
        str(schema),
        f"{input_file}",
    ]

    process = subprocess.run(
        command, check=True, capture_output=True, text=True  # noqa: S603
    )
    return process.returncode == 0


def convert_boiler():

    boiler = Path("/home/aan/Documents/trano/trano/models/elements/models/boiler.yaml")
    with tempfile.NamedTemporaryFile(mode="w+", suffix=".yaml", delete=False) as f:
        convert_copy(
            Path("/home/aan/Documents/trano/trano/models/elements/element.yaml"),
            boiler,
            "yaml",
            Path(f.name),
        )
        data = yaml.safe_load(Path(f.name).read_text())
    components = []
    for component in data["components"]:
        dynamic_component = DynamicComponentTemplate(**component["component_template"])
        ports = []
        for port in component["ports"]:
            ports.append(Port(**port))
        component_ = create_model(
            "BaseBoiler",
            __base__=LibraryData,
            template=(str, f"{component['template']}"),
            ports_factory=(Callable[[], List[Port]], lambda: ports),
            component_template=(DynamicComponentTemplate, dynamic_component),
            parameter_processing=(
                Callable[[BaseParameter], Dict[str, Any]],
                partial(
                    exclude_parameters,
                    exclude_parameters={
                        "sca_fac_rad",
                        "dt_boi_nominal",
                        "dt_rad_nominal",
                    },
                ),
            ),
        )
        components.append(component_)
    return components[0]


def boiler_parameters():
    boiler = Path(__file__).parents[3].joinpath("tests", "boiler.yaml")
    # data = yaml.safe_load(boiler.read_text())
    with tempfile.NamedTemporaryFile(mode="w+", suffix=".yaml", delete=False) as f:
        convert_copy(
            Path("/home/aan/Documents/trano/trano/models/elements/element.yaml"),
            boiler,
            "yaml",
            Path(f.name),
        )
        data = yaml.safe_load(Path(f.name).read_text())
        component_ = create_model(
            "CustomBoilerParameters",
            __base__=BaseParameter,
            **{
                k: (float if isinstance(v, float) else str, v)
                for k, v in data["components"][0]["parameters"].items()
            },
        )
    return component_()
