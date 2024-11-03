import json
import os
from pathlib import Path
from typing import IO, Any

import yaml


class Loader(yaml.SafeLoader):
    """YAML Loader with `!include` constructor."""

    def __init__(self, stream: IO) -> None:  # type: ignore
        """Initialise Loader."""

        try:
            self._root = os.path.split(stream.name)[0]
        except AttributeError:
            self._root = os.path.curdir

        super().__init__(stream)


def include_default(loader: Loader, node: yaml.Node) -> Any:  # noqa: ANN401
    filename = Path(__file__).parent.joinpath("default.yaml")

    return _load(filename)


def _load(filename: Path) -> Any:  # noqa: ANN401
    with filename.open("r") as f:
        if filename.suffix in (".yaml", ".yml"):
            return yaml.load(f, Loader)  # noqa: S506
        elif filename.suffix == ".json":
            return json.load(f)
        else:
            return "".join(f.readlines())


yaml.add_constructor(
    "!include_default",
    include_default,
    Loader,
)
