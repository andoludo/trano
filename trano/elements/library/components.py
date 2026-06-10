from functools import cache
from pathlib import Path
from typing import Any

import yaml
from pydantic import BaseModel


from trano.elements.library.base import LibraryData


class Components(BaseModel):
    components: list[dict[str, Any]]

    @classmethod
    def load(cls) -> "Components":
        libraries_json_path = Path(__file__).parent.joinpath("models")
        components = []
        for file in libraries_json_path.rglob("*.yaml"):
            file_data = yaml.safe_load(file.read_text())
            components += file_data

        return cls(components=components)

    def get_components(self, component_name: str) -> list[LibraryData]:
        libraries_data = [LibraryData.model_validate(c) for c in self.components if component_name in c["classes"]]
        if len({(ld.variant, ld.library) for ld in libraries_data}) != len(libraries_data):
            raise ValueError(f"Duplicate variant for {component_name}")
        return libraries_data


@cache
def get_components_registry() -> Components:
    """Load the component library lazily: parsing every YAML file is slow."""
    return Components.load()
