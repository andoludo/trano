from pathlib import Path
from typing import List, Any, Dict

import yaml
from pydantic import BaseModel


from trano.elements.library.base import LibraryData


class Components(BaseModel):
    components: List[Dict[str, Any]]

    @classmethod
    def load(cls) -> "Components":
        libraries_json_path = Path(__file__).parent.joinpath("models")
        libraries_json_path.mkdir(exist_ok=True)
        components = []
        for file in libraries_json_path.rglob("*.yaml"):
            file_data = yaml.safe_load(file.read_text())
            components += file_data

        return cls(components=components)

    def get_components(self, component_name: str) -> List[LibraryData]:
        libraries_data = [
            LibraryData.model_validate(c)
            for c in self.components
            if component_name in c["classes"]
        ]
        if len({(ld.variant, ld.library) for ld in libraries_data}) != len(
            libraries_data
        ):
            raise ValueError(f"Duplicate variant for {component_name}")
        return libraries_data


COMPONENTS = Components.load()
