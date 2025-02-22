import json
from pathlib import Path
from typing import List, Any, Dict

from pydantic import BaseModel


from trano.elements.library.base import LibraryData


class Components(BaseModel):
    components: List[Dict[str, Any]]

    @classmethod
    def load(cls) -> "Components":
        libraries_json_path = Path(__file__).parent.joinpath("models_json")
        libraries_json_path.mkdir(exist_ok=True)
        components = []
        for file in libraries_json_path.glob("*.json"):
            file_data = json.loads(file.read_text())
            components += file_data.get("components", [])

        return cls(components=components)

    def get_components(self, component_name: str) -> List[LibraryData]:
        return [
            LibraryData.model_validate(c)
            for c in self.components
            if component_name in c["classes"]
        ]


COMPONENTS = Components.load()
