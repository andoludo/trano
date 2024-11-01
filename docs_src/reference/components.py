from pathlib import Path

import yaml


def write_components() -> None:
    components_text = ""
    components = (
        Path(__file__).parents[2].joinpath("trano", "elements", "models").glob("*.yaml")
    )
    classes = {}
    for component in components:
        components_ = yaml.safe_load(component.read_text())["components"]
        classes.update(
            {
                cl: [
                    {
                        "variant": c_["variant"],
                        "library": c_["library"],
                        "template": c_["template"],
                    }
                    for c_ in components_
                    if cl in c_["classes"]
                ]
                for c in components_
                for cl in c["classes"]
            }
        )
    for name, components in classes.items():
        components_text += f"""
## {name}
"""
        for component in components:
            components_text += f"""
### Variant: {component["variant"]} from {component["library"].upper()}
The following template is used for this component:
```jinja
{component["template"]}
```
\n
"""
    Path(__file__).parents[2].joinpath("docs/reference/components.md").write_text(
        components_text
    )


if __name__ == "__main__":
    write_components()
