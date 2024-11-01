from pathlib import Path

import yaml


def write_parameters() -> None:
    parameter_st = ""
    parameter_path = (
        Path(__file__).parents[2].joinpath("trano", "data_models", "parameters.yaml")
    )
    parameters = yaml.safe_load(parameter_path.read_text())
    for name, parameter in parameters.items():
        if not parameter["classes"]:
            continue
        parameter_st += f"""
## {name}
The following parameters are valid for the following classes {",".join([x.lower() for x in parameter["classes"]])}
```yaml
{yaml.dump(parameter["attributes"], default_flow_style=False)}
```
\n
"""
    Path(__file__).parents[2].joinpath("docs/reference/parameters.md").write_text(
        parameter_st
    )


if __name__ == "__main__":
    write_parameters()
