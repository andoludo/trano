from pathlib import Path

import yaml


def create_final_schema(
    parameters_path: Path, trano_final_path: Path, trano_path: Path
) -> None:
    trano = yaml.safe_load(trano_path.read_text())
    parameters = yaml.safe_load(parameters_path.read_text())
    for name, parameter in parameters.items():
        parameter.pop("classes")
        parameter__ = {}
        for k, v in parameter["attributes"].items():
            if "func" not in v:
                v.pop("alias", None)
                parameter__[k] = v
        parameter["attributes"] = parameter__
        trano["classes"][name] = parameter
    yaml.dump(trano, trano_final_path.open("w"))
