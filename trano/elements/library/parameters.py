from pathlib import Path
from typing import Any, Dict, Optional, Type

import yaml
from pydantic import ConfigDict, Field, computed_field, create_model

from trano.elements.common_base import BaseParameter
from trano.elements.utils import _get_default, _get_type


def load_parameters() -> Dict[str, Type["BaseParameter"]]:
    # TODO: remove absoluth path reference
    parameter_path = (
        Path(__file__).parents[2].joinpath("data_models", "parameters.yaml")
    )
    data = yaml.safe_load(parameter_path.read_text())
    classes = {}

    for name, parameter in data.items():
        attrib_ = {}
        for k, v in parameter["attributes"].items():
            alias = v.get("alias", None)
            alias = alias if alias != "None" else None
            if v.get("range"):
                attrib_[k] = (
                    _get_type(v["range"]),
                    Field(
                        default=_get_default(v),
                        alias=alias,
                        description=v.get("description", None),
                    ),
                )
            else:
                attrib_[k] = computed_field(  # type: ignore # TODO: why?
                    eval(v["func"]),  # noqa: S307
                    return_type=eval(v["type"]),  # noqa: S307
                    alias=alias,  # TODO: avoid using eval
                )
        model = create_model(f"{name}_", __base__=BaseParameter, **attrib_)  # type: ignore # TODO: why?
        if parameter["classes"] is None:
            continue
        for class_ in parameter["classes"]:
            classes[class_] = model
    return classes


PARAMETERS = load_parameters()


def param_from_config(name: str) -> Optional[Type[BaseParameter]]:
    if name in PARAMETERS:
        return PARAMETERS[name]
    elif name.upper() in PARAMETERS:
        return PARAMETERS[name.upper()]
    else:
        return None
    # TODO: to be replaced with a raise later


def change_alias(
    parameter: BaseParameter, mapping: Optional[Dict[str, str]] = None
) -> Any:  # noqa: ANN401
    mapping = mapping or {}
    new_param = {}
    for name, field in parameter.model_fields.items():
        if mapping.get(name):
            field.alias = mapping[name]
        new_param[name] = (
            field.annotation,
            Field(field.default, alias=field.alias, description=field.description),
        )

    for name, field in parameter.model_computed_fields.items():  # type: ignore
        if mapping.get(name):
            new_param[name] = (
                Optional[field.return_type],  # type: ignore
                Field(None, alias=mapping[name], description=field.description),
            )
    return create_model(  # type: ignore
        "new_model",
        **new_param,
        __config__=ConfigDict(populate_by_name=True),
    )


def modify_alias(
    parameter: BaseParameter, modify_alias: Dict[str, str]
) -> Any:  # noqa: ANN401
    return change_alias(parameter, modify_alias)(**parameter.model_dump()).model_dump(
        by_alias=True, include=set(modify_alias)
    )


def exclude_parameters(
    parameters: BaseParameter, exclude_parameters: Optional[set[str]] = None
) -> Dict[str, Any]:
    return parameters.model_dump(by_alias=True, exclude=exclude_parameters)


def default_parameters(parameters: BaseParameter) -> Dict[str, Any]:
    if not parameters:
        return {}
    return parameters.model_dump(by_alias=True, exclude_none=True)
