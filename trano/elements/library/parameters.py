from pathlib import Path
from typing import Any, Optional

import yaml
from pydantic import ConfigDict, Field, computed_field, create_model, BaseModel

from trano.elements.common_base import BaseParameter
from trano.elements.utils import _get_default, _get_type

PRIORITY = ["DataSource"]


def load_parameters() -> dict[str, type["BaseParameter"]]:
    # TODO: remove absoluth path reference
    parameter_path = Path(__file__).parents[2].joinpath("data_models", "parameters.yaml")
    data = yaml.safe_load(parameter_path.read_text())
    classes: dict[str, type[BaseParameter]] = {}

    order = {name: i for i, name in enumerate(PRIORITY)}
    models: dict[str, type[BaseModel]] = {}
    for name, parameter in sorted(
        data.items(),
        key=lambda kv: (order.get(kv[0], len(PRIORITY)), kv[0]),
    ):
        attrib_ = {}
        computed_attrib_ = {}
        for k, v in parameter["attributes"].items():
            alias = v.get("alias", None)
            alias = alias if alias != "None" else None
            multivalued = v.get("multivalued", False)
            if v.get("range"):
                attrib_[k] = (
                    get_parameters_type(v["range"], models, multivalued),
                    Field(
                        default=_get_default(v),
                        alias=alias,
                        description=v.get("description", None),
                    ),
                )
            else:
                # TODO: avoid using eval
                computed_attrib_[k] = computed_field(
                    return_type=eval(v["type"]),  # noqa: S307
                    alias=alias,
                )(eval(v["func"]))  # noqa: S307
        # create_model does not accept computed fields as field definitions, so they
        # are attached through an intermediate class built by pydantic's metaclass.
        base: type[BaseParameter] = BaseParameter
        if computed_attrib_:
            base = type(f"{name}_computed_", (BaseParameter,), computed_attrib_)
        model = create_model(f"{name}_", __base__=base, **attrib_)  # type: ignore # TODO: why?
        models[name] = model
        if parameter.get("classes") is None:
            continue
        for class_ in parameter["classes"]:
            classes[class_] = model  # type: ignore[assignment]
    return classes


def get_parameters_type(_type: str, models: dict[str, type[BaseModel]], multivalued: bool = False) -> Any:  # noqa: ANN401
    try:
        type_ = _get_type(_type)
    except Exception:
        type_ = models.get(_type)
    return list[type_] if multivalued else type_  # type: ignore


PARAMETERS = load_parameters()


def param_from_config(name: str) -> type[BaseParameter] | None:
    if name in PARAMETERS:
        return PARAMETERS[name]
    elif name.upper() in PARAMETERS:
        return PARAMETERS[name.upper()]
    else:
        return None
    # TODO: to be replaced with a raise later


def change_alias(parameter: BaseParameter, mapping: dict[str, str] | None = None) -> Any:  # noqa: ANN401
    mapping = mapping or {}
    new_param = {}
    for name, field in parameter.model_fields.items():
        if mapping.get(name):
            field.alias = mapping[name]
        new_param[name] = (
            Optional[field.annotation] if getattr(parameter, name) is None else field.annotation,  # noqa: UP045
            Field(field.default, alias=field.alias, description=field.description),
        )

    for name, field in parameter.model_computed_fields.items():  # type: ignore
        if mapping.get(name):
            new_param[name] = (
                Optional[field.return_type],  # type: ignore  # noqa: UP045
                Field(None, alias=mapping[name], description=field.description),
            )
    return create_model(  # type: ignore
        "new_model",
        **new_param,
        __config__=ConfigDict(populate_by_name=True),
    )


def modify_alias(parameter: BaseParameter, modify_alias: dict[str, str]) -> Any:  # noqa: ANN401
    return change_alias(parameter, modify_alias)(**parameter.model_dump()).model_dump(
        by_alias=True, include=set(modify_alias)
    )


def exclude_parameters(parameters: BaseParameter, exclude_parameters: set[str] | None = None) -> dict[str, Any]:
    return parameters.model_dump(by_alias=True, exclude=exclude_parameters)


def default_parameters(parameters: BaseParameter) -> dict[str, Any]:
    if not parameters:
        return {}
    return parameters.model_dump(by_alias=True, exclude_none=True)
