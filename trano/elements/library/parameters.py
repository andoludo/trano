"""Parameter classes of the elements, built from ``data_models/parameters.yaml``.

Every attribute of the YAML is a field of a pydantic model. Besides the LinkML keys
(``description``, ``range``, ``ifabsent``) an attribute carries trano's own keys:

- ``alias``: the Modelica name in the reference library (Buildings); the field name when absent.
- ``libraries``: the Modelica name per library when it differs (see :class:`LibraryMapping`).
- ``short_name``: a shorter name accepted in the YAML for the same parameter.
- ``render``: ``false`` for a parameter trano uses itself, never written to a Modelica model.
- ``numerical``: ``true`` for a simulation setting rather than a physical property.
- ``deprecated``: a message logged when the parameter is given.
- ``default_from``: an expression of ``self`` giving the value when the parameter is absent (the
  air change rate from the airtightness, the occupancy gains from the gains per person).

A parameter class may list the ``libraries`` it serves (the mpc elements): the others do not take
any of its parameters.

:func:`library_parameters` turns a parameter object into the ``name=value`` pairs a component
template renders for a library: it maps every field to the library's name and drops the fields
the library does not take and the fields left unset, so that an absent parameter is the library's
own default.
"""

import logging
import re
from pathlib import Path
from typing import Any, ClassVar, Optional

import yaml
from pydantic import BaseModel, ConfigDict, Field, computed_field, create_model, model_validator

from trano.elements.common_base import BaseParameter
from trano.elements.utils import _get_default, _get_type

logger = logging.getLogger(__name__)

PRIORITY = ["DataSource"]
LIBRARIES = ("buildings", "ideas", "iso_13790", "reduced_order", "mpc")
SPEC_KEYS = ("alias", "libraries", "short_name", "render", "numerical", "deprecated", "default_from")
_UNIT = re.compile(r"\[([^\]]+)\]\s*$")


class LibraryMapping(BaseModel):
    """How one library takes a parameter.

    In the YAML a library entry is a Modelica name, ``null`` (the library does not take the
    parameter) or a mapping with ``name`` and the options below.
    """

    model_config = ConfigDict(frozen=True)
    name: str | None = Field(default=None, description="Modelica name; None when the library does not take it")
    when_given: bool = Field(default=False, description="Rendered only when the user set the parameter")
    variants: tuple[str, ...] | None = Field(default=None, description="Rendered only for these variants")
    handled: bool = Field(
        default=False, description="Taken another way: by the component template or through another parameter"
    )

    @classmethod
    def parse(cls, value: Any) -> "LibraryMapping":  # noqa: ANN401
        if value is None:
            return cls()
        if isinstance(value, str):
            return cls(name=value)
        if isinstance(value, dict):
            return cls(**value)
        raise TypeError(f"A library mapping is a name, null or a mapping, not {value!r}")

    def applies(self, variant: str | None, given: bool) -> bool:
        if self.name is None or self.handled:
            return False
        if self.when_given and not given:
            return False
        return not (self.variants and variant not in self.variants)


class ParameterSpec(BaseModel):
    """What the YAML says about one parameter beyond its type and default."""

    model_config = ConfigDict(frozen=True)
    name: str
    alias: str | None = None
    libraries: dict[str, LibraryMapping] = Field(default_factory=dict)
    short_name: str | None = None
    render: bool = True
    numerical: bool = False
    deprecated: str | None = None
    default_from: str | None = None
    computed: bool = False
    description: str | None = None
    default: Any = None

    @classmethod
    def from_attribute(
        cls, name: str, attribute: dict[str, Any], class_libraries: list[str] | None = None
    ) -> "ParameterSpec":
        alias = attribute.get("alias")
        alias = None if alias in (None, "None", "null") else alias
        libraries = {
            str(library).lower(): LibraryMapping.parse(mapping)
            for library, mapping in (attribute.get("libraries") or {}).items()
        }
        if class_libraries is not None:
            served = {str(library).lower() for library in class_libraries}
            libraries = {library: LibraryMapping() for library in LIBRARIES if library not in served} | libraries
        unknown = set(libraries) - set(LIBRARIES)
        if unknown:
            raise ValueError(f"Parameter {name}: unknown libraries {sorted(unknown)}, expected {LIBRARIES}")
        return cls(
            name=name,
            alias=alias,
            libraries=libraries,
            short_name=attribute.get("short_name"),
            render=attribute.get("render", True),
            numerical=attribute.get("numerical", False),
            deprecated=attribute.get("deprecated"),
            default_from=attribute.get("default_from"),
            computed="func" in attribute,
            description=attribute.get("description"),
            default=_get_default(attribute) if "range" in attribute else None,
        )

    @property
    def modelica_name(self) -> str:
        """The reference name: the alias, or the field name for a parameter without one."""
        return self.alias or self.name

    @property
    def unit(self) -> str | None:
        match = _UNIT.search(self.description or "")
        return match.group(1) if match else None

    def mapping(self, library: str | None) -> LibraryMapping:
        if not self.render:
            return LibraryMapping()
        if library is not None and library.lower() in self.libraries:
            return self.libraries[library.lower()]
        return LibraryMapping(name=self.modelica_name)


class SpecifiedParameter(BaseParameter):
    """A parameter class carrying its specs; accepts the short names and logs the deprecations."""

    __parameter_specs__: ClassVar[dict[str, ParameterSpec]] = {}

    @model_validator(mode="before")
    @classmethod
    def _accept_short_names(cls, data: Any) -> Any:  # noqa: ANN401
        if not isinstance(data, dict):
            return data
        data = dict(data)
        for spec in cls.__parameter_specs__.values():
            if spec.short_name and spec.short_name in data:
                value = data.pop(spec.short_name)
                if spec.name in data and data[spec.name] != value:
                    raise ValueError(f"{spec.short_name} and {spec.name} are the same parameter, given with two values")
                data[spec.name] = value
            if spec.deprecated and (spec.name in data or spec.alias in data):
                logger.warning("Parameter %s is deprecated: %s", spec.name, spec.deprecated)
        return data

    @model_validator(mode="after")
    def _derive_defaults(self) -> "SpecifiedParameter":
        """Fill an absent parameter from the ones it derives from; the derived value counts as given."""
        for spec in self.__parameter_specs__.values():
            if spec.default_from and not _given(self, spec.name, getattr(self, spec.name, None)):
                value = eval(spec.default_from, {"self": self})  # noqa: S307
                if value is not None:
                    setattr(self, spec.name, value)
        return self

    @classmethod
    def specs(cls) -> dict[str, ParameterSpec]:
        return cls.__parameter_specs__


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
        specs = {
            k: ParameterSpec.from_attribute(k, v, parameter.get("libraries"))
            for k, v in parameter["attributes"].items()
        }
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
        base: type[BaseParameter] = type(f"{name}_specified_", (SpecifiedParameter,), {"__parameter_specs__": specs})
        if computed_attrib_:
            base = type(f"{name}_computed_", (base,), computed_attrib_)
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


def parameter_specs(parameters: BaseParameter | type[BaseParameter]) -> dict[str, ParameterSpec]:
    """The specs of a parameter object or class; empty for a plain :class:`BaseParameter`."""
    return getattr(parameters, "__parameter_specs__", {})


def library_parameters(
    parameters: BaseParameter, library: str | None = None, variant: str | None = None
) -> dict[str, Any]:
    """The ``name=value`` pairs a component template renders for a library.

    Each field set to a value is written under the library's name for it. A field the library
    does not take is dropped, with a warning when the user had set it. A field left unset (None)
    is dropped too, so the library's own default applies. Values given under names the schema
    does not know (``extra="allow"``) are rendered as they are.
    """
    if not parameters:
        return {}
    specs = parameter_specs(parameters)
    rendered: dict[str, Any] = {}
    for name, value in parameters.model_dump().items():
        if value is None or name == "data":
            continue
        spec = specs.get(name)
        if spec is None:
            rendered[name] = value
            continue
        given = _given(parameters, name, value)
        mapping = spec.mapping(library)
        if mapping.applies(variant, given):
            rendered[mapping.name] = value  # type: ignore[index]
        elif given and spec.render and not mapping.handled and library is not None:
            logger.warning(
                "Parameter %s is not taken by the %s library%s: ignored",
                name,
                library,
                f" for the {variant} variant" if mapping.variants else "",
            )
    return rendered


def _given(parameters: BaseParameter, name: str, value: Any) -> bool:  # noqa: ANN401
    """Whether the user set the parameter: the YAML conversion fills the schema defaults in, so a
    value equal to the default counts as absent."""
    field = type(parameters).model_fields.get(name)
    if field is None:
        return False  # a computed field follows the parameters it is computed from
    return name in parameters.model_fields_set and value != field.default


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


def modify_alias(parameter: BaseParameter, modify_alias: dict[str, str], **_: Any) -> Any:  # noqa: ANN401
    """Render only the listed fields, under the given names (variant-specific wrappers)."""
    return change_alias(parameter, modify_alias)(**parameter.model_dump()).model_dump(
        by_alias=True, include=set(modify_alias), exclude_none=True
    )


def exclude_parameters(
    parameters: BaseParameter,
    exclude_parameters: set[str] | None = None,
    **_: Any,  # noqa: ANN401
) -> dict[str, Any]:
    """Render every field but the listed ones, under the reference names."""
    return parameters.model_dump(by_alias=True, exclude=exclude_parameters, exclude_none=True)


def default_parameters(
    parameters: BaseParameter, library: str | None = None, variant: str | None = None
) -> dict[str, Any]:
    """The default processing: the library mapping of the schema."""
    return library_parameters(parameters, library, variant)


__all__ = [
    "LIBRARIES",
    "PARAMETERS",
    "SPEC_KEYS",
    "LibraryMapping",
    "ParameterSpec",
    "SpecifiedParameter",
    "default_parameters",
    "exclude_parameters",
    "library_parameters",
    "modify_alias",
    "param_from_config",
    "parameter_specs",
]
