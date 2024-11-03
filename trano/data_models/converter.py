import logging
import os
from typing import Any, Dict, List, Optional

from linkml.generators.pythongen import PythonGenerator  # type: ignore
from linkml.utils import datautils, validation  # type: ignore
from linkml.utils.datautils import (  # type: ignore
    _get_context,
    _get_format,
    _is_xsv,
    get_loader,
    infer_index_slot,
    infer_root_class,
)
from linkml_runtime.linkml_model import Prefix  # type: ignore
from linkml_runtime.utils import inference_utils  # type: ignore
from linkml_runtime.utils.compile_python import compile_python  # type: ignore
from linkml_runtime.utils.inference_utils import infer_all_slot_values  # type: ignore
from linkml_runtime.utils.schemaview import SchemaView  # type: ignore


def delete_none(_dict: Dict[str, Any]) -> Dict[str, Any]:
    # TODO: this function needs to be reviewed
    """Delete None values recursively from all of the dictionaries"""
    for key, value in list(_dict.items()):
        if isinstance(value, dict):
            delete_none(value)
        elif value is None:
            del _dict[key]
        elif isinstance(value, list):
            for v_i in value:
                if isinstance(v_i, dict):
                    delete_none(v_i)

    return _dict


# TODO: function taken from linkml_runtime.cli.converter
def converter(  # noqa: PLR0915, PLR0912, PLR0913, C901
    input: Optional[str] = None,
    module: Optional[Any] = None,  # noqa: ANN401
    target_class: Optional[str] = None,
    context: Optional[List[Any]] = None,
    output: Optional[str] = None,
    input_format: Optional[str] = None,
    output_format: Optional[str] = None,
    prefix: Optional[str] = None,
    target_class_from_path: Optional[str] = None,
    schema: Optional[str] = None,
    validate: Optional[bool] = None,
    infer: Optional[bool] = None,
    index_slot: Optional[bool] = None,
) -> Dict[str, Any]:
    """
    Converts instance data to and from different LinkML Runtime serialization formats.

    The instance data must conform to a LinkML model, and either a path to a python
    module must be passed, or a path to a schema.

    The converter works by first using a linkml-runtime *loader* to
    instantiate in-memory model objects, then a *dumper* is used to serialize.
    A validation step is optionally performed in between

    When converting to or from RDF, a path to a schema must be provided.

    For more information, see https://linkml.io/linkml/data/index.html
    """
    if prefix is None:
        prefix = []  # type: ignore
    if module is None:
        if schema is None:
            raise Exception("must pass one of module OR schema")
        else:
            python_module = PythonGenerator(schema).compile_module()
    else:
        python_module = compile_python(module)
    prefix_map = {}
    if prefix:
        for p in prefix:
            base, uri = p.split("=")
            prefix_map[base] = uri
    if schema is not None:
        sv = SchemaView(schema)
        if prefix_map:
            for k, v in prefix_map.items():
                sv.schema.prefixes[k] = Prefix(k, v)
                sv.set_modified()
    if target_class is None and target_class_from_path:
        target_class = os.path.basename(input).split("-")[0]  # type: ignore
        logging.info(f"inferred target class = {target_class} from {input}")
    if target_class is None:
        target_class = infer_root_class(sv)
    if target_class is None:
        raise Exception("target class not specified and could not be inferred")
    py_target_class = python_module.__dict__[target_class]
    input_format = _get_format(input, input_format)
    loader = get_loader(input_format)

    inargs = {}
    outargs = {}
    if datautils._is_rdf_format(input_format):
        if sv is None:
            raise Exception("Must pass schema arg")
        inargs["schemaview"] = sv
        inargs["fmt"] = input_format
    if _is_xsv(input_format):
        if index_slot is None:
            index_slot = infer_index_slot(sv, target_class)
            if index_slot is None:
                raise Exception("--index-slot is required for CSV input")
        inargs["index_slot"] = index_slot
        inargs["schema"] = schema
    obj = loader.load(source=input, target_class=py_target_class, **inargs)
    if infer:
        infer_config = inference_utils.Config(
            use_expressions=True, use_string_serialization=True
        )
        infer_all_slot_values(obj, schemaview=sv, config=infer_config)
    if validate:
        if schema is None:
            raise Exception(
                "--schema must be passed in order to validate. Suppress with --no-validate"
            )
        # TODO: use validator framework
        validation.validate_object(obj, schema)

    output_format = _get_format(output, output_format, default="json")
    if output_format == "json-ld":
        if len(context) == 0:  # type: ignore
            if schema is not None:
                context = [_get_context(schema)]
            else:
                raise Exception("Must pass in context OR schema for RDF output")
        outargs["contexts"] = list(context)  # type: ignore
    if output_format in ["rdf", "ttl"]:
        if sv is None:
            raise Exception("Must pass schema arg")
        outargs["schemaview"] = sv
    if _is_xsv(output_format):
        if index_slot is None:
            index_slot = infer_index_slot(sv, target_class)
            if index_slot is None:
                raise Exception("--index-slot is required for CSV output")
        outargs["index_slot"] = index_slot  # type: ignore
        outargs["schema"] = schema  # type: ignore
    converted_data = delete_none(obj._as_dict)
    return converted_data
