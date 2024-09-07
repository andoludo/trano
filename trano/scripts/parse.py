import re
from pathlib import Path
from typing import Any, Dict, List

import yaml

# HOW TO RUN # : ERA001
# material_folders = [ # : ERA001
#     Path("../libraries/IDEAS/IDEAS/Buildings/Data/Materials"), # noqa : ERA001
#     Path("../libraries/IDEAS/IDEAS/Buildings/Data/Insulation"), # noqa : ERA001
#     Path("../libraries/IDEAS/IDEAS/Buildings/Data/Frames"), # noqa : ERA001
# ] # : ERA001
# glazing_folder = Path("../libraries/IDEAS/IDEAS/Buildings/Data/Glazing") # noqa : ERA001
# construction_folder = Path( # : ERA001
#     "../libraries/IDEAS/IDEAS/Buildings/Data/Constructions" # noqa : ERA001
# ) # : ERA001
# parser(material_folders, glazing_folder, construction_folder) # noqa : ERA001


def parse_materials(input_str: str) -> List[Dict[str, Any]]:
    pattern = re.compile(
        r"(IDEAS\.Buildings\.Data\.Materials\.\w+|Materials\.\w+)"
        r"\(d=([\d.]+)(?:,epsLw_a=([\d.]+))?(?:,epsLw_b=([\d.]+))?\)"
    )

    matches = pattern.findall(input_str)
    if not matches:
        raise ValueError("Input string does not match the expected format")

    materials = []
    for match in matches:
        material_type = match[0].split(".")[-1].upper() + ":001"
        thickness = float(match[1])
        type = "glass" if "GLASS" in material_type else "gas"
        material_dict = {
            type: material_type,
            "thickness": thickness,
        }
        materials.append(material_dict)

    return materials


def parse_gas_and_material(input_str: str) -> Dict[str, Any]:
    pattern = r"record\s+(.*?)\s*\""
    match = re.search(pattern, input_str, re.DOTALL)

    if not match:
        raise ValueError("Input string does not match the expected format")
    text = match.group(1).replace("\n", "").replace("\r", "").replace(" ", "")

    pattern_material = re.compile(
        r"(\w+)\s*=\s*IDEAS\.Buildings\.Data\.Interfaces\.(Insulation|Material)\s*\(\s*"
        r"k\s*=\s*([\d.]+)\s*,\s*"
        r"c\s*=\s*([\d.]+)\s*,\s*"
        r"rho\s*=\s*([\d.]+)\s*"
        r"(?:,\s*epsLw\s*=\s*([\d.]+))?\s*"
        r"(?:,\s*epsSw\s*=\s*([\d.]+))?\s*"
        r"(?:,\s*gas\s*=\s*(true|false))?\s*"
        r"(?:,\s*nu\s*=\s*([\d.eE\*]+))?\s*\)"
    )
    pattern_gas = re.compile(
        r"(\w+)\s*=\s*IDEAS\.Buildings\.Data\.Interfaces\.(Insulation|Material)\s*\(\s*"
        r"k\s*=\s*([\d.]+)\s*,\s*"
        r"c\s*=\s*([\d.]+)\s*,\s*"
        r"rho\s*=\s*([\d.]+)\s*"
        r"(?:,\s*epsSw\s*=\s*([\d.]+))?\s*"
        r"(?:,\s*epsLw\s*=\s*([\d.]+))?\s*"
        r"(?:,\s*gas\s*=\s*([\s\S]*))?\s*"
        r"(?:,\s*nu\s*=\s*([\s\S]*))?\s*\)"
    )

    matches = pattern_material.findall(text)
    if not matches:
        matches = pattern_gas.findall(text)
    if not matches:
        raise ValueError("Input string does not match the expected format")

    materials = []
    for match in matches:
        material_type = match[0].upper() + ":001"  # type: ignore
        thermal_conductivity = float(match[2])  # type: ignore
        specific_heat_capacity = float(match[3])  # type: ignore
        density = float(match[4])  # type: ignore
        longwave_emissivity = float(match[5]) if match[5] else None  # type: ignore
        shortwave_emissivity = float(match[6]) if match[6] else None  # type: ignore
        material_dict = {
            "id": material_type,
            "thermal_conductivity": thermal_conductivity,
            "specific_heat_capacity": specific_heat_capacity,
            "density": density,
        }
        if longwave_emissivity is not None:
            material_dict.update({"longwave_emissivity": longwave_emissivity})
        if shortwave_emissivity is not None:
            material_dict.update({"shortwave_emissivity": shortwave_emissivity})
        materials.append(material_dict)
    return materials[0]


def material_parser(folders: List[Path]) -> Dict[str, Any]:
    mo_files = [file for folder in folders for file in list(folder.glob("**/*.mo"))]
    materials: Dict[str, List[Dict[str, Any]]] = {
        "gas": [],
        "material": [],
        "glass_material": [],
    }
    for file in mo_files:
        try:
            results = parse_gas_and_material(file.read_text())
            if results["id"] == "GLASS:001":
                materials["glass_material"].append(results)
            elif any(g in results["id"] for g in ["AIR", "ARGON", "XENON", "KRYPTON"]):
                materials["gas"].append(results)
            else:
                materials["material"].append(results)
        except Exception as e:  # noqa: PERF203
            print(f"Material {file.stem} cannot be generated. Reason {e}")
    return materials


def parse_glazing(input_str: str) -> Dict[str, Any]:
    pattern = re.compile(
        r"record\s+(\w+)\s*=\s*IDEAS\.Buildings\.Data\.Interfaces\.Glazing\s*\(\s*"
        r"final\s+nLay\s*=\s*(\d+),\s*"
        r"(?:final\s+checkLowPerformanceGlazing\s*=\s*(true|false),\s*)?"
        r"final\s+mats\s*=\s*\{([\s\S]*?)\},\s*"
        r"final\s+SwTrans\s*=\s*\[([\s\S]*?)\],\s*"
        r"final\s+SwAbs\s*=\s*\[([\s\S]*?)\],\s*"
        r"final\s+SwTransDif\s*=\s*([\d.]+),\s*"
        r"final\s+SwAbsDif\s*=\s*\{([\s\S]*?)\},\s*"
        r"final\s+U_value\s*=\s*([\d.]+),\s*"
        r"final\s+g_value\s*=\s*([\d.]+)\s*\)"
    )

    match = pattern.search(input_str)
    if not match:
        raise ValueError("Input string does not match the expected format")

    record_id = match.group(1)
    n_lay = int(match.group(2))
    check_low_performance_glazing = match.group(3) == "true" if match.group(3) else None
    mats = match.group(4).replace("\n", "").replace(" ", "")
    sw_trans = match.group(5).replace("\n", "").replace(" ", "")
    sw_abs = match.group(6).replace("\n", "").replace(" ", "")
    sw_trans_dif = float(match.group(7))
    sw_abs_dif = match.group(8).replace("\n", "").replace(" ", "")
    u_value = float(match.group(9))
    g_value = float(match.group(10))

    glazing_dict = {
        "id": record_id,
        "nLay": n_lay,
        "checkLowPerformanceGlazing": check_low_performance_glazing,
        "mats": parse_materials(mats),
        "SwTrans": sw_trans,
        "SwAbs": sw_abs,
        "SwTransDif": sw_trans_dif,
        "SwAbsDif": sw_abs_dif,
        "U_value": u_value,
        "g_value": g_value,
    }

    return glazing_dict


def glazing_parser(folder: Path) -> List[Dict[str, Any]]:
    mo_files = list(folder.glob("*.mo"))
    glazing = []
    for f in mo_files:
        try:
            record = parse_glazing(f.read_text())
            glazing.append(
                {"layers": record["mats"], "id": record["id"].upper() + ":001"}
            )
        except Exception as e:  # noqa: PERF203
            print(f"Glazing {f.stem} cannot be generated. Reason {e}")
    return glazing


def parse_constructions(input_str: str) -> List[Dict[str, Any]]:
    pattern = re.compile(
        r'record\s+(\w+)\s*"([^"]*)"\s*'
        r"extends\s+IDEAS\.Buildings\.Data\.Interfaces\.Construction\s*\(\s*"
        r"(?:locGain\s*=\s*\{[\d,]*\}\s*,\s*)?"
        r"(?:incLastLay\s*=\s*[\w\.]+\s*,\s*)?"
        r"mats\s*=\s*\{([^}]*)\}\s*\)\s*;"
    )

    material_pattern = re.compile(
        r"IDEAS\.Buildings\.Data\.(Materials|Insulation)\.(\w+)\(d=([\d.]+)\)"
    )

    matches = pattern.findall(input_str)
    if not matches:
        raise ValueError("Input string does not match the expected format")

    constructions = []
    for match in matches:
        record_name = match[0]
        materials_str = match[2]

        materials = []
        material_matches = material_pattern.findall(materials_str)
        for material_match in material_matches:
            material_name = material_match[1]
            thickness = float(material_match[2])
            materials.append(
                {"material": material_name.upper() + ":001", "thickness": thickness}
            )

        construction_dict = {
            "id": record_name.upper() + ":001",
            "layers": materials,
        }
        constructions.append(construction_dict)

    return constructions


def construction_parser(folder: Path) -> List[Dict[str, Any]]:
    mo_files = list(folder.glob("*.mo"))
    constructions = []
    for f in mo_files:
        try:
            record = parse_constructions(f.read_text())
            constructions.extend(record)
        except Exception as e:  # noqa: PERF203
            print(f"Construction cannot be generated. Reason {e}")
    return constructions


def parser(
    material_folders: List[Path], glazing_folder: Path, construction_folder: Path
) -> None:
    materials = material_parser(material_folders)
    constructions = construction_parser(construction_folder)
    glazing = glazing_parser(glazing_folder)
    default = {"constructions": constructions, "glazings": glazing, **materials}
    p = Path(__file__).parents[1].joinpath("data/default.yaml")
    with p.open(mode="w+") as f:
        yaml.safe_dump(default, f)
