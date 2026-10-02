# -*- coding: utf-8 -*-
"""
Reading and writing the config.

config/config.defaults.json — shared values (in git).
config/config.local.json    — local overrides for this robot (in .gitignore).
load_config() returns defaults with local deep-merged on top.
"""

import json
import os

CONFIG_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "config"
)
DEFAULTS_PATH = os.path.join(CONFIG_DIR, "config.defaults.json")
LOCAL_PATH = os.path.join(CONFIG_DIR, "config.local.json")


def _read(path):
    try:
        with open(path, encoding="utf-8") as f:
            return json.load(f)
    except FileNotFoundError:
        return {}


def deep_merge(base, over):
    out = dict(base)
    for k, v in over.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = v
    return out


def load_config():
    return deep_merge(_read(DEFAULTS_PATH), _read(LOCAL_PATH))


def _is_scalar(x):
    return not isinstance(x, (dict, list))


def dumps(obj, indent=0):
    """JSON with compact numeric arrays (HSV on one line, tables in rows of 16)."""
    pad = "  " * indent
    pad2 = "  " * (indent + 1)
    if isinstance(obj, dict):
        if not obj:
            return "{}"
        items = [
            f"{pad2}{json.dumps(k, ensure_ascii=False)}: {dumps(v, indent + 1)}"
            for k, v in obj.items()
        ]
        return "{\n" + ",\n".join(items) + "\n" + pad + "}"
    if isinstance(obj, list):
        if all(_is_scalar(x) for x in obj):
            if len(obj) <= 16:
                return "[" + ", ".join(json.dumps(x) for x in obj) + "]"
            rows = [
                pad2 + ", ".join(json.dumps(x) for x in obj[i : i + 16])
                for i in range(0, len(obj), 16)
            ]
            return "[\n" + ",\n".join(rows) + "\n" + pad + "]"
        items = [pad2 + dumps(x, indent + 1) for x in obj]
        return "[\n" + ",\n".join(items) + "\n" + pad + "]"
    return json.dumps(obj, ensure_ascii=False)


def _write(path, data):
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(dumps(data) + "\n")


def save_detection(target, hsv_lo, hsv_hi, min_area, where):
    """
    Saves the HSV range and min_area of a target (ball / gateB / gateY)
    into vision.detection.<target>. where: "defaults" or "local".
    The rest of the file's contents is left untouched.
    """
    if where not in ("defaults", "local"):
        raise ValueError(where)
    path = DEFAULTS_PATH if where == "defaults" else LOCAL_PATH
    data = _read(path)
    det = data.setdefault("vision", {}).setdefault("detection", {})
    entry = det.setdefault(target, {})
    entry["hsv"] = [[int(x) for x in hsv_lo], [int(x) for x in hsv_hi]]
    entry["min_area"] = int(min_area)
    _write(path, data)
    return path


def save_zone(xc, yc, rad_sml, rad_big, where):
    """Saves the ROI in vision."accessible zone": xc, yc, rad_sml, rad_big."""
    if where not in ("defaults", "local"):
        raise ValueError(where)
    path = DEFAULTS_PATH if where == "defaults" else LOCAL_PATH
    data = _read(path)
    zone = data.setdefault("vision", {}).setdefault("accessible zone", {})
    zone["xc"] = int(xc)
    zone["yc"] = int(yc)
    zone["rad_sml"] = int(rad_sml)
    zone["rad_big"] = int(rad_big)
    _write(path, data)
    return path