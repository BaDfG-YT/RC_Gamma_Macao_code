# -*- coding: utf-8 -*-
"""
Чтение и запись конфига.

config/config.defaults.json — общие значения (в git).
config/config.local.json    — локальные переопределения этого робота (в .gitignore).
load_config() возвращает defaults, поверх которых глубоко наложен local.
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
    """JSON с компактными числовыми массивами (HSV в одну строку, таблицы по 16)."""
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
    Сохраняет HSV-диапазон и min_area цели (ball / gateB / gateY)
    в vision.detection.<target>. where: "defaults" или "local".
    Остальное содержимое файла не трогается.
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
