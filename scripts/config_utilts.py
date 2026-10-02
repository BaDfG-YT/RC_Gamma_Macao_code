import json
import pathlib


def deep_merge(base: dict, overrides: dict) -> None:
    for key, value in overrides.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            deep_merge(base[key], value)
        else:
            base[key] = value


def load_config(name: str, folder: pathlib.Path) -> dict:
    """Reads <name>.defaults.json from folder, overlays it with
    <name>.local.json (if present). The same pattern is used
    for config.*, device_id.* and any future defaults/local pairs."""
    with open(folder / f"{name}.defaults.json") as f:
        config = json.load(f)

    local_path = folder / f"{name}.local.json"
    if local_path.exists():
        with open(local_path) as f:
            deep_merge(config, json.load(f))

    return config