import json
import pathlib


def deep_merge(base: dict, overrides: dict) -> None:
    for key, value in overrides.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            deep_merge(base[key], value)
        else:
            base[key] = value


def load_config(name: str, folder: pathlib.Path) -> dict:
    """Читает <name>.defaults.json из folder, накладывает поверх
    <name>.local.json (если есть). Один и тот же паттерн используется
    для config.*, device_id.* и любых будущих defaults/local пар."""
    with open(folder / f"{name}.defaults.json") as f:
        config = json.load(f)

    local_path = folder / f"{name}.local.json"
    if local_path.exists():
        with open(local_path) as f:
            deep_merge(config, json.load(f))

    return config