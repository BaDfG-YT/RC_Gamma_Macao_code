# -*- coding: utf-8 -*-
"""Единый источник параметров лидара LD19: config/config.defaults.json (секция lidar)."""

import json
import os

import numpy as np

CONFIG_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..",
    "config",
    "config.defaults.json",
)


def load_lidar_cfg():
    with open(CONFIG_PATH, encoding="utf-8") as f:
        return json.load(f)["lidar"]


_CFG = load_lidar_cfg()

# CRC-8 таблица LD19 (256 значений) и начальное значение CRC
CRC_TABLE = np.array(_CFG["crc_table"], dtype=np.uint8)
CRC_INIT = int(_CFG["crc_init"])
