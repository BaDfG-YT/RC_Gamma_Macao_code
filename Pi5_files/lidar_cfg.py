# -*- coding: utf-8 -*-
"""Параметры лидара LD19 из конфига (секция lidar): CRC-таблица и начальное значение."""

import numpy as np

from config_io import load_config

CFG = load_config()["lidar"]

# CRC-8 таблица LD19 (256 значений) и начальное значение CRC
CRC_TABLE = np.array(CFG["crc_table"], dtype=np.uint8)
CRC_INIT = int(CFG["crc_init"])
