# -*- coding: utf-8 -*-
"""LD19 lidar parameters from the config (lidar section): CRC table and initial value."""

import numpy as np

from config_io import load_config

CFG = load_config()["lidar"]

# LD19 CRC-8 table (256 values) and initial CRC value
CRC_TABLE = np.array(CFG["crc_table"], dtype=np.uint8)
CRC_INIT = int(CFG["crc_init"])
