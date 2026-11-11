#!/usr/bin/env python3
"""Nanoradar MR76 77 GHz radar, one DBC per sensor ID (0-7).

Every MR76 address is shifted by 0x10 * sensor ID, so each sensor ID gets its
own DBC: nanoradar_mr76_s<id>_generated.

Layouts follow the vendor's MR76 Communication Protocol and the
BSD-3 MR76_Radar driver (github.com/MoradSayed/MR76_Radar), and were
confirmed on frames from a real radar. Both list each Motorola signal by its
least significant bit; they are converted here to the DBC convention, where a
Motorola start bit is the most significant bit. The MR76 has no object quality
(0x60C) or extended (0x60D) messages. The collision detection messages
(0x400-0x402, 0x408, 0x60E) are left out because nothing here turns that
feature on.
"""

SENSOR_IDS = range(8)
SENSOR_ID_STRIDE = 0x10

RADAR_CFG = 0x200
RADAR_STATE = 0x201
OBJ_LIST_STATUS = 0x60A
OBJ_GENERAL = 0x60B
SW_VERSION = 0x700

# name: (base address, sender, [(signal, lsb, size, factor, offset, unit)])
MESSAGES = {
  "RadarCfg": (RADAR_CFG, "XXX", [
    ("MaxDistance_valid", 0, 1, 1, 0, ""),
    ("SensorID_valid", 1, 1, 1, 0, ""),
    ("RadarPower_valid", 2, 1, 1, 0, ""),
    ("OutputType_valid", 3, 1, 1, 0, ""),
    ("SendQuality_valid", 4, 1, 1, 0, ""),
    ("SendExtInfo_valid", 5, 1, 1, 0, ""),
    ("SortIndex_valid", 6, 1, 1, 0, ""),
    ("StoreInNVM_valid", 7, 1, 1, 0, ""),
    ("MaxDistance", 22, 10, 2, 0, "m"),
    ("SensorID", 32, 3, 1, 0, ""),
    ("OutputType", 35, 2, 1, 0, ""),
    ("RadarPower", 37, 3, 1, 0, ""),
    ("SortIndex", 44, 3, 1, 0, ""),
    ("StoreInNVM", 47, 1, 1, 0, ""),
    ("RCS_Threshold_valid", 48, 1, 1, 0, ""),
    ("RCS_Threshold", 49, 3, 1, 0, ""),
    ("Calibration", 57, 2, 1, 0, ""),
    ("Calibration_valid", 59, 1, 1, 0, ""),
    ("BaudRate_valid", 60, 1, 1, 0, ""),
    ("BaudRate", 61, 3, 1, 0, ""),
  ]),
  "RadarState": (RADAR_STATE, "MR76", [
    ("NVMReadStatus", 6, 1, 1, 0, ""),
    ("NVMWriteStatus", 7, 1, 1, 0, ""),
    ("MaxDistance", 22, 10, 2, 0, "m"),
    ("SensorID", 32, 3, 1, 0, ""),
    ("SortIndex", 36, 3, 1, 0, ""),
    ("RadarPower", 39, 3, 1, 0, ""),
    ("OutputType", 42, 2, 1, 0, ""),
    ("CANBaudRate", 53, 3, 1, 0, ""),
    ("RCS_Threshold", 58, 3, 1, 0, ""),
    ("Calibration", 62, 2, 1, 0, ""),
  ]),
  "ObjListStatus": (OBJ_LIST_STATUS, "MR76", [
    ("NofObjects", 0, 8, 1, 0, ""),
    ("MeasCount", 16, 16, 1, 0, ""),
    ("InterfaceVersion", 28, 4, 1, 0, ""),
  ]),
  "ObjGeneral": (OBJ_GENERAL, "MR76", [
    ("ID", 0, 8, 1, 0, ""),
    ("DistLong", 19, 13, 0.2, -500, "m"),
    ("DistLat", 24, 11, 0.2, -204.6, "m"),
    ("VrelLong", 46, 10, 0.25, -128, "m/s"),
    ("DynProp", 48, 3, 1, 0, ""),
    ("Class", 51, 2, 1, 0, ""),
    ("VrelLat", 53, 9, 0.25, -64, "m/s"),
    ("RCS", 56, 8, 0.5, -64, "dBm2"),
  ]),
  "SwVersion": (SW_VERSION, "MR76", [
    ("Major", 0, 8, 1, 0, ""),
    ("Minor", 8, 8, 1, 0, ""),
    ("Patch", 16, 8, 1, 0, ""),
  ]),
}

VALUES = {
  "RadarCfg": {
    "OutputType": {0: "none", 1: "objects", 2: "clusters"},
    "RadarPower": {0: "standard", 1: "minus_3dB", 2: "minus_6dB", 3: "minus_9dB"},
    "SortIndex": {0: "none", 1: "range", 2: "rcs"},
    "BaudRate": {0: "500k", 1: "250k", 2: "1M"},
  },
  "ObjGeneral": {
    "DynProp": {0: "moving", 1: "stationary", 2: "oncoming", 3: "crossing_left", 4: "crossing_right", 5: "unknown", 6: "stopped"},
    "Class": {0: "point", 1: "vehicle", 2: "reserved_2", 3: "reserved_3"},
  },
}

HEADER = """VERSION ""

NS_ :
\tCM_
\tBA_DEF_
\tBA_
\tVAL_

BS_:

BU_: XXX MR76
"""


def motorola_msb(lsb: int, size: int) -> int:
  """DBC start bit (MSB) of a Motorola signal given by its least significant bit."""
  bit = lsb
  for _ in range(size - 1):
    bit = bit + 1 if bit % 8 != 7 else (bit // 8 - 1) * 8
  return bit


def address(base: int, sensor_id: int) -> int:
  return base + SENSOR_ID_STRIDE * sensor_id


def dbc_name(sensor_id: int) -> str:
  return f"nanoradar_mr76_s{sensor_id}_generated"


def _dbc(sensor_id: int) -> str:
  parts = [HEADER]
  for name, (base, sender, signals) in MESSAGES.items():
    parts.append(f"\nBO_ {address(base, sensor_id)} {name}: 8 {sender}\n")
    for signal, lsb, size, factor, offset, unit in signals:
      low, high = offset, offset + factor * ((1 << size) - 1)
      parts.append(f' SG_ {signal} : {motorola_msb(lsb, size)}|{size}@0+ ({factor},{offset}) [{low:g}|{high:g}] "{unit}" XXX\n')
  parts.append("\n")
  for name, signals in VALUES.items():
    for signal, values in signals.items():
      described = " ".join(f'{value} "{label}"' for value, label in values.items())
      parts.append(f"VAL_ {address(MESSAGES[name][0], sensor_id)} {signal} {described} ;\n")
  return "".join(parts)


def generate():
  return {f"nanoradar_mr76_s{sensor_id}.dbc": _dbc(sensor_id) for sensor_id in SENSOR_IDS}
