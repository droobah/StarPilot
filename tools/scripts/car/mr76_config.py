#!/usr/bin/env python3
"""Bench tool for a Nanoradar MR76: find it on the bus and change its sensor ID.

Run it with the radar on a 12 V supply and a USB-CAN adapter at 500 kbit/s, never on the car.
Changing settings sends the radar's config frame (0x200 + 0x10 x sensor ID), which a car
module may use for something else; a release panda, including the comma device's, blocks it.
On the car StarPilot only listens to the radar, so this is only needed when changing settings.

Needs only python-can, so it runs outside the openpilot environment. Add the extra your adapter's
backend needs: serial for slcan adapters such as a CANable with stock firmware, gs_usb for
candleLight firmware (which also needs libusb).

  uv run --with "python-can[serial]" tools/scripts/car/mr76_config.py --interface slcan --channel /dev/tty.usbmodem1101 scan
  uv run --with "python-can[serial]" tools/scripts/car/mr76_config.py --interface slcan --channel /dev/tty.usbmodem1101 set-id 2

The frame layouts below repeat opendbc's nanoradar_mr76 DBC generator, so the tool doesn't need
opendbc; test_mr76_config.py checks that they agree for every sensor ID.
"""
import argparse
import sys
import time
from dataclasses import dataclass

SENSOR_IDS = range(8)
SENSOR_ID_STRIDE = 0x10
RADAR_CFG = 0x200
RADAR_STATE = 0x201
OBJ_LIST_STATUS = 0x60A
SW_VERSION = 0x700

# signal: (least significant bit, size), Motorola byte order
RADAR_CFG_SIGNALS = {
  "SensorID_valid": (1, 1),
  "StoreInNVM_valid": (7, 1),
  "SensorID": (32, 3),
  "StoreInNVM": (47, 1),
}
RADAR_STATE_SIGNALS = {
  "NVMReadStatus": (6, 1),
  "NVMWriteStatus": (7, 1),
  "MaxDistance": (22, 10),  # x2 m
  "SensorID": (32, 3),
  "SortIndex": (36, 3),
  "RadarPower": (39, 3),
  "OutputType": (42, 2),
  "CANBaudRate": (53, 3),
  "RCS_Threshold": (58, 3),
}
SW_VERSION_SIGNALS = {"Major": (0, 8), "Minor": (8, 8), "Patch": (16, 8)}

OUTPUT_TYPES = {0: "none", 1: "objects", 2: "clusters"}
SORT_INDEXES = {0: "none", 1: "range", 2: "rcs"}
LISTEN_S = 3.0


def address(base: int, sensor_id: int) -> int:
  return base + SENSOR_ID_STRIDE * sensor_id


def _bits(lsb: int, size: int):
  """Frame bit positions of a Motorola signal, least significant first."""
  bit = lsb
  for _ in range(size):
    yield bit
    bit = bit + 1 if bit % 8 != 7 else (bit // 8 - 1) * 8


def encode(signals: dict[str, tuple[int, int]], values: dict[str, int]) -> bytes:
  dat = bytearray(8)
  for name, value in values.items():
    lsb, size = signals[name]
    if not 0 <= value < 1 << size:
      raise ValueError(f"{name}={value} does not fit in {size} bits")
    for i, bit in enumerate(_bits(lsb, size)):
      if value >> i & 1:
        dat[bit // 8] |= 1 << (bit % 8)
  return bytes(dat)


def decode(signals: dict[str, tuple[int, int]], dat: bytes) -> dict[str, int]:
  return {name: sum(((dat[bit // 8] >> (bit % 8)) & 1) << i for i, bit in enumerate(_bits(lsb, size)))
          for name, (lsb, size) in signals.items()}


def set_id_frame(current_id: int, new_id: int) -> tuple[int, bytes]:
  """The config frame, sent to the radar's current address, that moves it to new_id and saves it."""
  if current_id not in SENSOR_IDS or new_id not in SENSOR_IDS:
    raise ValueError("sensor IDs are 0-7")
  return address(RADAR_CFG, current_id), encode(RADAR_CFG_SIGNALS, {"SensorID_valid": 1, "SensorID": new_id,
                                                                     "StoreInNVM_valid": 1, "StoreInNVM": 1})


@dataclass
class Radar:
  sensor_id: int
  object_cycles: int = 0
  state: dict[str, int] | None = None
  version: tuple[int, int, int] | None = None

  def describe(self) -> str:
    parts = [f"sensor ID {self.sensor_id}: {self.object_cycles} object cycles"]
    if self.state is None:
      parts.append("no RadarState heard (it is sent about once a second)")
    else:
      s = self.state
      output = OUTPUT_TYPES.get(s["OutputType"], s["OutputType"])
      sort = SORT_INDEXES.get(s["SortIndex"], s["SortIndex"])
      saved = "ok" if s["NVMWriteStatus"] else "not reported"
      parts.append(f"reports ID {s['SensorID']}, max distance {s['MaxDistance'] * 2} m, output {output}, sort {sort}, " +
                   f"RCS threshold {s['RCS_Threshold']}, last save {saved}")
    if self.version is not None:
      parts.append("firmware " + ".".join(str(v) for v in self.version))
    return "; ".join(parts)


def scan(bus, seconds: float = LISTEN_S, clock=time.monotonic) -> dict[int, Radar]:
  """Radars heard on the bus, by sensor ID. bus has recv(timeout) -> (address, data) or None."""
  found: dict[int, Radar] = {}
  by_address = {}
  for i in SENSOR_IDS:
    by_address[address(RADAR_STATE, i)] = (i, "state")
    by_address[address(OBJ_LIST_STATUS, i)] = (i, "cycle")
    by_address[address(SW_VERSION, i)] = (i, "version")
  end = clock() + seconds
  while (remaining := end - clock()) > 0:
    frame = bus.recv(remaining)
    if frame is None or frame[0] not in by_address or len(frame[1]) != 8:
      continue
    sensor_id, kind = by_address[frame[0]]
    radar = found.setdefault(sensor_id, Radar(sensor_id))
    if kind == "state":
      radar.state = decode(RADAR_STATE_SIGNALS, frame[1])
    elif kind == "cycle":
      radar.object_cycles += 1
    else:
      v = decode(SW_VERSION_SIGNALS, frame[1])
      radar.version = (v["Major"], v["Minor"], v["Patch"])
  # A RadarState alone could be another device; a radar sends its object cycle every 60 ms.
  return {i: radar for i, radar in found.items() if radar.object_cycles > 0}


def set_id(bus, new_id: int, current_id: int | None = None, *, confirm=input, out=print, clock=time.monotonic) -> bool:
  radars = scan(bus, clock=clock)
  if current_id is None:
    if len(radars) != 1:
      out(f"Found {len(radars)} radars; connect exactly one, or name it with --current." if radars else
          "No radar heard. Check power, wiring, termination and the bit rate (500 kbit/s).")
      return False
    current_id = next(iter(radars))
  elif current_id not in radars:
    out(f"No radar heard at sensor ID {current_id}.")
    return False
  if current_id == new_id:
    out(f"The radar is already at sensor ID {new_id}.")
    return True

  addr, dat = set_id_frame(current_id, new_id)
  out(f"Will send 0x{addr:03X} {dat.hex(' ')}: sensor ID {current_id} -> {new_id}, saved in the radar.")
  if confirm("Bench only: the radar must not be connected to a car. Type OK to send: ").strip().upper() != "OK":
    out("Nothing sent.")
    return False
  bus.send(addr, dat)

  after = scan(bus, clock=clock)
  radar = after.get(new_id)
  if radar is None:
    out(f"The radar did not come back at sensor ID {new_id}. Heard: {sorted(after) or 'nothing'}.")
    return False
  out("Now " + radar.describe())
  if radar.state is not None and not radar.state["NVMWriteStatus"]:
    out("The radar didn't confirm the save. Power-cycle it and run scan to check the ID stuck.")
    return False
  out("Done. Power-cycle the radar and run scan to confirm it stays at the new ID.")
  return True


class PythonCanBus:
  def __init__(self, interface: str, channel: str | None, bitrate: int):
    try:
      import can
    except ImportError:
      sys.exit('python-can is not installed; run with: uv run --with "python-can[serial]" ' + " ".join(sys.argv))
    self.can = can
    try:
      self.bus = can.Bus(interface=interface, channel=channel, bitrate=bitrate)
    except can.CanInterfaceNotImplementedError as error:
      extra = {"slcan": "serial", "gs_usb": "gs_usb"}.get(interface)
      hint = f' Run with uv run --with "python-can[{extra}]".' if extra else ""
      sys.exit(f"python-can can't use the {interface} backend: {error}.{hint}")
    except (can.CanInitializationError, OSError, ValueError) as error:
      sys.exit(f"Couldn't open the adapter ({interface}, {channel}): {error}. Check the interface name and the port " +
               "(ls /dev/tty.usb* /dev/cu.usb*).")

  def recv(self, timeout: float):
    msg = self.bus.recv(timeout)
    if msg is None or msg.is_extended_id or msg.is_error_frame or msg.is_remote_frame:
      return None
    return msg.arbitration_id, bytes(msg.data)

  def send(self, addr: int, dat: bytes) -> None:
    self.bus.send(self.can.Message(arbitration_id=addr, data=dat, is_extended_id=False))

  def close(self) -> None:
    self.bus.shutdown()


def main() -> int:
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument("--interface", required=True, help="python-can interface, e.g. slcan, gs_usb, pcan, socketcan")
  parser.add_argument("--channel", help="adapter channel, e.g. /dev/tty.usbmodem1101 or can0")
  parser.add_argument("--bitrate", type=int, default=500_000)
  commands = parser.add_subparsers(dest="command", required=True)
  scan_cmd = commands.add_parser("scan", help="list the radars on the bus and their settings")
  scan_cmd.add_argument("--seconds", type=float, default=LISTEN_S)
  set_cmd = commands.add_parser("set-id", help="change the radar's sensor ID and save it")
  set_cmd.add_argument("new_id", type=int, choices=SENSOR_IDS)
  set_cmd.add_argument("--current", type=int, choices=SENSOR_IDS, help="the radar's sensor ID now (default: detect it)")
  args = parser.parse_args()

  bus = PythonCanBus(args.interface, args.channel, args.bitrate)
  try:
    if args.command == "scan":
      radars = scan(bus, args.seconds)
      for radar in radars.values():
        print(radar.describe())
      if not radars:
        print("No radar heard. Check power, wiring, termination and the bit rate (500 kbit/s).")
      return 0 if radars else 1
    return 0 if set_id(bus, args.new_id, args.current) else 1
  finally:
    bus.close()


if __name__ == "__main__":
  sys.exit(main())
