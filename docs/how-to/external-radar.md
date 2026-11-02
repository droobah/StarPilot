# Add an external radar (Nanoradar MR76)

*Experimental. Hardware required: a Nanoradar MR76, a USB-CAN adapter for setup, and a free CAN bus on your comma harness.*

Cars that have no factory radar find leads with the camera alone. StarPilot can add a Nanoradar MR76 77 GHz radar. radard combines its objects with the camera's lead, just as it does with a factory radar.

The radar only adds information. StarPilot listens to it and never sends anything to it, and it never makes the car disengage. If the radar stops working, StarPilot goes back to the camera alone and shows a small alert.

## Which cars can use it

External Radar appears only for cars where:

- StarPilot controls speed (openpilot longitudinal control), and
- there is no factory radar in use.

## 1. Choose the sensor ID

Every MR76 address moves by `0x10` × its sensor ID (0–7). A new radar ships at ID 0, which is also Galaxy's default.

- **Usually no change is needed.** If the radar has a bus to itself, such as the spare bus 1 on the 2018–2021 Bolt, every ID is free. Keep 0 and skip to wiring.
- **Change it only if a car module uses one of its addresses on the bus.** At ID 0 the radar uses `0x200`/`0x201`, the same addresses as the comma pedal, so on a bus shared with a pedal choose another ID. On the Bolt's bus 0, only ID 2 is clear of the pedal and the car's own traffic. If you skip this, StarPilot's startup check refuses to start the radar and tells you which addresses collide.

### Changing it on the bench

Do this before the radar goes anywhere near the car. Changing the ID means sending the radar's config frame. A release panda blocks that frame, including the one in the comma device. Use a USB-CAN adapter that python-can supports (for example CANable, PCAN or gs_usb), at 500 kbit/s. The commands below are for an slcan adapter such as a CANable with stock firmware; for candleLight firmware use `--interface gs_usb` and `python-can[gs_usb]`, which also needs libusb (`brew install libusb` on macOS). Power the radar from a 12 V bench supply, and terminate the bus with 120 Ω at each end.

```sh
uv run --with "python-can[serial]" tools/scripts/car/mr76_config.py --interface slcan --channel /dev/tty.usbmodem1101 scan
uv run --with "python-can[serial]" tools/scripts/car/mr76_config.py --interface slcan --channel /dev/tty.usbmodem1101 set-id 2
```

- `scan` lists every radar it hears, with its sensor ID, maximum distance, output type and sort order.
- `set-id` shows the frame and asks you to type OK before it sends anything. It then checks that the radar comes back at the new ID and has saved it.

Power-cycle the radar and run `scan` again to confirm the new ID stuck.

## 2. Wire and mount it

- **Bus:** use a CAN bus the car leaves free if you can. Before wiring, record a drive and confirm in Cabana that the bus is silent. On the 2018–2021 Bolt with a beartech GM harness, CAN1 is free (beartech confirmed this); ask your harness vendor for the pinout. Use a twisted pair for CAN H/L and keep stubs short.
- **Termination:** with everything off, measure between CAN H and CAN L on the finished bus. About 60 Ω is right. About 120 Ω means one end has no terminator; add 120 Ω at the radar end.
- **Power:** use a fused circuit that switches with the ignition. The radar draws about 0.3 A at 12 V.
- **Mounting:** mount it centered and level, facing straight ahead. Put it behind plastic, never behind metal or metallic paint. Measure the following and enter them in Galaxy:
  - **Forward Offset:** meters from the front bumper to the radar, negative if it sits behind the bumper face.
  - **Lateral Offset:** meters left of the car's centerline, negative to the right.
  - **Yaw:** degrees the radar points left of straight ahead.

## 3. Turn it on in Galaxy

1. Developer → turn on **External Radar Setup**.
2. Vehicle settings → **External Radar**. Set the CAN bus and the offsets, and the sensor ID if you changed it.
3. Set **Mode** to **Passive** and drive. Passive reads and checks the radar but doesn't affect driving.
4. Review the Passive drive, then switch Mode to **Active**.

Settings apply after a restart. To take the radar out, use **Remove Setup** at the bottom of the External Radar page. It deletes the saved setup and turns the Developer switch off.

At startup, StarPilot checks the radar's addresses against the CAN traffic it sees during fingerprinting. If a car module already uses one of them, the radar isn't started and you'll see "External radar not started". Choose another sensor ID or bus.

## What the alerts mean

| Alert | What happened |
|---|---|
| External radar lost — Using vision only | No complete radar cycle for 0.3 s. It recovers on its own after 2 s of good data. |
| External radar off until restart | Lost three times in one drive, or heard at a different sensor ID than the one set in Galaxy. |
| External radar available again | Back after being lost. |
| External radar not detected — Check wiring and ID | Nothing heard for 10 s after starting. |
| External radar not started — Check radar settings | The saved settings are invalid, or the bus or sensor ID collides with the car. |

In Passive mode, the second line says "Passive mode" instead of "Using vision only".

## Limitations

- This is experimental, and the radar isn't part of any safety system. StarPilot never relies on it to stay engaged.
- At low speed, radard can pick a close radar object in your path as the lead even when the camera sees nothing there. Wrong offsets or yaw can cause unexpected braking, so check them on a Passive drive before using Active.
- The left/right sign of the radar's lateral output and the object quality and extended-info messages (`0x60C`, `0x60D`) haven't been confirmed against hardware yet.
- The startup address check only sees traffic sent during fingerprinting. Some modules only transmit while driving, so check a whole drive in Cabana as well.
