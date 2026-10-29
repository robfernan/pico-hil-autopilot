# Aero-RTOS HIL Autopilot

Hardware-in-the-loop flight dynamics on a Raspberry Pi Pico 2 (**RP2350**): the chip integrates a quadrotor model, runs the attitude loop and safety state machine, and an **SDL2** ground station shows whether they hold when you arm, tilt, or inject a fault.

![Aero-RTOS ground station: artificial horizon, pitch and roll traces, and four motor gauges](docs/cockpit_screenshot.png)

The panel is the scan a student pilot learns to read — horizon, heading, altitude, and a motor gauge that drops out — drawn from the same packets the firmware is computing. The cockpit runs on the bench by itself, and binds a Pico when one enumerates on USB.

## Run the ground station

Debian or Ubuntu:

```bash
sudo apt update && sudo apt install -y build-essential cmake libsdl2-dev
./run_ground_station.sh
```

The first launch configures and builds `ground_station/`. Later launches start the cockpit directly.

| Key | What it does |
| --- | --- |
| `Space` / `A` | Arm or disarm. Armed flight uses a hover thrust mix. |
| `Left` / `Q`, `Right` / `E` | Bank. Setpoints stop at ±40°. |
| `Up` / `W`, `Down` / `S` | Pitch. Setpoints stop at ±35°. |
| `1` | Gyro bias fault. |
| `2` | Cut motor 3. |
| `3` | Link-timeout fault. |
| `R` | Clear the fault and level the stick setpoints. |
| `Esc` | Quit. |

## Flash the RP2350 firmware

`./build_firmware.sh` looks for the [Pico SDK](https://github.com/raspberrypi/pico-sdk) at `PICO_SDK_PATH`, or at `~/.pico-sdk/pico-sdk` if that variable is unset. A normal Pico SDK toolchain (`gcc-arm-none-eabi`, newlib) has to be on `PATH`.

```bash
./build_firmware.sh
```

The UF2 is written to `firmware/build_rp2350/pico_hil_autopilot.uf2`.

1. Hold **BOOTSEL** while plugging in the Pico 2 W.
2. Copy the UF2 onto the drive that appears (the script uses `/media/$USER/RP2350/`):

```bash
cp firmware/build_rp2350/pico_hil_autopilot.uf2 /media/$USER/RP2350/
```

3. The board reboots as a USB serial device. The ground station scans `/dev/ttyACM0`–`2` and `/dev/ttyUSB0` and binds when one opens.

## How the loop is split

```
Pico 2 / Pico 2 W — RP2350 bare-metal C
  Core 0  dynamics and control
          2 ms step: Euler attitude, body rates, vertical speed and altitude
          cascaded PID (angle -> rate) and Quad-X motor mix
          STANDBY -> ARMED -> FLIGHT -> FAILSAFE -> EMERGENCY_KILL
  Core 1  USB CDC frames (0xAA 0x55, CRC-16-CCITT)
          fast telemetry every 10 ms, slow status every 100 ms
          hardware watchdog, 500 ms
                    |
                 USB CDC
                    |
Linux ground station — SDL2, accelerated renderer, vsync, ~16 ms frame delay
  horizon, heading, altitude
  pitch / roll strip chart
  four motor gauges in PWM permille
  arm, stick, and fault keys
  local attitude display when no Pico is attached
```

On the chip, tilt past **40°** or a body rate past **250 deg/s** enters failsafe and the hover thrust is reduced. Past **75°** of roll or pitch the motors are cut until disarm.

## Layout

```
pico-hil-autopilot/
├── build_firmware.sh          # RP2350 UF2 build (needs the Pico SDK)
├── run_ground_station.sh      # configure, build, and launch the cockpit
├── docs/cockpit_screenshot.png
├── firmware/                  # bare-metal C: model, PID, state machine, USB frames
└── ground_station/            # SDL2 cockpit, embedded 8x8 font, serial reader thread
```

If this bench is useful, a star helps other people find it.

## License

MIT
