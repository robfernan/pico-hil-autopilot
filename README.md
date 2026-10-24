# Aero-RTOS HIL Autopilot & Telemetry Ground Station

An industrial-grade **Hardware-in-the-Loop (HIL)** flight dynamics supervisor and flight control computer running baremetal **FreeRTOS SMP** on the Raspberry Pi Pico 2 W (**RP2350 Dual ARM Cortex-M33**), interfaced to a real-time **LVGL 9** Primary Flight Display (PFD) and diagnostic ground station running on Linux.

---

## Architecture Overview

```
+-----------------------------------------------------------------------------------------+
|                               PICO 2 W (RP2350 Baremetal C++)                           |
|                                                                                         |
|  [CORE 0: Deterministic Control Loop]          [CORE 1: Safety & Telemetry Broker]       |
|  * FreeRTOS Task (500 Hz):                     * FreeRTOS Task (100 Hz):                |
|    - 6-DOF Quadrotor/AMR Kinematics Model        - USB CDC Packet Framer (COBS / CRC16) |
|    - Cascaded PID Attitude & Velocity Loop       - Hardware Watchdog Timer (WDT)        |
|    - Deterministic State Machine:                - Fault Injection Engine (Simulate     |
|      IDLE -> ARMED -> FLIGHT -> FAILSAFE           sensor drift, motor cutoff)          |
|                                                                                         |
|                     <==== FreeRTOS Queues & Direct Task Notifications ====>              |
+-----------------------------------------------------------------------------------------+
                                          |
                                   USB CDC Serial
                                          |
+-----------------------------------------------------------------------------------------+
|                                DEBIAN LINUX LAPTOP                                      |
|                                                                                         |
|  [Desktop Window: LVGL 9 + SDL2 Ground Control Station]                                 |
|  * Primary Flight Display (PFD): Real-time Artificial Horizon (pitch/roll/yaw gauge)   |
|  * Multi-channel strip chart: Real-time PID error curves, attitude vs setpoint          |
|  * Interactive Control Deck:                                                            |
|    - Live PID parameter tuning (sends binary packets down to Pico flash/RAM)            |
|    - "Inject Fault" buttons (kill motor 3, corrupt gyro) to prove real-time failsafe    |
+-----------------------------------------------------------------------------------------+
```

---

## Key Engineering Features

- **Dual-Core Asymmetric Processing (FreeRTOS SMP):**
  - **Core 0:** High-frequency, deterministic real-time tasks (500 Hz physics solver, attitude estimation, cascaded PID loop).
  - **Core 1:** Asynchronous communication, binary packet framing (COBS + CRC16), telemetry streaming, and safety supervisor.
- **Hardware-in-the-Loop (HIL) Simulation:**
  - Evaluates autopilot firmware behavior against a real-time mathematical 6-DOF dynamic model running directly on the RP2350 microcontroller.
- **Fault Injection & Active Safety Failsafes:**
  - Dynamic simulation of sensor anomalies (gyro saturation, accelerometer bias drift, motor thrust degradation, link timeouts).
  - Autonomous transitions through a formal safety state machine (`IDLE` &rarr; `ARMED` &rarr; `FLIGHT` &rarr; `FAILSAFE_RTL` &rarr; `EMERGENCY_KILL`).
- **Real-Time Visual Ground Control Station (LVGL 9 on Linux):**
  - Native desktop graphical window powered by LVGL and SDL2.
  - Interactive Artificial Horizon / Primary Flight Display (PFD).
  - Live strip-chart telemetry plotting attitude, angular rates, and control output in real time.
  - Interactive PID parameter tuning and fault-injection triggers.

---

## Repository Structure

```
pico-hil-autopilot/
├── docs/                      # Architectural specs, state diagrams, protocol definition
├── firmware/                  # RP2350 Pico 2 W FreeRTOS baremetal C++ firmware
│   ├── CMakeLists.txt
│   ├── FreeRTOSConfig.h       # FreeRTOS SMP dual-core configuration
│   ├── include/
│   │   ├── protocol.h         # Zero-copy binary packet definitions & CRC16
│   │   └── state_machine.h    # Safety state machine definitions
│   └── src/
│       ├── main.cpp           # Firmware entrypoint & FreeRTOS scheduler launch
│       ├── core0_control.cpp  # 500 Hz PID & 6-DOF kinematics solver
│       └── core1_telemetry.cpp# 100 Hz USB packet framer & fault supervisor
└── ground_station/            # Linux desktop ground station (LVGL 9 + SDL2)
    ├── CMakeLists.txt
    ├── include/
    │   └── ui_cockpit.h       # LVGL UI widgets, PFD horizon, charts
    └── src/
        ├── main.c             # SDL2 event loop & LVGL initialization
        ├── ui_cockpit.c       # Primary Flight Display implementation
        └── serial_bridge.c    # POSIX non-blocking serial communication thread
```

---

## License
MIT
