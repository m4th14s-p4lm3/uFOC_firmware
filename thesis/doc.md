# uFOC Firmware

Embedded Field Oriented Control (FOC) firmware for the **uFOC integrated BLDC driver**.

The firmware is designed specifically for the custom **uFOC hardware platform** built around:

- STM32F3 MCU
- DRV8316C 3-phase gate driver
- MT6835 magnetic encoder
- CAN bus communication

It implements real-time torque, velocity, and position control for BLDC/PMSM motors using cascaded FOC control loops.

---

# Features

- Real-time FOC current control
- Torque / velocity / position control modes
- SVPWM modulation
- Synchronized ADC current sampling
- MT6835 SPI encoder support
- DRV8316C support
- CAN bus communication
- Daisy-chain multi-device support
- Runtime configuration via CAN
- Electrical offset calibration
- Modular firmware structure for extension

---

# Repository Structure

```text
uFOC_lib/
├── communication/    CAN communication layer
├── config/           Global configuration abstraction
├── driver/           DRV8316C + PWM + current sensing
├── encoder/          MT6835 encoder handling
├── foc/              FOC math + SVPWM
└── pi_controller/    PI/PID controllers

src/
└── main.c            Application entry point
```

---

# Control Architecture

The firmware uses cascaded control loops:

```text
Position PID
    ↓
Velocity PI
    ↓
Torque / FOC Current Loop
    ↓
SVPWM
    ↓
Motor Phases
```

## Control Modes

Defined in:

```c
enum ControlState
```

Available modes:

| Mode | Description |
|---|---|
| `NO_CONTROL` | PWM disabled |
| `CURRENT_CONTROL` | Torque/current FOC control |
| `VELOCITY_CONTROL` | Velocity + current control |
| `POSITION_CONTROL` | Position + velocity + current control |

---

# Real-Time Execution

## Current Control Loop

Executed inside:

```c
HAL_ADCEx_InjectedConvCpltCallback()
```

Triggered by:

```text
TIM1 CC4 → ADC Injected Conversion
```

Responsibilities:

- Current sampling
- Encoder update
- Clarke/Park transforms
- PI current regulation
- SVPWM update

Loop frequency:

```text
~8.9 kHz
```

---

## Velocity & Position Loops

Executed inside:

```c
HAL_TIM_PeriodElapsedCallback()
```

Using:

```text
TIM6 @ 2 kHz
```

Responsibilities:

- Velocity PI control
- Position PID control

---

# Hardware Configuration

Main configuration file:

```text
uFOC_lib/config/config.h
```

Important parameters:

```c
MOTOR_MAGNETIC_PAIRS
ENCODER_ELECTRICAL_OFFSET
CAN_COMMUNICATION_DEVICE_ID

PI_CURRENT_KP
PI_CURRENT_KI

PI_VELOCITY_KP
PI_VELOCITY_KI

PI_POSITION_KP
PI_POSITION_KI
PI_POSITION_KD
```

---

# CAN Communication

The firmware exposes a simple CAN command interface for runtime control.

## CAN Frame Format

| Byte | Description |
|---|---|
| 0 | Device ID |
| 1 | Command ID |
| 2-5 | Payload (`float`) |
| 6-7 | Reserved |

---

## Example Workflow

### Set Position Control Mode

```text
DEVICE_ID | SET_CONTROL_STATE | POSITION_CONTROL
```

### Set Target Position

```text
DEVICE_ID | SET_POSITION_TARGET | float
```

### Read Current Position

```text
DEVICE_ID | GET_POSITION
```

---

# Python CAN Client

Example Python tools are located in:

```text
pytools/
```

## Examples

| File | Description |
|---|---|
| `ufoc_client.py` | High-level CAN client |
| `mirror_mode.py` | Motion mirroring demo |

Dependencies:

```bash
pip install python-can
```

Hardware used:

```text
CANable USB-CAN adapter
```

---

# Encoder Calibration

FOC requires correct electrical angle alignment.

Run electrical offset calibration once:

```c
calibrate_electrical_offset()
```

Then store the resulting offset in:

```c
ENCODER_ELECTRICAL_OFFSET
```

Without correct calibration the motor may:

- vibrate
- lose torque
- fail to start
- overheat

---

# Current Sensing

Current sensing is implemented using DRV8316C integrated current sense amplifiers.

Conversion pipeline:

```text
ADC samples
    ↓
Offset compensation
    ↓
Gain conversion
    ↓
Phase currents [A]
```

Configured in:

```c
drv8316_init()
```

CSA gain selection:

```c
drv8316_csa_gain_t
```

---

# Extending the Firmware

The firmware is intentionally modular.

## Adding New CAN Commands

1. Add command ID in:

```text
communication/commands.h
```

2. Implement handling in:

```text
communication/commands.c
```

3. Connect to config setters/getters

---

## Adding New Control Modes

1. Extend:

```c
enum ControlState
```

2. Add logic in:

```c
HAL_TIM_PeriodElapsedCallback()
```

or

```c
HAL_ADCEx_InjectedConvCpltCallback()
```

depending on required update frequency.

---

## Supporting Different Hardware

Hardware-specific logic is isolated into:

- `driver/`
- `encoder/`

Porting to another driver or encoder should mostly require replacing these modules.

---

# Build Environment

Developed using:

- STM32CubeMX
- STM32 HAL
- GCC ARM Toolchain
- C language

---

# Tested Hardware

Validated using:

- iPower GM4108 motors
- 20 V power supply
- CANable interface

Tested functionality:

- Torque control
- Velocity control
- Position control
- Multi-device CAN communication

---

# Known Limitations

- Encoder SPI communication currently limits maximum FOC loop frequency
- IMU support not implemented yet
- No trajectory synchronization between multiple motors
- Anticogging not implemented

---

# Thesis

This firmware was developed as part of the bachelor thesis:

> *FOC Firmware for Integrated BLDC Driver*  
> Mathias Palme  
> Czech Technical University in Prague, 2026

---

# License

Add your preferred license here.