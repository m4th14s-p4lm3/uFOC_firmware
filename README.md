# uFOC firmware

Firmware for the uFOC compact BLDC/PMSM motor driver. The firmware implements Field Oriented Control (FOC), cascaded torque/current, velocity and position control loops, CAN communication and Python-side tooling for testing and runtime tuning.

The project targets the uFOC board based on the STM32F302K8U MCU, DRV8316C three-phase gate driver with integrated current sensing, MT6835 magnetic encoder and TCAN332 CAN transceiver. The board is intended for compact servo actuators where only power and CAN bus wiring should be required.

## Repository layout

```text
Core/                     STM32CubeMX generated application and peripheral setup
Drivers/                  STM32 HAL and CMSIS sources
uFOC_lib/
  communication/          CAN RX/TX wrapper and command dispatcher
  config/                 Runtime configuration, limits and unit conversion
  debug_utils/            UART debug output helpers
  driver/                 DRV8316C setup, PWM output and ADC current conversion
  encoder/                MT6835 SPI encoder driver, position unwrap, velocity and calibration
  foc/                    Clarke/Park transforms, inverse transforms, SVPWM and sin/cos LUT
  pi_controller/          PI/PID regulators with saturation and anti-windup
pytools/                  Python CAN client, UI and test scripts
```

## Firmware topology

The firmware is split between STM32CubeMX generated initialization code in `Core/Src/main.c` and reusable modules in `uFOC_lib`.

The fast inner current loop runs in `HAL_ADCEx_InjectedConvCpltCallback()`. Injected ADC conversions are triggered from `TIM1_CC4`, synchronized to the PWM cycle. In that interrupt the firmware:

1. reads the three phase-current ADC samples,
2. converts raw ADC values to phase currents,
3. updates the MT6835 encoder position and electrical angle,
4. runs FOC current control when `control_state >= CURRENT_CONTROL`,
5. applies SVPWM duty cycles through the DRV8316C PWM driver.

TIM1 is configured for center-aligned PWM at approximately 8888 Hz. This is also the current control loop frequency.

The slower outer loops run in `HAL_TIM_PeriodElapsedCallback()` from TIM6 at 2 kHz:

1. `VELOCITY_CONTROL` runs a PI velocity controller and writes the torque-current target for the inner loop.
2. `POSITION_CONTROL` runs a PID position controller and writes the velocity target for the velocity loop.

The control states are ordered intentionally:

```c
NO_CONTROL       = 0
CURRENT_CONTROL  = 1
VELOCITY_CONTROL = 2
POSITION_CONTROL = 3
```

Selecting a higher-level mode enables the lower-level loops below it.

## Configuration

The main firmware configuration is in `uFOC_lib/config/Inc/config.h`.

Motor and encoder parameters:

```c
#define MOTOR_MAGNETIC_PAIRS 11
#define ENCODER_ELECTRICAL_OFFSET 23562
#define ENCODER_INVERT_DIR true
```

`MOTOR_MAGNETIC_PAIRS` must match the motor pole-pair count. `ENCODER_INVERT_DIR` can be flipped when the measured encoder direction is opposite to the motor phase direction. `ENCODER_ELECTRICAL_OFFSET` stores the calibrated electrical zero offset.

Control limits and default gains are also configured there:

```c
#define PI_CURRENT_KP 1.0f
#define PI_CURRENT_KI 1000.0f
#define PI_CURRENT_HARD_LIMIT 0.75f

#define PI_VELOCITY_KP 0.2f
#define PI_VELOCITY_KI 15.0f
#define PI_VELOCITY_HARD_LIMIT 16.0f

#define PI_POSITION_KP 33.0f
#define PI_POSITION_KI 0.0f
#define PI_POSITION_KD 0.12f
```

The hard limits are compile-time safety limits. Runtime soft limits are clamped to those hard limits by the setters in `uFOC_lib/config/Src/config.c`.

Hard limits define the maximum allowed range compiled into the firmware. They are intended as safety boundaries for a specific motor. A CAN client cannot increase a runtime limit above the corresponding hard limit without changing `config.h` and reflashing the firmware.

Soft limits are runtime limits used during normal operation. They can be changed over CAN for tuning or for application-level restrictions, but every setter clamps them to the hard limit. For example, `PI_CURRENT_HARD_LIMIT` defines the absolute maximum torque current allowed by the firmware, while `torque_current_soft_limit` can be lowered at runtime to restrict the current command for a specific test or operating mode. The same idea is used for velocity: `PI_VELOCITY_HARD_LIMIT` is the compiled maximum and `angular_velocity_soft_limit` is the runtime-adjustable limit.

CAN identity:

```c
#define CAN_COMMUNICATION_DEVICE_ID 0x00
#define CAN_COMMUNICATION_STD_ID 0x123
```

For daisy-chained drivers on the same bus, compile each board with a unique `CAN_COMMUNICATION_DEVICE_ID`. The CAN filter currently accepts all frames and the command handler filters by the first data byte.

## Units

Internally, all angular values are stored in rotations. Current is stored in amperes and time in seconds.

The client-facing angle unit can be changed at runtime:

```c
RADIANS   = 0
ROTATIONS = 1
DEGREES   = 2
```

Position values use the selected angle unit. Velocity values use the selected angle unit per second. For example, if `DEGREES` is selected, velocity targets and velocity getters are interpreted as degrees per second. Current targets and current limits are always amperes.

## Electrical offset calibration

Correct electrical offset is required for stable FOC operation. The calibration aligns the encoder mechanical reading with the motor electrical zero.

Recommended workflow:

1. Keep the motor unloaded and mechanically free to move.
2. Set the controller to `NO_CONTROL`.
3. Call `calibrate_electrical_offset(&config.encoder)` or send the `CALIBRATE_ELECTRICAL_OFFSET` CAN command.
4. Read the result with `GET_ELECTRICAL_OFFSET`.
5. Store the value in `ENCODER_ELECTRICAL_OFFSET`.
6. Recompile and flash the firmware.
7. Disable repeated calibration during normal startup unless the mechanical setup allows it.

The current `Core/Src/main.c` startup sequence calls `calibrate_electrical_offset(&config.encoder)` during initialization. For normal use with a stored offset, remove or comment out that startup calibration and keep the calibrated value in `config.h`.

## CAN protocol

CAN is configured for standard 11-bit frames at 500 kbit/s. The default arbitration ID is `0x123`.

Client-to-driver frame:

```text
byte 0      device ID (CAN_COMMUNICATION_DEVICE_ID)
byte 1      command
byte 2..5   optional little-endian float payload
```

For `SET_CONTROL_STATE` and `SET_USER_ANGLE_UNITS`, byte 2 is a single enum value instead of a float.

Float getter response:

```text
byte 0      device ID
byte 1      command
byte 2..5   little-endian float value
```

`GET_USER_ANGLE_UNITS` returns three bytes: device ID, command and the enum value in byte 2.

## CAN commands

| Command | Value | Payload | Description |
| --- | ---: | --- | --- |
| `SET_CONTROL_STATE` | `0x00` | `uint8` | Set `NO_CONTROL`, `CURRENT_CONTROL`, `VELOCITY_CONTROL` or `POSITION_CONTROL`. |
| `SET_USER_ANGLE_UNITS` | `0x01` | `uint8` | Set user-facing angle units: radians, rotations or degrees. |
| `GET_USER_ANGLE_UNITS` | `0x02` | none | Read current user-facing angle unit enum. |
| `SET_TORQUE_CURRENT_TARGET` | `0x06` | `float` | Set q-axis torque current target in amperes. |
| `SET_TORQUE_CURRENT_SOFT_LIMIT` | `0x07` | `float` | Set runtime torque-current limit in amperes. Clamped by `PI_CURRENT_HARD_LIMIT`. |
| `GET_TORQUE_CURRENT_SOFT_LIMIT` | `0x08` | none | Read runtime torque-current limit in amperes. |
| `SET_CURRENT_KP` | `0x09` | `float` | Set both d-axis and q-axis current PI proportional gain. |
| `SET_CURRENT_KI` | `0x0A` | `float` | Set both d-axis and q-axis current PI integral gain. |
| `SET_ANGULAR_VELOCITY_TARGET` | `0x10` | `float` | Set velocity target in selected angle unit per second. |
| `SET_ANGULAR_VELOCITY_SOFT_LIMIT` | `0x11` | `float` | Set runtime velocity limit in selected angle unit per second. |
| `GET_ANGULAR_VELOCITY_SOFT_LIMIT` | `0x12` | none | Read runtime velocity limit in selected angle unit per second. |
| `GET_ANGULAR_VELOCITY` | `0x13` | none | Read filtered angular velocity from the encoder. |
| `SET_ANGULAR_VELOCITY_KP` | `0x14` | `float` | Set velocity PI proportional gain. |
| `SET_ANGULAR_VELOCITY_KI` | `0x15` | `float` | Set velocity PI integral gain. |
| `SET_POSITION_TARGET` | `0x1A` | `float` | Set position target in selected angle unit. |
| `GET_POSITION_TARGET` | `0x1B` | none | Read position target in selected angle unit. |
| `GET_POSITION` | `0x1C` | none | Read unwrapped position in selected angle unit. |
| `GET_RELATIVE_POSITION` | `0x1D` | none | Currently returns `0`; relative position is not implemented yet. |
| `SET_POSITION_KP` | `0x1E` | `float` | Set position PID proportional gain. |
| `SET_POSITION_KI` | `0x1F` | `float` | Set position PID integral gain. |
| `SET_POSITION_KD` | `0x20` | `float` | Set position PID derivative gain. |
| `GET_ELECTRICAL_OFFSET` | `0x26` | none | Read current encoder electrical offset. |
| `SET_ELECTRICAL_OFFSET` | `0x27` | `float` | Update encoder electrical offset at runtime. |
| `CALIBRATE_ELECTRICAL_OFFSET` | `0x28` | none | Run blocking electrical offset calibration. Use only when the motor is not under closed-loop control. |

## Python tools

`pytools/ufoc_client.py` contains a Python client that implements the CAN protocol. It uses `python-can` and defaults to a CANable-style `slcan` interface.

Install dependencies from `pytools`:

```sh
cd pytools
uv sync
```

Minimal example:

```python
from ufoc_client import uFOCclient, ControlState, AngleUnits

client = uFOCclient(
    channel="/dev/tty.usbmodem206F327555481",
    arbitration_id=0x123,
    device_id=0x00,
)

client.set_user_angle_units(AngleUnits.ROTATIONS)
client.set_control_state(ControlState.POSITION_CONTROL)
client.set_position_target(1.0)
print(client.get_position())
client.close()
```

`pytools/ui.py` and `pytools/ui2.py` are interactive tuning/testing tools. `pytools/mirror_mode.py` demonstrates a two-driver daisy-chain setup where one motor is manually moved and the second motor mirrors its position.

## Build

Required tools:

- CMake 3.22 or newer
- Ninja
- `arm-none-eabi-gcc` toolchain available in `PATH`

Configure and build:

```sh
cmake --preset Debug
cmake --build --preset Debug
```

Release build:

```sh
cmake --preset Release
cmake --build --preset Release
```

The CMake project name is `test_foc`, so the ELF output is generated under the selected build directory, for example:

```text
build/Debug/test_foc.elf
```

Flash the produced ELF with your usual STM32 flashing workflow, for example STM32CubeProgrammer, ST-Link tooling or the IDE setup used for the board.

## Extending the firmware

Prefer keeping runtime-adjustable values behind the config abstraction in `uFOC_lib/config/Src/config.c`:

1. Add the field to `config_t` in `uFOC_lib/config/Inc/config.h` if state has to be stored.
2. Add a setter/getter in `config.c`.
3. Clamp user-provided values against hard limits where needed.
4. Add a CAN command ID in `uFOC_lib/communication/Inc/commands.h`.
5. Dispatch the command in `uFOC_lib/communication/Src/commands.c`.
6. Add the matching method to `pytools/ufoc_client.py`.

This keeps safety limits, unit conversion and CAN access in one consistent path.

## Notes and current limitations

- `GET_RELATIVE_POSITION` is a placeholder and currently returns `0`.
- The MT6835 encoder is read synchronously over SPI inside the current-loop interrupt; SPI latency is the main limit for increasing the current-loop rate.
- The IMU present on the uFOC hardware is not used by this firmware.
