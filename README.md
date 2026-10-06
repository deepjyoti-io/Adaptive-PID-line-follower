# Adaptive PID Line Following Robot

An Arduino Nano-based competitive line-following robot using an 8-channel analog reflectance sensor array, TB6612FNG motor driver and PID control.

The controller combines **automatic sensor calibration, adaptive PID gains, PID smoothing, speed scaling during sharp deviations, controlled line-loss recovery and straight-through intersection handling** to improve tracking stability at higher speeds.

## Project Overview

### Hardware

| Component | Specification |
|---|---|
| Microcontroller | Arduino Nano |
| Motor Driver | TB6612FNG |
| Motors | 2 × N20 geared DC motors |
| Line Sensor | 8-channel analog reflectance sensor array |
| Battery | 2S Li-ion/LiPo pack (~7.4 V nominal) |
| Voltage Regulation | MP1584 buck converter |
| User Input | Push button |
| Status | On-board LED |

### Control Architecture

```text
8-Channel Sensor Array
        │
        ▼
Sensor Calibration
        │
        ▼
Normalized Sensor Values
        │
        ▼
Weighted Position / Error
        │
        ▼
Adaptive PID Controller
   ┌────┼───────────┐
   │    │           │
   ▼    ▼           ▼
  Kp   Ki          Kd
   │    │           │
   └────┼───────────┘
        ▼
 PID Smoothing
        │
        ▼
 Speed Scaling
        │
        ▼
 Differential Motor Control
        │
        ▼
    TB6612FNG
```

## Key Features

- **8-sensor analog line detection** with per-sensor calibration.
- **Weighted position estimation** using sensor weights from `+7` to `-7`.
- **PID control** for continuous steering correction.
- **Adaptive Kp/Kd**:
  - Lower gains near the center to reduce oscillation.
  - Higher gains during larger deviations for faster recovery.
- **PID output smoothing** using an exponential filter to reduce abrupt motor commands.
- **Dynamic base-speed reduction** during sharp turns.
- **Line-loss recovery** based on the last valid tracking error.
- **Post-recovery stabilization** to reduce oscillation after reacquiring the line.
- **Straight-through intersection detection** when multiple sensors are active and both outer sensors detect the line.
- **Single-button state machine** for calibration, ready and final-run modes.
- **Status LED feedback** for operating state.

## State Machine

```text
              ┌───────────────┐
              │     IDLE      │
              └───────┬───────┘
                      │ Button
                      ▼
              ┌───────────────┐
              │  CALIBRATION  │
              └───────┬───────┘
                      │ ~4 seconds
                      ▼
              ┌───────────────┐
              │     READY     │
              └───────┬───────┘
                      │ Button
                      ▼
              ┌───────────────┐
              │   FINAL RUN   │
              └───────────────┘

CALIBRATION / FINAL RUN ── Button ──► IDLE
```

## Pin Configuration

### TB6612FNG

| Arduino Nano | TB6612FNG |
|---|---|
| D4 | AIN1 |
| D3 | AIN2 |
| D6 | BIN1 |
| D7 | BIN2 |
| D9 | PWMA |
| D10 | PWMB |
| D5 | STBY |
| GND | GND |

### User Interface

| Arduino Nano | Function |
|---|---|
| D11 | Push button (`INPUT_PULLUP`) |
| D13 | Status LED |

### Sensor Array

The eight analog sensor channels are read as:

```text
A0  A1  A2  A3  A4  A5  A6  A7
+7  +5  +3  +1  -1  -3  -5  -7
```

The sign convention determines the direction of the PID correction. If the robot turns in the wrong direction, verify sensor orientation and motor polarity before changing the PID gains.

## How It Works

### 1. Calibration

When the robot is in `IDLE`, pressing the button starts a roughly four-second calibration routine.

The robot rotates in place while recording the minimum and maximum ADC values for every sensor:

```cpp
sensorMin[i]
sensorMax[i]
```

The raw readings are later mapped into a normalized `0–1000` range.

### 2. Position Estimation

Each active sensor contributes a predefined positional weight:

```cpp
{7, 5, 3, 1, -1, -3, -5, -7}
```

The average of the active sensor weights becomes the tracking error.

A value near zero means the line is approximately centered beneath the sensor array.

### 3. PID Control

The controller calculates:

```text
P = error
I = accumulated error
D = error - previousError
```

The motor correction is:

```text
PID = Kp × P + Ki × I + Kd × D
```

The resulting correction is applied differentially:

```text
Left Motor  = Base Speed + PID
Right Motor = Base Speed - PID
```

### 4. Adaptive Gains

The controller changes `Kp` and `Kd` depending on the magnitude of the error.

Near the center:

```text
Kp → 0.5 × Kp
Kd → 0.6 × Kd
```

For large deviations:

```text
Kp → 1.2 × Kp
Kd → 1.3 × Kd
```

This aims to reduce oscillation on straight sections while retaining stronger correction in turns.

### 5. Speed Scaling

The base speed is reduced when the tracking error becomes large:

```text
|error| > 3.5  → 60% base speed
|error| > 5.5  → 30% base speed
```

This prevents the robot from entering a sharp turn at the same speed used on a straight section.

### 6. Line-Loss Recovery

If none of the eight sensors detects the line, the robot uses the last valid error to decide the recovery direction.

It then performs a controlled pivot until the line is detected again.

### 7. Intersection Handling

When at least five sensors are active and both outer sensors detect the line, the robot treats the pattern as an intersection and briefly drives straight.

**Important:** this implementation does not perform left/right route selection or path mapping. It is specifically a straight-through intersection strategy.

## PID Tuning

The current starting values are:

```cpp
Kp = 17.0
Ki = 0.0
Kd = 150.0
```

These values are not universal. They depend on:

- Sensor height and spacing
- Track surface and line width
- Motor characteristics
- Gear ratio
- Battery voltage
- Robot mass
- Wheel traction
- Sensor sampling rate

For a new track or hardware configuration, tune `Kp` and `Kd` before introducing integral control.

A practical tuning sequence is:

1. Set `Ki = 0`.
2. Increase `Kp` until the robot responds quickly but begins to oscillate.
3. Increase `Kd` to reduce the oscillation.
4. Tune maximum speed.
5. Tune line-loss recovery.
6. Only introduce a small `Ki` if steady-state error remains.

## Getting Started

### Requirements

- Arduino IDE
- Arduino Nano
- USB cable
- TB6612FNG
- 8-channel analog reflectance sensor array
- 2 DC gear motors
- Suitable battery and regulator

### Upload

1. Open `src/adaptive_pid_line_follower.ino` in Arduino IDE.
2. Select **Tools → Board → Arduino AVR Boards → Arduino Nano**.
3. Select the correct processor if required by your Nano.
4. Select the correct COM port.
5. Upload the sketch.
6. Place the robot over the track.
7. Press the button to start calibration.
8. After calibration, the robot enters `READY`.
9. Press the button again to start the final run.

## Repository Structure

```text
adaptive-pid-line-follower/
├── src/
│   └── adaptive_pid_line_follower.ino
├── README.md
├── LICENSE
└── .gitignore
```

## Limitations

This repository documents the current controller honestly:

- The intersection routine always drives straight; it does not select a branch.
- PID parameters are manually tuned.
- Calibration is performed at startup rather than stored in EEPROM.
- The integral gain is currently zero.
- The controller does not include wheel encoders, IMU feedback or closed-loop wheel-speed control.
- Track-specific performance depends heavily on sensor geometry, motor matching and mechanical alignment.

These are natural areas for future development.

## Possible Improvements

- Add wheel encoders for closed-loop velocity control.
- Add EEPROM storage for calibrated sensor ranges.
- Replace blocking `delay()` calls with non-blocking timing.
- Add configurable track navigation and junction decision logic.
- Add telemetry through serial or wireless communication.
- Add automated PID parameter tuning.
- Add battery-voltage monitoring.
- Add a lightweight diagnostic mode for sensor visualization.

## Author

**Deepjyoti Dutta**  
B.Tech — Electronics & Telecommunication Engineering  
Assam Engineering College

## License

This project is released under the MIT License. See `LICENSE`.
