# 🚀 Vertical Landing Rocket

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![PlatformIO](https://img.shields.io/badge/PlatformIO-Teensy%204.1-orange)](https://platformio.org/)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)

A complete open-source **thrust vector controlled (TVC) vertical landing rocket** control system, inspired by SpaceX's Falcon 9 propulsive landing. Features a distributed architecture with a Teensy 4.1 flight controller running at 1000Hz and an NVIDIA Jetson Nano for high-level guidance and computer vision.

<p align="center">
  <img src="docs/images/rocket_render.png" alt="Rocket Render" width="400"/>
</p>

## 🎯 Features

- **Real-time Flight Control** - 1000Hz control loop with cascaded PID controllers
- **Thrust Vector Control** - Dual-axis gimbaled motor mount for active stabilization
- **Sensor Fusion** - Extended Kalman Filter combining IMU, barometer, and GPS
- **Computer Vision Landing** - ArUco marker detection for precision landing
- **ROS2 Integration** - Modular nodes for guidance, vision, and telemetry
- **Ground Control Station** - Professional PyQt6 dashboard with 3D visualization
- **Software-In-The-Loop** - Complete flight simulator for testing without hardware
- **Safety Systems** - Watchdog timers, abort modes, and pre-flight checks

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        VERTICAL LANDING ROCKET                          │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  ┌──────────────────────┐         ┌──────────────────────────────────┐ │
│  │   TEENSY 4.1         │  UART   │      JETSON NANO                 │ │
│  │   Flight Controller  │◄───────►│      Guidance Computer           │ │
│  │   (1000Hz Loop)      │         │      (ROS2 Humble)               │ │
│  ├──────────────────────┤         ├──────────────────────────────────┤ │
│  │ • BMI088 IMU         │         │ • Guidance Node (Trajectory)     │ │
│  │ • BMP390 Barometer   │         │ • Vision Node (Landing Target)   │ │
│  │ • NEO-M9N GPS        │         │ • Telemetry Node                 │ │
│  │ • Extended Kalman    │         │ • Serial Bridge                  │ │
│  │ • Cascaded PID       │         │                                  │ │
│  │ • TVC Control        │         │                                  │ │
│  │ • State Machine      │         │                                  │ │
│  │ • SD Card Logger     │         │                                  │ │
│  └──────────────────────┘         └──────────────────────────────────┘ │
│           │                                    │                        │
│           │ PWM                                │ UDP                    │
│           ▼                                    ▼                        │
│  ┌──────────────────────┐         ┌──────────────────────────────────┐ │
│  │   TVC SERVOS         │         │   GROUND CONTROL STATION         │ │
│  │   (MG996R x2)        │         │   (PyQt6 + OpenGL)               │ │
│  └──────────────────────┘         └──────────────────────────────────┘ │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

## 📦 Hardware Requirements

| Component | Model | Purpose |
|-----------|-------|---------|
| Flight Controller | Teensy 4.1 | 600MHz ARM Cortex-M7, real-time control |
| Companion Computer | Jetson Nano 4GB | Vision processing, ML guidance |
| IMU | BMI088 | 6-DOF motion sensing, vibration resistant |
| Barometer | BMP390 | Altitude measurement, ±0.25m accuracy |
| GPS | u-blox NEO-M9N | Position/velocity, 25Hz update |
| TVC Servos | MG996R (x2) | Gimbal actuation, 15kg-cm torque |
| Camera | Raspberry Pi Camera v2 | Landing target detection |
| Radio | LoRa SX1276 | Telemetry downlink |
| Battery | 2S 450mAh LiPo | Avionics power |

See [docs/hardware/BOM.md](docs/hardware/BOM.md) for complete bill of materials with purchase links.

## 🚀 Quick Start

### Prerequisites

```bash
# Install PlatformIO CLI
pip install platformio

# Install Python dependencies for GCS
cd gcs
pip install -r requirements.txt

# For Jetson Nano - Install ROS2 Humble
# Follow: https://docs.ros.org/en/humble/Installation.html
```

### Build Firmware

```bash
cd firmware
pio run                    # Build firmware
pio run -t upload          # Upload to Teensy 4.1
```

### Run Ground Control Station

```bash
cd gcs
python main.py
```

### Run Simulation

```bash
cd simulation
python rocket_sim.py --visualize
```

### Build ROS2 Workspace (Jetson Nano)

```bash
cd jetson
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 launch rocket_guidance rocket.launch.py
```

## 📂 Project Structure

```
vertical_landingrocket/
├── firmware/           # Teensy 4.1 flight controller (PlatformIO)
│   ├── src/
│   │   ├── sensors/    # IMU, barometer, GPS drivers
│   │   ├── estimation/ # Kalman filter, quaternion math
│   │   ├── control/    # PID, attitude, TVC controllers
│   │   ├── state/      # Flight state machine
│   │   └── comms/      # Serial protocol
│   └── include/        # Configuration headers
├── jetson/             # Jetson Nano ROS2 workspace
│   └── src/rocket_guidance/
├── gcs/                # Ground Control Station (PyQt6)
├── simulation/         # Software-in-the-loop simulator
├── docs/               # Documentation
│   ├── hardware/       # BOM, wiring diagrams
│   └── api/            # Protocol specifications
└── tools/              # Utilities (PID tuner, log analyzer)
```

## 🛠️ Flight Phases

The state machine manages the following flight phases:

```
     ┌──────┐
     │ IDLE │ ◄── Power on, systems check
     └──┬───┘
        │ ARM command
        ▼
     ┌──────┐
     │ARMED │ ◄── Waiting for launch
     └──┬───┘
        │ Ignition
        ▼
     ┌──────┐
     │BOOST │ ◄── Motor burning, TVC active
     └──┬───┘
        │ Burnout detected
        ▼
     ┌──────┐
     │COAST │ ◄── Ballistic arc, attitude hold
     └──┬───┘
        │ Apogee detected
        ▼
     ┌───────┐
     │DESCENT│ ◄── Controlled descent, TVC active
     └──┬────┘
        │ Landing detected
        ▼
     ┌──────┐
     │LANDED│ ◄── Safe state
     └──────┘

        * ABORT can be triggered from any state *
```

## 📊 Control System

### Cascaded PID Architecture

```
                    ┌─────────────────────────────────────────────────────┐
                    │              ATTITUDE CONTROLLER                     │
                    ├─────────────────────────────────────────────────────┤
                    │                                                      │
  Target      ┌─────┴─────┐   Rate    ┌───────────┐  Gimbal   ┌─────────┐ │
  Attitude ──►│  Outer    │──Setpoint─►│  Inner    │──Command──►│  TVC    │─┼──► Servos
              │  Loop     │           │  Loop     │           │  Mixer  │ │
  Current     │  (PID)    │  Current  │  (PID)    │           │         │ │
  Attitude ──►│           │  Rate ───►│           │           │         │ │
              └───────────┘           └───────────┘           └─────────┘ │
                    │                                                      │
                    └─────────────────────────────────────────────────────┘
```

### Extended Kalman Filter

- **State Vector (12 elements)**:
  - Position (x, y, z)
  - Velocity (vx, vy, vz)
  - Attitude quaternion (qw, qx, qy, qz)
  - Angular rates (p, q, r)

- **Sensors Fused**:
  - IMU (accelerometer + gyroscope) @ 1000Hz
  - Barometer @ 100Hz
  - GPS @ 25Hz

## ⚠️ Safety Features

- Hardware watchdog timer with 100ms timeout
- Software ARM/DISARM sequence with confirmation
- Geofencing with configurable boundaries
- Maximum altitude/velocity limits
- Battery voltage monitoring with low-voltage abort
- Pre-flight sensor self-test
- Abort mode with immediate motor cutoff

## 🧪 Testing

### Unit Tests (Firmware)

```bash
cd firmware
pio test -e native
```

### Python Tests (GCS)

```bash
cd gcs
pytest tests/ -v
```

### ROS2 Tests

```bash
cd jetson
colcon test --packages-select rocket_guidance
```

