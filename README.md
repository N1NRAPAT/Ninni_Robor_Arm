# Ninni Robot Arm 🦾

**Summer Project** · Started: 21 June 2026 · Author: Ninrapat Suttinual

An open-source 5-DOF desktop robot arm integrating **STM32 firmware**, **ROS2**, **Docker**, and **CUDA**-accelerated computer vision — built from scratch as a hands-on study of robotics, kinematics, and real-time perception.

---

## 🎯 Project Goals

| # | Goal | Status |
|---|------|--------|
| 1 | Build and assemble a 5-joint physical robot arm with 3D-printed structure | 🔧 In progress |
| 2 | Implement forward kinematics (FK) — compute end-effector pose from joint angles | 📐 Planned |
| 3 | Implement inverse kinematics (IK) — solve joint angles from target XYZ pose | 📐 Planned |
| 4 | Integrate ROS2 with URDF model and visualize in RViz2 before physical execution | ✅ ROS2 installed |
| 5 | Develop CUDA-accelerated computer vision pipeline (real-time object detection) | 🔧 In progress |
| 6 | Connect CV output to IK solver for vision-guided grasping | 🔮 Future |
| 7 | Run MoveIt2 for motion planning and collision avoidance | 🔮 Future |
| 8 | Open-source the full stack for anyone learning robot arm development | 🎯 Ongoing |

---

## 🧠 Theory & Concepts

### Forward Kinematics (FK)
Given a set of joint angles **θ = [θ₁, θ₂, θ₃, θ₄, θ₅]**, FK computes the position and orientation of the end-effector (gripper) in 3D space using **Denavit–Hartenberg (DH) parameters**. Each joint transformation is represented as a 4×4 homogeneous transformation matrix:

```
T = Rot(z, θ) · Trans(0,0,d) · Trans(a,0,0) · Rot(x, α)
```

The final end-effector pose is the product of all joint transforms: **T_total = T₁ · T₂ · T₃ · T₄ · T₅**

### Inverse Kinematics (IK)
Given a target position **(x, y, z)** and orientation, IK solves for the joint angles required. For a 5-DOF arm this problem may be underdetermined (infinite solutions) or may require numerical solvers. Approaches explored:
- **Geometric/analytical IK** — closed-form solution for specific configurations
- **Jacobian-based numerical IK** — iterative solver via pseudo-inverse of the Jacobian matrix
- **MoveIt2 IK plugins** — leverage KDL or TRAC-IK within the ROS2 ecosystem

### ROS2 Architecture
The system uses a publish/subscribe model across nodes:
- `joint_state_publisher` → publishes current joint states
- `robot_state_publisher` → broadcasts URDF transforms (TF tree)
- `moveit2` → motion planning server
- `cv_node` → CUDA vision pipeline, publishes detected object poses
- `serial_bridge` → relays ROS2 joint commands to STM32 over USB serial

### CUDA Computer Vision
Real-time perception runs on an RTX 5080 GPU inside a Docker container. The pipeline:
1. Capture frames from Logitech USB webcam via `usb_cam` ROS2 node
2. Run object detection / pose estimation (CUDA-accelerated)
3. Publish target pose to ROS2 topic → feed into IK solver

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────┐
│              Windows PC (Host)              │
│  ┌─────────────────┐  ┌──────────────────┐  │
│  │   WSL2 Ubuntu   │  │  Docker Container│  │
│  │   22.04         │  │  CUDA 12.8       │  │
│  │                 │  │                  │  │
│  │  RViz2 (WSLg)   │  │  MoveIt2         │  │
│  │  ROS2 Humble    │◄─►  CV Pipeline     │  │
│  │  Serial Bridge  │  │  Kinematics Node │  │
│  └────────┬────────┘  └──────────────────┘  │
│           │ USB Serial (/dev/ttyACM0)        │
└───────────┼─────────────────────────────────┘
            │
   ┌────────▼──────────────┐
   │  NUCLEO-F446RE (STM32)│
   │  Servo Executor        │
   │  Half-duplex UART bus  │
   └────────┬──────────────┘
            │ TTL Bus (daisy-chain)
   ┌────────▼──────────────┐
   │  5× Feetech STS3215   │
   │  Servos (12V, 30kg·cm)│
   └───────────────────────┘
```

---

## 🔌 STM32 NUCLEO-F446RE — Pin Mapping

| Pin | Label | Connected To | Notes |
|-----|-------|-------------|-------|
| PA9 | USART1_TX | Servo Bus TX | Half-duplex TTL UART to STS3215 chain |
| PA10 | USART1_RX | Servo Bus RX | Half-duplex receive |
| PA8 | GPIO OUT | Bus Direction Pin | HIGH = TX, LOW = RX (half-duplex control) |
| CN1 USB | USB Device / ST-Link | PC via USB cable | Serial bridge to ROS2 (`/dev/ttyACM0`) |
| VIN (CN6) | 12V Power In | External 12V PSU | Powers STM32 + logic side |
| GND | Ground | Shared ground | Common with servo power rail |
| PA0 | GPIO (optional) | Status LED / Debug | Heartbeat or fault indicator |
| PB6 | USART1_TX alt | (Reserve) | Alt pin if PA9 conflicts |

> **Note:** The Feetech STS3215 uses a **single-wire half-duplex UART** bus. TX and RX share the same wire via the Waveshare Bus Servo Adapter (A). The direction-control GPIO must toggle before each read/write operation.


## 🧰 Hardware BOM

| Component | Model | Qty | Unit Price (USD) | Notes |
|-----------|-------|-----|-----------------|-------|
| Smart Servo | Feetech STS3215 | 5 | ~$15 | 12V, 30 kg·cm, TTL half-duplex, 360° |
| Servo Bus Adapter | Waveshare Bus Servo Adapter (A) | 1 | ~$10 | USB-to-TTL bridge |
| MCU | STM32 NUCLEO-F446RE | 1 | ~$20 | 180 MHz, ST-Link onboard |
| Webcam | Logitech USB Camera | 1 | — | ROS2 `usb_cam` compatible |
| Power Supply | 12V / 8–10A switching PSU | 1 | ~$15 | Powers all servos |
| PC (GPU) | RTX 5080 host | 1 | — | CUDA 12.8 Docker, RViz2 via WSLg |

---

## 🗺️ Development Roadmap

### Phase 1 — Infrastructure ✅
- [x] ROS2 Humble installed on PC (WSL2 Ubuntu 22.04)
- [x] CUDA 12.8 verified in Docker (RTX 5080, `sm_89`)
- [x] RViz2 running via WSLg
- [x] Architecture finalized: PC + Docker + STM32

### Phase 2 — Robot Description & Visualization 🔧
- [ ] Write URDF for 5-DOF arm
- [ ] Launch `robot_state_publisher` + `joint_state_publisher_gui`
- [ ] Visualize full arm in RViz2 with interactive sliders
- [ ] Assign unique IDs to all 5 STS3215 servos

### Phase 3 — Firmware & Hardware Loop
- [ ] STM32 PlatformIO firmware: receive joint commands over USB serial
- [ ] Half-duplex UART driver for STS3215 bus
- [ ] Round-trip test: ROS2 → serial → STM32 → servo moves

### Phase 4 — Kinematics
- [ ] Implement FK using DH parameters
- [ ] Implement analytical IK for the 5-DOF configuration
- [ ] Validate FK/IK in simulation vs. physical arm

### Phase 5 — Computer Vision
- [ ] CUDA object detection pipeline (camera → detected pose)
- [ ] Publish target pose as ROS2 topic
- [ ] Connect vision output to IK → arm reaches detected object

### Phase 6 — MoveIt2 & Motion Planning
- [ ] MoveIt2 configuration package
- [ ] Collision-aware path planning
- [ ] Demo: pick-and-place with vision-guided grasping

---

## 🛠️ Software Stack

| Layer | Tool | Purpose |
|-------|------|---------|
| OS | Ubuntu 22.04 (WSL2) | ROS2 native environment |
| Containerization | Docker + NVIDIA Container Toolkit | Reproducible CUDA + MoveIt2 |
| Robot Framework | ROS2 Humble | Node communication, TF, visualization |
| Visualization | RViz2 (via WSLg) | Real-time 3D arm model |
| Motion Planning | MoveIt2 | Path planning, IK, collision |
| Vision / GPU | CUDA 12.8 | Accelerated CV pipeline |
| Firmware IDE | PlatformIO + VS Code | STM32 firmware development |
| Version Control | Git / GitHub | [Ninni_Robor_Arm](https://github.com/N1NRAPAT/Ninni_Robor_Arm) |

---

## 🚀 Getting Started

```bash
# Clone the repo
git clone https://github.com/N1NRAPAT/Ninni_Robor_Arm.git
cd Ninni_Robor_Arm

# Build Docker image (from project root)
docker build -f docker/Dockerfile . -t ninni_arm

# Run with GPU support
docker run --gpus all --rm -it ninni_arm

# Launch RViz2 (WSL2 native)
source /opt/ros/humble/setup.bash
rviz2
```

---

## 📄 License

Open source — contributions and forks welcome. Built for learning; shared for the community.