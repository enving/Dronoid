# Universal Drone OS - Natural Language Control for PX4 Drones

Ein universelles, ROS2-basiertes Drohnen-Steuerungssystem mit Natural Language Interface für PX4 Drohnen. Entwickelt für Fassadenreinigung und andere autonome Aufgaben.

## 🎯 Features

- **Natural Language Control**: Steuere Drohne mit Textkommandos
  - "takeoff to 5 meters"
  - "land"
  - "go to 10 20 5"
- **PX4 Integration**: Volle Integration mit PX4 Autopilot via DDS
- **Gazebo Simulation**: Teste alles in Simulation vor echtem Flug
- **ROS2 Native**: Nutzt ROS2 Jazzy für Kommunikation
- **REST API**: HTTP Interface für Integration

## 📋 Prerequisites

- Ubuntu 24.04 LTS
- ROS2 Jazzy
- Python 3.12+
- Gazebo Harmonic (kommt mit ROS2 Jazzy)

## 🚀 Quick Start

### 1. System starten

```bash
cd ~/Repos/Humanoid
./start_drone_system.sh
```

Das Script startet:
- PX4 SITL (Software-in-the-Loop)
- Gazebo Simulator mit X500 Quadcopter
- Micro-XRCE-DDS Agent (ROS2 Bridge)
- Natural Language Bridge (HTTP API)

### 2. Drohne testen

In einem neuen Terminal:

```bash
./test_drone_commands.sh
```

Oder manuell:

```bash
# Drohne armen
curl -X POST http://localhost:8080/command \
  -H 'Content-Type: application/json' \
  -d '{"text": "arm"}'

# Takeoff
curl -X POST http://localhost:8080/command \
  -H 'Content-Type: application/json' \
  -d '{"text": "takeoff to 5 meters"}'

# Status checken
curl http://localhost:8080/status

# Landen
curl -X POST http://localhost:8080/command \
  -H 'Content-Type: application/json' \
  -d '{"text": "land"}'
```

## 📦 Architektur

```
┌──────────────────┐
│  Natural Language│
│  HTTP Interface  │  (Port 8080)
│  (FastAPI)       │
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│  DroneCommander  │  (ROS2 Node)
│  ROS2 Publisher  │
└────────┬─────────┘
         │
         ▼ /fmu/in/vehicle_command
┌──────────────────┐
│ Micro-XRCE-DDS   │  (UDP Port 8888)
│     Agent        │
└────────┬─────────┘
         │
         ▼ uORB
┌──────────────────┐
│   PX4 Autopilot  │
│   (SITL Mode)    │
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│  Gazebo Sim      │
│  X500 Quadcopter │
└──────────────────┘
```

## 🗂️ Projekt-Struktur

```
~/Repos/
├── PX4-Autopilot/           # PX4 Firmware (v1.15.4)
├── Micro-XRCE-DDS-Agent/    # DDS Bridge
├── px4_msgs/                # PX4 ROS2 Messages
└── Humanoid/
    ├── dronecore/           # ROS2 Package
    │   ├── drone_commander.py  # Drohnensteuerung
    │   └── nl_bridge.py        # Natural Language Interface
    ├── start_drone_system.sh   # System-Start Script
    ├── test_drone_commands.sh  # Test Script
    └── DRONE_*.md              # Dokumentation
```

## 🔧 Komponenten

### 1. DroneCommander (drone_commander.py)

ROS2 Node der PX4 VehicleCommands sendet:

```python
from dronecore.drone_commander import DroneCommander

drone = DroneCommander()
drone.arm()
drone.takeoff(altitude=5.0)
drone.land()
```

**Publizierte Topics:**
- `/fmu/in/vehicle_command` - Kommandos an PX4
- `/fmu/in/trajectory_setpoint` - Position Control
- `/fmu/in/offboard_control_mode` - Offboard Mode

**Subskribierte Topics:**
- `/fmu/out/vehicle_status` - Drohnen-Status
- `/fmu/out/vehicle_odometry` - Position/Velocity

### 2. Natural Language Bridge (nl_bridge.py)

FastAPI Server für Text-Kommandos:

```bash
# Einfaches Pattern Matching
curl -X POST http://localhost:8080/command \
  -d '{"text": "takeoff", "use_llm": false}'

# Optional: LLM-basiertes Parsing (braucht OpenRouter API Key)
curl -X POST http://localhost:8080/command \
  -d '{"text": "fliege 3 meter hoch", "use_llm": true}'
```

**Unterstützte Kommandos:**
- `takeoff` / `take off` / `start` → Takeoff
- `land` / `landen` / `come down` → Land
- `arm` → Motoren aktivieren
- `disarm` → Motoren deaktivieren
- `go to X Y Z` → Zu Position fliegen (NED frame)
- `status` / `where` → Status abrufen

### 3. PX4 SITL + Gazebo

Software-in-the-Loop Simulation:
- X500 Quadcopter Model
- Physics-basierte Simulation
- Sensor Simulation (IMU, GPS, Baro, etc.)

### 4. Micro-XRCE-DDS Agent

Bridge zwischen PX4 (uORB) und ROS2 (DDS):
- UDP Port 8888
- Verbindet sich automatisch mit PX4
- Übersetzt uORB ↔ DDS Topics

## 🛠️ Manuelle Installation (falls nötig)

### PX4 Autopilot

```bash
cd ~/Repos
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
git checkout v1.15.4
git submodule update --init --recursive
make px4_sitl gz_x500
```

### Micro-XRCE-DDS Agent

```bash
cd ~/Repos
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make -j4
```

### ROS2 Packages

```bash
cd ~/Repos
git clone https://github.com/PX4/px4_msgs.git -b release/1.15

# Build
colcon build --packages-select px4_msgs dronecore
source install/setup.bash
```

### Python Dependencies

```bash
pip3 install --break-system-packages fastapi uvicorn pydantic httpx
```

## 📝 ROS2 Commands

```bash
# Alle PX4 Topics anzeigen
ros2 topic list | grep fmu

# Drohnen-Status live anzeigen
ros2 topic echo /fmu/out/vehicle_status

# Position anzeigen
ros2 topic echo /fmu/out/vehicle_odometry

# DroneCommander einzeln starten
ros2 run dronecore drone_commander

# Natural Language Bridge einzeln starten
ros2 run dronecore nl_bridge
```

## 🐛 Troubleshooting

### Problem: "px4_msgs not found"

```bash
cd ~/Repos
colcon build --packages-select px4_msgs
source install/setup.bash
```

### Problem: "DDS Agent connection timeout"

```bash
# Check if Agent is running
ps aux | grep MicroXRCEAgent

# Restart Agent
pkill MicroXRCEAgent
~/Repos/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent udp4 -p 8888
```

### Problem: "Gazebo model not found"

```bash
export GZ_SIM_RESOURCE_PATH=$HOME/Repos/PX4-Autopilot/Tools/simulation/gz/models:$HOME/Repos/PX4-Autopilot/Tools/simulation/gz/worlds
```

### Problem: "NL Bridge fails to start"

```bash
# Check if port 8080 is free
netstat -tulpn | grep 8080

# Check Python dependencies
pip3 list | grep -E "fastapi|uvicorn|pydantic"
```

## 🔮 Nächste Schritte

1. **VLM Integration**: Wall-X oder TinyML für Schmutz-Erkennung
2. **Hardware Discovery**: YAML-basierte Auto-Configuration
3. **Mission Planning**: Autonome Fassadenreinigung
4. **Hardware Testing**: Raspberry Pi + Real Hardware

## 📚 Ressourcen

- [PX4 Documentation](https://docs.px4.io/)
- [ROS2 Jazzy](https://docs.ros.org/en/jazzy/)
- [Gazebo Sim](https://gazebosim.org/)
- [Micro-XRCE-DDS](https://micro-xrce-dds.docs.eprosima.com/)

## 📄 Lizenz

MIT License - siehe LICENSE file

## 👥 Contributors

- Claude (Anthropic) - Architecture & Implementation
- Tristan - Project Vision & Testing

---

**Status**: ✅ Simulation läuft | 🚧 Hardware-Integration pending

**Last Updated**: 2025-12-03
