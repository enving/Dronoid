# 🚁 Dronoid - Drones that Understand Natural Language

**Vision**: Open-source drone operating system with natural language control and computer vision for autonomous facade cleaning.

## 🎯 Project Goals

1. **Universal Drone OS** - Hardware-agnostic system that auto-detects sensors/actuators
2. **Natural Language Control** - Fly drones with plain text commands ("takeoff to 10m", "land")
3. **Facade Cleaning** - Autonomous dirt detection and cleaning with mounted water system
4. **Simulation First** - Test everything in PX4 SITL + Gazebo before hardware deployment
5. **Budget-Friendly** - Start with cheap components ($50-200), scalable to industrial ($$$)

## ✨ Features

- **🗣️ Natural Language Interface**: Control drone with text commands (English/German)
  - "takeoff" / "abheben"
  - "fly up to 10m" / "fliege hoch auf 10m"
  - "land" / "landen"
- **🤖 PX4 SITL Integration**: Full simulation environment with Gazebo
- **🔌 ROS2 Bridge**: DDS communication for advanced features
- **🎯 Vision-Ready**: Prepared for VLM integration (Wall-X, TinyML)
- **📦 Plug & Play**: YAML-based hardware configuration

## 🚀 Quick Start (5 Minutes)

### 1. Install Dependencies

```bash
# Install ROS2 Jazzy (if not already installed)
sudo apt update
sudo apt install -y ros-jazzy-desktop

# Install PX4 dependencies
sudo apt install -y \
    python3-pip \
    git \
    wget \
    gnutls-bin \
    openjdk-11-jre-headless \
    gz-harmonic

# Install Python packages
pip3 install --break-system-packages \
    pymavlink \
    python-dotenv \
    requests \
    kconfiglib \
    pyros-genmsg \
    packaging \
    toml \
    numpy \
    jinja2

# Or use requirements.txt:
pip3 install --break-system-packages -r requirements.txt
```

### 2. Clone and Setup

```bash
git clone https://github.com/YOUR_USERNAME/Dronoid.git
cd Dronoid

# Download dependencies (PX4, DDS Agent, etc.)
chmod +x setup_dependencies.sh
./setup_dependencies.sh
```

### 3. Start the Drone System

```bash
./start_drone.sh
```

This will:
- Start PX4 SITL (Software In The Loop)
- Launch Gazebo Harmonic simulator
- Spawn X500 quadcopter drone
- Wait for "Ready for takeoff!" message

### 4. Control the Drone

**Easy Mode** (auto-starts PX4 if needed):
```bash
./px4-client
```

**Manual Mode**:
```bash
# Make sure PX4 is running first
./start_drone.sh

# Then start control
python3 nl_drone.py
```

Then type natural language commands:
- `takeoff` - Take off to 5m altitude
- `takeoff to 10m` - Take off to 10m
- `fly up` / `abheben` - Take off (German)
- `land` / `landen` - Land the drone
- `q` - Quit

## 🏗️ Architecture

```
┌─────────────────────────────────────┐
│  Natural Language Interface          │
│  (nl_drone.py)                      │
│  • Text parsing (Regex/Cloud LLM)   │
│  • Bilingual support (EN/DE)        │
└──────────────┬──────────────────────┘
               │ MAVLink UDP
               ▼
┌─────────────────────────────────────┐
│  PX4 Autopilot (SITL)               │
│  • Flight controller firmware       │
│  • MAVLink communication            │
│  • UDP Port 14580 (Offboard API)    │
└──────────────┬──────────────────────┘
               │ Gazebo Bridge
               ▼
┌─────────────────────────────────────┐
│  Gazebo Harmonic Simulator          │
│  • Physics simulation               │
│  • X500 quadcopter model            │
│  • 3D visualization                 │
└─────────────────────────────────────┘

┌─────────────────────────────────────┐
│  ROS2 Integration (Future)          │
│  • Micro-XRCE-DDS-Agent             │
│  • px4_msgs                         │
│  • dronecore package                │
└─────────────────────────────────────┘
```

## 📁 Project Structure

```
Humanoid/
├── README.md                       # This file
├── start_drone.sh                  # Quick start script
├── nl_drone.py                     # Natural language control
├── control_drone.py                # Numeric menu control
├── quick_test.sh                   # PX4 + Gazebo launcher
│
├── PX4-Autopilot/                  # PX4 firmware (v1.15.4)
│   ├── Tools/simulation/gz/        # Gazebo models & worlds
│   └── build/px4_sitl_default/     # Build artifacts (git ignored)
│
├── Micro-XRCE-DDS-Agent/           # ROS2 ↔ PX4 bridge
│   └── build/                      # Build artifacts (git ignored)
│
├── px4_msgs/                       # ROS2 message definitions
├── dronecore/                      # ROS2 control package
│   ├── drone_commander.py          # ROS2 node for commands
│   └── nl_bridge.py                # FastAPI HTTP server
│
└── old_docs/                       # Legacy documentation
```

## 🎮 Control Methods

### 1. Natural Language (Recommended)
```bash
python3 nl_drone.py
```
Text commands with intelligent parsing.

### 2. Numeric Menu
```bash
python3 control_drone.py
```
Simple 1-4 menu for basic commands.

### 3. ROS2 Commands (Advanced)
```bash
# Build dronecore package first
cd dronecore && colcon build
source install/setup.bash

# Launch ROS2 nodes
ros2 run dronecore drone_commander
```

## 🧠 Natural Language Processing

Dronoid supports **two modes** for understanding your commands:

### 1. Regex Parser (Default - Lightweight)
- ⚡ **Fast**: Instant response
- 💻 **No internet needed**: Works offline
- 🪶 **Lightweight**: No ML model, runs on any hardware
- 📝 Pattern matching: `"takeoff" → takeoff()`

**Perfect for**: Weak laptops, offline use, predictable commands

### 2. Cloud LLM via OpenRouter (Optional - Smart)
- 🤖 **Intelligent**: Understands context and synonyms
- 🌍 **Multilingual**: Works in any language
- 💡 **Flexible**: "fly a bit higher" → understands intent
- 🆓 **Free tier available**: Google Gemini Flash 1.5

**Setup** (optional):
```bash
# 1. Get free API key: https://openrouter.ai/keys
# 2. Copy example config
cp .env.example .env

# 3. Edit .env and add your key
nano .env  # or use any text editor
```

The system **automatically falls back to regex** if OpenRouter is unavailable!

## 🔧 Configuration

### Environment Variables
Create `.env` file (not tracked in git):
```bash
# Optional: Cloud LLM for advanced NL processing
OPENROUTER_API_KEY=your-key-here
```

### Hardware Configuration (Future)
YAML-based plug & play:
```yaml
# drone_config.yaml
hardware:
  flight_controller: px4
  camera:
    type: raspberry_pi_camera_v2
    resolution: 1920x1080
  water_system:
    pump: 12v_dc
    pressure_sensor: analog
```

## 📊 System Requirements

### Minimum (Simulation)
- **CPU**: Dual-core 2.0 GHz
- **RAM**: 4GB (note: Gazebo can be heavy)
- **Storage**: 10GB free
- **OS**: Ubuntu 24.04 (or 22.04)

### Recommended (Hardware + Vision)
- **CPU**: Quad-core 2.5 GHz+
- **RAM**: 8GB+
- **GPU**: For VLM inference (optional)
- **Additional**: Raspberry Pi 4 (4GB) for onboard processing

## 🎯 Roadmap

### ✅ Phase 1: COMPLETED
- PX4 SITL + Gazebo setup
- MAVLink communication
- Natural language control (regex-based)
- Basic commands (arm, takeoff, land)

### 🚧 Phase 2: IN PROGRESS
- Cloud LLM integration (OpenRouter)
- Advanced command parsing
- HTTP API for remote control
- ROS2 full integration

### 📋 Phase 3: PLANNED
- VLM integration (Wall-X / TinyML)
- Dirt detection on facades
- Water system control
- Autonomous cleaning routines

### 📋 Phase 4: FUTURE
- Hardware deployment (Raspberry Pi + PX4)
- Real-world testing
- Multi-drone coordination
- Industrial scalability

## 🐛 Troubleshooting

### Drone won't connect
```bash
# Check if PX4 is running
ps aux | grep px4

# Restart system
./start_drone.sh
```

### Gazebo GUI not showing drone
```bash
# Check environment variables
echo $GZ_SIM_RESOURCE_PATH

# Should include PX4 models path
export GZ_SIM_RESOURCE_PATH=$PWD/PX4-Autopilot/Tools/simulation/gz/models:$PWD/PX4-Autopilot/Tools/simulation/gz/worlds
```

### Python connection timeout
```bash
# PX4 MAVLink should be on port 14580
lsof -i UDP:14580

# If port is closed, PX4 isn't running
```

## 🤝 Contributing

This is an open-source project! Contributions welcome:
- 🐛 Bug reports via GitHub Issues
- 💡 Feature suggestions
- 🔧 Pull requests
- 📖 Documentation improvements

## 📚 Related Projects

- [PX4 Autopilot](https://px4.io/) - Drone flight controller
- [MAVSDK](https://mavsdk.mavlink.io/) - Drone API library
- [Wall-X](https://github.com/remyxai/WallX) - VLM for visual inspection
- [Dronekit-Python](https://dronekit-python.readthedocs.io/) - Similar project

## 📝 License

MIT License - See LICENSE file

## 👤 Author

**Tristan** - Universal Drone OS Project

Built with [Claude Code](https://claude.com/claude-code) 🤖

---

**Note**: This project is in active development. Star ⭐ the repo to follow progress!
