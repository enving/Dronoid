# Drone Simulation Setup Guide - PX4 SITL + ROS2

## Aktueller Status

### ✅ Abgeschlossen:
1. System Check (ROS2 Jazzy, Python 3.12, Gazebo)
2. PX4 Autopilot v1.15 klonen (läuft gerade...)

### 🔄 In Arbeit:
- PX4 Repository wird geklont mit allen Submodulen

### ⏳ Nächste Schritte:
1. PX4 Dependencies installieren
2. PX4 SITL kompilieren
3. Gazebo PX4 Plugin setup
4. Micro-XRCE-DDS Agent (ROS2 Bridge)
5. Erste Drohne starten!

---

## Installation Steps (Automated)

### 1. PX4 Dependencies installieren

Nach dem Clone wird PX4's eigener Setup-Script ausgeführt:

```bash
cd ~/Repos/PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
```

**Was wird installiert:**
- Build tools (CMake, Ninja, etc.)
- Python packages (MAVLink, etc.)
- Gazebo dependencies
- Fast-RTPS/DDS für ROS2

**Dauer:** ~5-10 Minuten (je nach Internet)

---

### 2. PX4 SITL kompilieren

```bash
cd ~/Repos/PX4-Autopilot
make px4_sitl gz_x500
```

**Was passiert:**
- PX4 Firmware für Simulation bauen
- Gazebo X500 Quadcopter Model laden
- Beim ersten Build: ~10-15 Minuten
- Danach: <1 Minute

**Output (wenn erfolgreich):**
```
[100%] Built target px4
...
INFO  [gz_bridge] Creating Gazebo::Transport bridge
```

---

### 3. Erste Drohne starten (Gazebo)

```bash
cd ~/Repos/PX4-Autopilot
make px4_sitl gz_x500
```

**Automatisch gestartet:**
- Gazebo Simulator (3D Window öffnet sich)
- PX4 Autopilot (SITL)
- MAVLink connection
- Gazebo Bridge

**Du siehst:**
- Gazebo Fenster mit X500 Quadcopter
- PX4 Console mit Telemetrie
- "pxh>" Prompt für Kommandos

**Erste Kommandos testen:**
```bash
pxh> commander takeoff    # Drohne startet!
pxh> commander land        # Drohne landet
```

---

### 4. Micro-XRCE-DDS Agent (ROS2 Bridge)

**Was ist das?**
- Verbindet PX4 (uORB) mit ROS2 (DDS)
- Ermöglicht ROS2 Nodes mit Drohne zu kommunizieren

**Installation:**
```bash
cd ~/Repos
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig /usr/local/lib/
```

**Starten:**
```bash
MicroXRCEAgent udp4 -p 8888
```

**Output:**
```
UDP4 Agent initialization... OK
Running... OK
```

---

### 5. ROS2 Integration testen

**Terminal 1: PX4 SITL**
```bash
cd ~/Repos/PX4-Autopilot
make px4_sitl gz_x500
```

**Terminal 2: Micro-XRCE-DDS Agent**
```bash
MicroXRCEAgent udp4 -p 8888
```

**Terminal 3: ROS2 Topics checken**
```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list | grep fmu
```

**Du siehst Topics wie:**
```
/fmu/in/offboard_control_mode
/fmu/in/vehicle_command
/fmu/out/vehicle_status
/fmu/out/vehicle_odometry
/fmu/out/sensor_combined
```

**Testen: Drohne Position auslesen**
```bash
ros2 topic echo /fmu/out/vehicle_odometry
```

**Testen: Drohne via ROS2 steuern**
```bash
ros2 topic pub /fmu/in/vehicle_command px4_msgs/msg/VehicleCommand \
  "command: 176
   param1: 1.0
   param2: 0.0"
```

(Command 176 = ARM, param1=1 = enable)

---

## Architektur (Wie es zusammenarbeitet)

```
┌──────────────────────────────────────────┐
│  GAZEBO SIMULATOR                        │
│  - 3D Visualisierung                     │
│  - Physik Engine                         │
│  - Sensoren (GPS, IMU, Camera)           │
└────────────┬─────────────────────────────┘
             │ Gazebo Transport
             ▼
┌──────────────────────────────────────────┐
│  PX4 AUTOPILOT (SITL)                    │
│  - Flight Controller                     │
│  - Sensorfusion                          │
│  - Position/Attitude Control             │
│  - MAVLink Protocol                      │
└────────────┬─────────────────────────────┘
             │ uORB messages
             ▼
┌──────────────────────────────────────────┐
│  MICRO-XRCE-DDS AGENT                    │
│  - uORB → ROS2 DDS Converter             │
│  - Topic Bridge                          │
└────────────┬─────────────────────────────┘
             │ ROS2 Topics
             ▼
┌──────────────────────────────────────────┐
│  ROS2 NODES                              │
│  - Unser HTTP Bridge (Natural Language)  │
│  - Computer Vision Nodes                 │
│  - Mission Planner                       │
└──────────────────────────────────────────┘
```

---

## Wichtige PX4 Topics (ROS2)

### Input (Steuerung):
- `/fmu/in/vehicle_command` - Kommandos (ARM, TAKEOFF, LAND)
- `/fmu/in/offboard_control_mode` - Offboard Mode Config
- `/fmu/in/trajectory_setpoint` - Position/Velocity setpoints

### Output (Telemetrie):
- `/fmu/out/vehicle_status` - Status (armed, flight mode)
- `/fmu/out/vehicle_odometry` - Position/Velocity
- `/fmu/out/vehicle_attitude` - Roll/Pitch/Yaw
- `/fmu/out/sensor_combined` - IMU, GPS, Baro
- `/fmu/out/battery_status` - Batterie-Info

### Camera (wenn aktiviert):
- `/camera/image_raw` - Kamera-Stream
- `/camera/camera_info` - Kamera-Kalibrierung

---

## Troubleshooting

### Gazebo öffnet nicht / schwarzer Screen
```bash
# Prüfe Gazebo Installation
gz sim --version

# Falls Problem:
export GZ_SIM_RESOURCE_PATH=~/Repos/PX4-Autopilot/Tools/simulation/gz/models
export GZ_SIM_SYSTEM_PLUGIN_PATH=~/Repos/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic
```

### PX4 kompiliert nicht
```bash
# Dependencies erneut installieren
cd ~/Repos/PX4-Autopilot
bash ./Tools/setup/ubuntu.sh --reinstall

# Clean build
make distclean
make px4_sitl gz_x500
```

### ROS2 Topics erscheinen nicht
```bash
# Prüfe ob Micro-XRCE-DDS Agent läuft:
ps aux | grep MicroXRCE

# Falls nicht, starte:
MicroXRCEAgent udp4 -p 8888

# Prüfe PX4 SITL Output:
# Sollte zeigen: "INFO  [uxrce_dds_client] connected to agent"
```

### Drohne startet nicht (ARM failed)
```bash
# In PX4 Console (pxh>):
commander arm
commander takeoff

# Prüfe Pre-Arm checks:
commander status
```

---

## Nächster Schritt: Natural Language Control

Sobald PX4 + ROS2 funktioniert, erstellen wir:

**File:** `~/ros2_ws/src/dronecore/dronecore/nl_controller.py`

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint
from geometry_msgs.msg import PoseStamped
import requests

class DroneNLController(Node):
    """Natural Language Controller für PX4 Drohne (wie MeArm!)"""

    def __init__(self):
        super().__init__('drone_nl_controller')

        # Publishers für PX4
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10
        )
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10
        )
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10
        )

        # HTTP Server (wie bei MeArm)
        # FastAPI integration kommt hier hin

    def parse_nl_command(self, text):
        """Natural Language → Drone Command"""
        # "Flieg 2 Meter hoch" → Trajectory(z=2.0)
        # "Lande" → VehicleCommand(LAND)
        # Mit OpenRouter LLM (wie MeArm!)
        pass

    def execute_command(self, command):
        """Execute drone command"""
        if command['type'] == 'takeoff':
            self.send_takeoff_command()
        elif command['type'] == 'land':
            self.send_land_command()
        elif command['type'] == 'goto':
            self.send_goto_command(command['position'])

    def send_takeoff_command(self):
        """Takeoff command"""
        cmd = VehicleCommand()
        cmd.command = 176  # MAV_CMD_DO_SET_MODE
        cmd.param1 = 1.0  # ARM
        self.vehicle_command_pub.publish(cmd)

        # Then takeoff
        cmd.command = 177  # MAV_CMD_NAV_TAKEOFF
        cmd.param7 = 2.5  # Altitude
        self.vehicle_command_pub.publish(cmd)
```

**Dann:**
```bash
# Terminal 1: PX4
make px4_sitl gz_x500

# Terminal 2: Agent
MicroXRCEAgent udp4 -p 8888

# Terminal 3: Natural Language Controller
python3 ~/ros2_ws/src/dronecore/dronecore/nl_controller.py

# Terminal 4: Web Interface
curl -X POST http://localhost:8000/command -d '{"text": "Flieg 3 Meter hoch"}'
```

**→ Drohne führt aus!**

---

## Timeline

**Heute (Installation & Setup):**
- [✓] PX4 klonen
- [ ] Dependencies installieren (5-10 min)
- [ ] PX4 kompilieren (10-15 min)
- [ ] Erste Drohne fliegen (5 min)

**Morgen (ROS2 Integration):**
- [ ] Micro-XRCE-DDS Agent
- [ ] ROS2 Topics testen
- [ ] Erste ROS2 Node schreiben

**Übermorgen (Natural Language):**
- [ ] HTTP Bridge für Drohne (wie MeArm!)
- [ ] OpenRouter LLM Integration
- [ ] Web Interface

**= 3 Tage bis Natural Language Drone Control!**

---

Bereit für nächsten Schritt sobald Clone fertig ist!
