# Humanoid Roboter Projekt - Architektur v2.0

## Vision
Humanoider Roboter mit Smartphone als "Augen und Ohren" - komplett simuliert, später Hardware-ready.

## Systemübersicht

```
┌─────────────────────────────────────────────────────────────────┐
│                        USER INTERFACE                            │
│  ┌─────────────┐  ┌──────────────┐  ┌─────────────────────┐    │
│  │ Smartphone  │  │ Claude Code  │  │  Terminal/Web UI    │    │
│  │ (Zukunft)   │  │   (via MCP)  │  │                     │    │
│  └──────┬──────┘  └──────┬───────┘  └──────────┬──────────┘    │
└─────────┼────────────────┼──────────────────────┼───────────────┘
          │                │                      │
          │ HTTP/WebSocket │ MCP Protocol         │ ROS2 CLI
          ▼                ▼                      ▼
┌─────────────────────────────────────────────────────────────────┐
│                    CONTROL LAYER                                 │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │  ROS2 MCP Server (Yutarop/ros-mcp)                       │   │
│  │  - Natural Language → ROS2 Commands                      │   │
│  │  - Topic/Service/Action Management                       │   │
│  │  - Gazebo GUI Launch                                     │   │
│  └────────────────────┬─────────────────────────────────────┘   │
│                       │                                          │
│  ┌────────────────────┴─────────────────────────────────────┐   │
│  │  BehaviorTree.CPP (High-Level Logic) - PHASE 3           │   │
│  │  - Task Planning (pick, place, walk, etc.)               │   │
│  │  - State Management                                      │   │
│  └────────────────────┬─────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
                        │
                        ▼ ROS2 Topics/Actions
┌─────────────────────────────────────────────────────────────────┐
│                  MIDDLEWARE LAYER (ROS2)                         │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────────┐  │
│  │ ros2_control │  │ MoveIt2      │  │ Navigation2          │  │
│  │ (Joints)     │  │ (Kinematics) │  │ (Locomotion - LATER) │  │
│  └──────┬───────┘  └──────┬───────┘  └──────────┬───────────┘  │
└─────────┼──────────────────┼──────────────────────┼──────────────┘
          │                  │                      │
          ▼                  ▼                      ▼
┌─────────────────────────────────────────────────────────────────┐
│              SIMULATION / HARDWARE INTERFACE                     │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │  Gazebo Harmonic (or Classic for low RAM)                │   │
│  │  - Physics Simulation                                    │   │
│  │  - Sensor Simulation (Camera, IMU, Lidar)                │   │
│  │  - ros_gz_bridge                                         │   │
│  └──────────────────────────────────────────────────────────┘   │
│                                                                  │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │  RViz2 (Visualization)                                    │   │
│  │  - 3D Model Display                                      │   │
│  │  - Sensor Data Visualization                             │   │
│  └──────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
```

## Entwicklungsphasen

### PHASE 1: Minimal Viable Setup (JETZT)
**Ziel**: 1-DOF Joint mit ROS2 Control testen

**Components**:
- Single revolute joint (test_joint)
- ROS2 Control (Hardware Interface Mock)
- Gazebo Classic (leichter für 4GB RAM)
- RViz2 für Visualisierung

**Deliverables**:
1. URDF mit ros2_control tags
2. Controller Config (joint_state_broadcaster + position_controller)
3. Launch file: Gazebo + ROS2 Control + RViz2
4. Test: `ros2 topic pub /test_joint_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [1.57]}"` bewegt Joint

**Duration**: 1-2 Stunden

---

### PHASE 2: MeArm mit ROS2 Control
**Ziel**: 4-DOF Arm vollständig steuerbar

**Components**:
- 4 Joints: waist, shoulder, elbow, gripper
- ROS2 Control (mock_components)
- Joint Trajectory Controller
- Simple Command Interface (CLI oder Python Script)

**Deliverables**:
1. MeArm URDF mit ros2_control
2. Multi-joint trajectory execution
3. Gazebo + RViz2 Launch
4. Python Script: `send_arm_pose(shoulder=30, elbow=45, ...)`

**Duration**: 2-3 Stunden

---

### PHASE 3: Natural Language + Smartphone Prep
**Ziel**: Sprachbefehle (via Terminal erstmal, später Smartphone)

**Components**:
- ROS2 MCP Server (ros-mcp) für Claude Desktop
- HTTP ↔ ROS2 Bridge (z.B. rosbridge_suite oder custom FastAPI)
- BehaviorTree.CPP (optional, für komplexe Tasks)

**Architecture für Smartphone**:
```
Smartphone App
  └─> HTTP POST /command {"text": "greife den Block"}
       └─> FastAPI Server (Python)
            └─> LLM (OpenRouter API)
                 └─> JSON: {"action": "pick", "target": "block", "position": [x,y,z]}
                      └─> ROS2 Action Client
                           └─> MoveIt2 (Inverse Kinematics)
                                └─> ros2_control → Gazebo
```

**Deliverables**:
1. ros-mcp Server konfiguriert für Claude Desktop
2. FastAPI Server mit /command Endpoint
3. Test: `curl -X POST http://localhost:8000/command -d '{"text":"move arm up"}'`
4. Smartphone Mock Interface (HTML/JavaScript)

**Duration**: 3-4 Stunden

---

### PHASE 4: Torso + Kopf (Humanoid Basis)
**Ziel**: Upper Body (Kopf mit Kamera-Mount für Smartphone)

**Components**:
- Torso (1-2 DOF für Rotation/Neigung)
- Kopf/Hals (2 DOF: pan, tilt)
- Smartphone-Mount am Kopf (fixed oder gimbal)
- Kamera-Simulation in Gazebo → ROS2 Image Topics

**Deliverables**:
1. Upper-body URDF (Torso + Kopf + 2x Arms)
2. Camera Plugin in Gazebo (simulated phone camera)
3. Image Processing Node (object detection - später)

**Duration**: 4-5 Stunden

---

### PHASE 5: Beine + Locomotion
**Ziel**: Vollständiger Humanoid mit Gehfähigkeit

**Components**:
- 2x Beine (6 DOF pro Bein: hip, knee, ankle)
- Navigation2 für Path Planning
- Balance Controller (ZMP oder ähnlich)
- Footstep Planning

**Deliverables**:
1. Full humanoid URDF (15+ DOF)
2. Walking Gait Controller
3. Navigation in Gazebo World

**Duration**: 10+ Stunden (komplex!)

---

## Tech Stack Entscheidungen

### ROS2 Distribution
**Wahl**: ROS2 Jazzy (bereits installiert)
- Neueste Features
- Gute Gazebo Harmonic Integration
- Aber: Gazebo Classic Option für RAM-Limit

### Simulation
**Option 1 (Aktuell)**: Gazebo Harmonic
- ✅ Beste Integration mit ROS2 Jazzy
- ❌ Hoher RAM-Verbrauch (problematisch bei 4GB)

**Option 2 (Fallback)**: Gazebo Classic 11
- ✅ Leichter, läuft auf 4GB RAM
- ✅ Gut dokumentiert
- ❌ Weniger moderne Features

**Option 3 (Ultraleicht)**: Nur RViz2 (kein Physics)
- ✅ Minimaler RAM
- ✅ Schnelles Debugging
- ❌ Keine Physik-Simulation
- **Empfehlung**: Für Development nutzen, Gazebo nur für Tests

### Control Framework
**Wahl**: ros2_control
- Industry Standard
- Hardware-ready (später echte Servos anschließen)
- Plugin-basiert (einfach Mock → Real Hardware wechseln)

### Kinematik (später)
**Wahl**: MoveIt2
- Standard für Manipulation
- Inverse Kinematics
- Collision Avoidance
- **Phase 3+** (erstmal manuelles Joint-Control)

### High-Level Logic
**Wahl**: BehaviorTree.CPP
- Visuelles Behavior Design
- Wiederverwendbare Subtrees
- Gut für komplexe Tasks
- **Phase 3+**

### Smartphone Integration
**Wahl**: FastAPI + WebSocket/HTTP
```python
# server.py
from fastapi import FastAPI
import rclpy
from std_msgs.msg import String

app = FastAPI()
ros_node = None  # init in startup

@app.post("/command")
async def handle_command(cmd: dict):
    # cmd = {"text": "pick up red block"}
    # → LLM (OpenRouter) → {"action": "pick", "target": "red_block"}
    # → ROS2 Action
    return {"status": "processing"}

@app.websocket("/camera")
async def stream_camera(websocket):
    # Stream ROS2 Image Topic → Smartphone Display
    pass
```

### LLM Integration (für NLP)
**Wahl**: OpenRouter API (wegen 4GB RAM)
- Kein lokales Mistral-7B (braucht >16GB RAM)
- Cloud-basiert, günstig
- Später: Feintuning für Roboter-Kommandos

---

## RAM-Optimierung (4GB System)

### Problem
- Gazebo Harmonic: ~2GB RAM
- ROS2 Nodes: ~500MB
- OS + Browser: ~1GB
- **Total: >3.5GB** → System wird swappen

### Lösungen

1. **Gazebo Classic statt Harmonic**
   ```bash
   sudo apt install gazebo11 ros-jazzy-gazebo-ros-pkgs
   ```
   RAM-Ersparnis: ~800MB

2. **Headless Gazebo** (kein GUI)
   ```bash
   gz sim -s world.sdf  # Server-only mode
   # RViz2 für Visualisierung nutzen
   ```
   RAM-Ersparnis: ~1.2GB

3. **Swap Space erhöhen**
   ```bash
   sudo fallocate -l 8G /swapfile
   sudo chmod 600 /swapfile
   sudo mkswap /swapfile
   sudo swapon /swapfile
   ```

4. **Minimale World Files**
   - Keine komplexen Meshes
   - Einfache Primitives (Box, Cylinder)
   - Weniger Plugins

---

## Nächste Schritte (SOFORT)

1. **ros-mcp Server einrichten** (damit Claude Code besser mit ROS/Gazebo interagieren kann)
2. **Phase 1 starten**: Minimal 1-DOF Test
3. **Entscheidung**: Gazebo Classic vs. Harmonic (RAM-Test)
4. **Phase 2**: MeArm mit ros2_control

---

## Langzeit-Roadmap (nach Phase 5)

- **Hardware Integration**: Echte Servos (z.B. Dynamixel)
- **Smartphone App**: React Native (iOS/Android)
- **Vision**: YOLOv8 für Object Detection
- **Voice**: Whisper für Speech-to-Text (auf Smartphone)
- **Cloud Sim**: AWS RoboMaker oder NVIDIA Isaac Sim (für Heavy Workloads)
- **Reinforcement Learning**: Policies trainieren (Walking, Manipulation)

---

## Weitere Überlegungen

### Warum nicht direkt Humanoid?
**Antwort**: Inkrementelles Development verhindert Komplexitäts-Explosion
- Arm: 4 DOF → testbar in 1 Tag
- Humanoid: 20+ DOF → Wochen Debug-Zeit
- Jede Phase baut auf vorheriger auf → solides Fundament

### Warum ros2_control statt eigene Lösung?
**Antwort**: Industry Standard, Hardware-ready
- ros2_control → egal ob Gazebo oder echte Motoren
- Plugin-System → Mock → Real ohne Code-Change
- Community Support + Dokumentation

### Smartphone als "Augen/Ohren" - warum?
**Antwort**: Kostengünstig, powerful, bereits vorhanden
- Smartphone = 12MP Camera + Mikrofon + IMU + GPS
- Kein Raspberry Pi + separate Kamera nötig
- ROS2 ↔ Smartphone über WiFi/HTTP sehr einfach
