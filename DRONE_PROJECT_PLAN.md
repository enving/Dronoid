# Universal Drone OS - Project Plan

## Vision

**Ein universelles, Open-Source Betriebssystem für Drohnen mit:**
- Hardware Auto-Discovery (Motoren, Sensoren, Kameras)
- Vision Language Model (VLM) für intelligente Steuerung
- TinyML für Edge Computing (Raspberry Pi)
- Natural Language Interface
- ROS2-basierte Architektur
- Fassadenreinigung als erste Anwendung

**Motto:** "Plug & Play für Drohnen - wie ROS2 für Roboter"

---

## Komponenten-Übersicht

### 1. DroneCore OS (Basis)
```
Hardware Abstraction Layer:
- Auto-Discovery: ESCs, Motoren, Servos
- Sensor Manager: IMU, GPS, Barometer, Camera
- Communication Layer: MAVLink ↔ ROS2 Bridge
- Safety Manager: Battery, Failsafe, Geofencing
```

### 2. Vision Layer
```
VLM (Vision-Language-Action Model):
- OpenVLA oder Wall-OSS
- Schmutzerkennung auf Fassaden
- Natural Language Commands
- Object Tracking
```

### 3. Edge AI (TinyML)
```
Raspberry Pi 4/5 als Companion Computer:
- Lokale Bildverarbeitung
- Keine Cloud nötig (Privacy!)
- Real-time Inference
- Low Latency
```

### 4. Flight Control
```
PX4 Autopilot:
- Position Control
- Stabilization gegen Wasserdruck
- Waypoint Navigation
- Autonomous Missions
```

---

## Phase 1: Simulation (2-3 Wochen)

### Ziel: Funktionierendes System in Gazebo

**Schritt 1.1: PX4 SITL + Gazebo Setup**
- PX4 Installation
- Gazebo Garden/Harmonic
- ROS2 Jazzy Integration
- Micro-XRCE-DDS Agent

**Schritt 1.2: Basis-Drohne fliegen**
- Takeoff / Landing
- Position Control via ROS2
- Waypoint Navigation
- Gimbal Control (Kamera)

**Schritt 1.3: Kamera + Computer Vision**
- Gazebo Camera Plugin
- OpenCV Integration
- Einfache Objekterkennung (z.B. Farberkennung)
- "Schmutz"-Detektion simulieren

**Schritt 1.4: Natural Language Control**
- Unser HTTP Bridge wiederverwenden!
- "Flieg zu Position X,Y,Z"
- "Reinige die Wand vor dir"
- OpenRouter LLM Integration

**Deliverables Phase 1:**
```
✅ Drohne fliegt autonom in Gazebo
✅ Kamera erkennt simulierte Schmutzflecken
✅ Natural Language Steuerung funktioniert
✅ ROS2 System läuft headless (wie MeArm!)
```

---

## Phase 2: Hardware Auto-Discovery (2 Wochen)

### Ziel: System erkennt automatisch Hardware

**Hardware-Konfiguration über YAML:**
```yaml
# drone_config.yaml
hardware:
  motors:
    - id: 1
      type: brushless_2207
      kv: 1750
      position: front_left
    - id: 2
      type: brushless_2207
      kv: 1750
      position: front_right
    # ... weitere Motoren

  sensors:
    - type: imu
      model: mpu6050
      i2c_address: 0x68
    - type: gps
      model: ublox_m8n
      serial: /dev/ttyS1
    - type: camera
      model: raspberry_pi_cam_v3
      interface: csi

  payloads:
    - type: spray_system
      pump_pwm_pin: 18
      pressure_sensor: analog_1
      max_pressure: 30  # MPa
```

**Auto-Discovery Script:**
```python
# dronecore_discovery.py
import rclpy
from rclpy.node import Node
import yaml

class DroneDiscovery(Node):
    def __init__(self):
        super().__init__('drone_discovery')
        self.discover_hardware()
        self.setup_ros2_interfaces()
        self.publish_capabilities()

    def discover_hardware(self):
        """Scanne I2C, Serial, GPIO für Hardware"""
        # IMU detection
        # GPS detection
        # Motor ESC detection
        # Camera detection
        pass

    def setup_ros2_interfaces(self):
        """Erstelle ROS2 Topics/Services basierend auf Hardware"""
        # z.B. wenn Spray-System erkannt:
        # → /spray_system/pump_control
        # → /spray_system/pressure_status
        pass

    def publish_capabilities(self):
        """Publiziere, was die Drohne kann"""
        # → /drone/capabilities
        # Ermöglicht Mission Planner zu wissen: "Diese Drohne kann sprühen"
        pass
```

**Deliverables Phase 2:**
```
✅ YAML-basierte Hardware-Konfiguration
✅ Auto-Discovery Script erkennt angeschlossene Hardware
✅ ROS2 Topics werden dynamisch erstellt
✅ System weiß, welche "Skills" verfügbar sind
```

---

## Phase 3: VLM Integration (2-3 Wochen)

### Ziel: Vision-Language-Model steuert Drohne intelligent

**VLM-Optionen:**
1. **OpenVLA** (empfohlen)
   - Open Source
   - 970k Trainings-Trajektorien
   - Roboter-fokussiert

2. **Wall-OSS** (2025, neu)
   - Embodied AI fokussiert
   - Evtl. besser für Drohnen?
https://github.com/X-Square-Robot/wall-x


3. **TinyML auf Raspberry Pi**
   - Lokale Inferenz
   - Edge Impulse Framework
   - Schneller, keine Cloud

**Integration:**
```python
# vlm_controller.py
from openvla import OpenVLA  # oder TinyML Model

class VLMDroneController(Node):
    def __init__(self):
        super().__init__('vlm_controller')
        self.model = OpenVLA.load("openvla-7b")  # 7B Parameter Model
        self.camera_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )

    def image_callback(self, msg):
        """Process camera image with VLM"""
        image = self.ros_to_cv2(msg)

        # VLM Processing
        prompt = "Describe what you see and suggest cleaning actions"
        action = self.model.predict(image, prompt)

        # Execute action
        self.execute_cleaning_action(action)

    def execute_cleaning_action(self, action):
        """Convert VLM output to drone commands"""
        # z.B. action = "Move 2m right, spray dirty area"
        # → ROS2 commands: position_control + spray_system
        pass
```

**Natural Language + VLM:**
```
User: "Reinige die Fassade"
  ↓
LLM (OpenRouter): Verstehe Absicht
  ↓
VLM (OpenVLA): Erkenne Schmutz in Kamera-Bild
  ↓
DroneCore: Flugpfad planen + Reinigen
  ↓
PX4: Ausführen
```

**Deliverables Phase 3:**
```
✅ VLM erkennt Schmutz in Simulation
✅ VLM schlägt Reinigungsaktionen vor
✅ Natural Language + Vision kombiniert
✅ Autonom: "Reinige die Wand" → Drohne macht alles
```

---

## Phase 4: Real Hardware (Raspberry Pi Test)

### Ziel: TinyML auf Raspberry Pi testen (KEIN Flug nötig!)

**Hardware:**
- Raspberry Pi 4 (hast du schon!)
- Pi Camera Module v3 (~30€)
- Micro SD Karte (32GB+)

**Setup:**
```bash
# Edge Impulse installieren
curl -sL https://deb.nodesource.com/setup_14.x | sudo bash -
sudo apt install -y nodejs
sudo npm install -g edge-impulse-cli

# TinyML Model trainieren
edge-impulse-studio  # → Browser öffnet

# Model auf Pi deployen
edge-impulse-linux-runner
```

**Test ohne Drohne:**
- Raspberry Pi + Kamera auf Stativ
- Simuliere "Drohnen-Sicht" auf Wand
- TinyML erkennt Schmutzflecken
- ROS2 Node auf Pi läuft

**Deliverables Phase 4:**
```
✅ TinyML läuft auf Raspberry Pi
✅ Camera erkennt Schmutz (Wand-Test)
✅ ROS2 Node kommuniziert über WiFi
✅ Latenz gemessen (sollte <100ms sein)
```

---

## Phase 5: Hardware Integration (später)

### Erst kaufen, wenn Software fertig ist!

**Minimales Setup:**
```
DJI-kompatible Drohne (z.B. gebrauchte Phantom):  ~300-500€
Raspberry Pi 4 (hast du):                          0€
Pi Camera v3:                                      ~30€
Gimbal (für Stabilität):                           ~50€
Spray-System (DIY, low-pressure):                  ~100€
────────────────────────────────────────────────────────
TOTAL:                                             ~480-680€
```

**Später (Professionell):**
```
DJI M350 RTK:                                      ~10.000€
AeroClean T50 Payload:                             ~5.000€
Hochdruck-System:                                  ~2.000€
────────────────────────────────────────────────────────
TOTAL:                                             ~17.000€
```

---

## Open Source & PRP

### GitHub Repository Structure:
```
dronecore-os/
├── README.md                  # Vision & Quick Start
├── docs/
│   ├── ARCHITECTURE.md        # System Design
│   ├── HARDWARE_GUIDE.md      # Supported Hardware
│   ├── SIMULATION.md          # Gazebo Setup
│   └── VLM_INTEGRATION.md     # Vision-Language Models
├── dronecore/                 # Core Python Package
│   ├── discovery/             # Hardware Auto-Discovery
│   ├── control/               # Flight Controllers
│   ├── vision/                # VLM Integration
│   └── natural_language/      # LLM Interface
├── simulation/                # Gazebo Worlds & Models
├── config/                    # YAML Configs
├── examples/                  # Tutorials
│   ├── 01_basic_flight/
│   ├── 02_object_detection/
│   ├── 03_facade_cleaning/
│   └── 04_custom_payload/
└── hardware/                  # Hardware Integration Guides
    ├── raspberry_pi/
    ├── px4_setup/
    └── diy_spray_system/
```

### Lizenz: MIT oder Apache 2.0
→ Jeder kann es nutzen, auch kommerziell!

---

## Warum das funktioniert

**1. Wir haben schon 80% der Skills:**
- ROS2 Control System ✅ (MeArm-Projekt)
- Natural Language Interface ✅ (HTTP Bridge)
- Headless Operation ✅ (230MB RAM)
- Hardware Abstraction ✅ (ros2_control Pattern)

**2. PX4 ist Industriestandard:**
- Wie ROS2 für Roboter
- Riesige Community
- Perfekte Dokumentation

**3. Simulation first = Sicher & Günstig:**
- Keine Absturzgefahr
- Unbegrenzt experimentieren
- Später: Hardware = nur noch Testing

**4. Open Source = Impact:**
- Andere können mithelfen
- Feedback aus Community
- Portfolio-Projekt!

---

## Nächste Schritte (wenn du willst)

### Option 1: Sofort starten mit Simulation
Ich helfe dir:
1. PX4 SITL installieren
2. Gazebo Setup
3. Erste Drohne fliegen lassen (in 30 Minuten!)

### Option 2: Hardware-Discovery Design
Erstmal die Architektur durchdenken:
- Wie soll Auto-Discovery funktionieren?
- Welche Hardware-Typen unterstützen?
- YAML-Format designen

### Option 3: VLM Research
Tiefer in VLM eintauchen:
- OpenVLA lokal testen
- TinyML auf deinem Raspberry Pi
- Object Detection Benchmark

---

## Timeline (Realistisch)

```
Woche 1-2:  PX4 SITL + Gazebo + ROS2 Setup
Woche 3-4:  Basis-Flugsteuerung + Kamera
Woche 5-6:  Natural Language Control
Woche 7-8:  Hardware Auto-Discovery Design
Woche 9-11: VLM Integration (Simulation)
Woche 12:   Raspberry Pi + TinyML Test

= 3 Monate bis funktionierender Prototyp in Simulation
```

Danach: Hardware-Kauf & Real-World Testing

---

## Frage an dich:

**Soll ich starten mit Phase 1 (PX4 SITL Installation)?**

Oder möchtest du erstmal:
- Die Architektur weiter ausarbeiten?
- Hardware-Discovery-System designen?
- VLM-Optionen vergleichen?

**Dein Projekt, deine Entscheidung!**
