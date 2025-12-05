# Drone Project - Setup Status

**Datum:** 2025-11-28
**Zeit:** ~15:00

---

## ✅ Abgeschlossen

### 1. Projekt-Entscheidung
- **Von:** MeArm (Roboterarm)
- **Zu:** Fassadenreinigungs-Drohne
- **Architektur:** Universal Drone OS mit Hardware Auto-Discovery

### 2. Recherche
- Kommerzielle Lösungen analysiert (KTV, Spider-i, AeroClean)
- VLM-Optionen identifiziert (OpenVLA, Wall-X, TinyML)
- PX4 + ROS2 + Gazebo Stack ausgewählt

### 3. Dokumentation erstellt
- `/home/tristan/Repos/Humanoid/DRONE_PROJECT_PLAN.md` - Gesamtplan
- `/home/tristan/Repos/Humanoid/DRONE_SETUP_GUIDE.md` - Setup-Anleitung
- `/home/tristan/Repos/Humanoid/DRONE_SETUP_STATUS.md` - Dieser Status

### 4. PX4 Autopilot Installation gestartet
- Repository geklont: `~/Repos/PX4-Autopilot` (1.6 GB)
- Version: PX4 v1.15 (release branch)
- 35 Submodule erfolgreich geklont

---

## 🔄 In Arbeit

### PX4 SITL Compilation
**Prozess läuft:** `make px4_sitl gz_x500`
**Status:** Build @ 2% - Kompiliert erfolgreich!
**Log:** `/tmp/px4_build.log`
**Dauer:** 10-15 Minuten (erste Build)

**Was wird kompiliert:**
- uORB topics (PX4 messaging)
- XRCE-DDS bridge (ROS2 integration)
- MAVLink (drone protocol)
- Flight tasks & drivers
- Gazebo X500 quadcopter model

**Gelöste Probleme:**
1. Git tag fehlte → `git fetch --tags` + checkout v1.15.4
2. symforce missing → `pip3 install symforce`
3. gz-transport nicht gefunden → ROS2 env sourcen
4. Python dependencies → Manuell installiert mit --break-system-packages

---

## ⏳ Nächste Schritte (nach Setup)

### 1. PX4 SITL kompilieren (~10-15 min)
```bash
cd ~/Repos/PX4-Autopilot
make px4_sitl gz_x500
```

### 2. Erste Drohne starten
```bash
cd ~/Repos/PX4-Autopilot
make px4_sitl gz_x500
# → Gazebo öffnet mit X500 Quadcopter
# → PX4 Console startet (pxh>)
```

### 3. Basis-Kommandos testen
```bash
pxh> commander takeoff
pxh> commander land
```

### 4. Micro-XRCE-DDS Agent installieren (ROS2 Bridge)
```bash
cd ~/Repos
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

### 5. ROS2 Integration testen
```bash
# Terminal 1: PX4
make px4_sitl gz_x500

# Terminal 2: DDS Agent
MicroXRCEAgent udp4 -p 8888

# Terminal 3: ROS2 Topics
source /opt/ros/jazzy/setup.bash
ros2 topic list | grep fmu
ros2 topic echo /fmu/out/vehicle_odometry
```

### 6. Natural Language Control
Unser MeArm HTTP Bridge wiederverwenden:
```python
# ~/ros2_ws/src/dronecore/dronecore/nl_controller.py
# Ähnlich wie MeArm, aber mit PX4 Topics
```

---

## Architektur (Geplant)

```
┌─────────────────────────────────────────┐
│  UNIVERSAL DRONE OS (DroneCore)         │
│  - Hardware Auto-Discovery              │
│  - Natural Language (OpenRouter LLM)    │
│  - VLM Integration (OpenVLA/TinyML)     │
└──────────────┬──────────────────────────┘
               │
     ┌─────────┼─────────┐
     │         │         │
┌────▼─────┐ ┌▼────────┐ ┌▼─────────┐
│ Gazebo   │ │ PX4 SITL│ │ ROS2     │
│ Sim      │ │ Flight  │ │ Control  │
│          │ │ Control │ │          │
└──────────┘ └─────────┘ └──────────┘
     │              │           │
     └──────────────┴───────────┘
                    │
     ┌──────────────▼──────────────┐
     │  Micro-XRCE-DDS Agent       │
     │  (uORB ↔ ROS2 Bridge)       │
     └──────────────┬──────────────┘
                    │
     ┌──────────────▼──────────────┐
     │  Natural Language Interface │
     │  (FastAPI + OpenRouter)     │
     │  (Wie MeArm-System!)        │
     └─────────────────────────────┘
```

---

## Wiederaufnahme-Anleitung

### Wenn Setup-Script fertig ist:

**1. Check ob erfolgreich:**
```bash
cat /tmp/px4_setup.log | tail -20
# Sollte "Done" oder Ähnliches zeigen
```

**2. PX4 kompilieren:**
```bash
cd ~/Repos/PX4-Autopilot
make px4_sitl gz_x500
```

**3. Bei Fehlern:**
```bash
# Clean build
make distclean
make px4_sitl gz_x500

# Falls Python-Dependencies fehlen:
pip3 install --break-system-packages kconfiglib pyros-genmsg packaging jinja2 empy pyyaml jsonschema
```

---

## Lessons Learned

### 1. Python Package Management auf Ubuntu 24.04
- **Problem:** PEP 668 blockiert `pip install`
- **Lösung:** PX4 Setup-Script nutzt `--break-system-packages` automatisch
- **Alternative:** venv (braucht `python3.12-venv` package)

### 2. Warum KEIN venv für PX4?
- PX4 Build-System erwartet System-Python
- Kompliziert venv in Makefiles zu nutzen
- PX4 Setup-Script macht es korrekt

### 3. Recycling vom MeArm-Projekt
- ROS2 Architektur ✅
- HTTP Bridge Pattern ✅
- Natural Language Interface ✅
- OpenRouter LLM Integration ✅
- Headless Operation ✅

**→ 80% der MeArm-Arbeit ist wiederverwendbar!**

---

## Timeline

**Heute (2025-11-28):**
- 14:30: Projekt-Entscheidung (Drohne statt MeArm)
- 14:35: PX4 Clone gestartet
- 14:40: PX4 geklont (1.6 GB)
- 14:45: Setup-Script gestartet
- 15:00: [AKTUELL] Setup läuft

**Geschätzt:**
- 15:10: Setup fertig
- 15:25: PX4 kompiliert
- 15:30: Erste Drohne fliegt in Gazebo! 🚁

**Morgen:**
- Micro-XRCE-DDS Agent
- ROS2 Integration
- Natural Language Control

**Übermorgen:**
- VLM Integration (Computer Vision)
- Hardware Auto-Discovery Design

---

## Ressourcen

### Installiert:
- PX4 Autopilot v1.15: `~/Repos/PX4-Autopilot`
- ROS2 Jazzy: `/opt/ros/jazzy`
- Gazebo: `/opt/ros/jazzy/opt/gz_tools_vendor`

### Logs:
- PX4 Setup: `/tmp/px4_setup.log`
- PX4 Build: `/tmp/px4_build.log`

### Dokumentation:
- `/home/tristan/Repos/Humanoid/DRONE_PROJECT_PLAN.md`
- `/home/tristan/Repos/Humanoid/DRONE_SETUP_GUIDE.md`
- `/home/tristan/Repos/Humanoid/DRONE_SETUP_STATUS.md`

### GitHub Repos (zum Referenzieren):
- PX4 + ROS2 Tutorial: https://github.com/nhma20/px4-ros2-gazebo-simulation
- OpenVLA: https://github.com/openvla/openvla
- Wall-X: https://github.com/X-Square-Robot/wall-x

---

## Nächstes Mal

```bash
# Check Setup Status:
cat /tmp/px4_setup.log | tail -20

# Compile PX4:
cd ~/Repos/PX4-Autopilot
make px4_sitl gz_x500

# Fly first drone:
# → Gazebo window opens
# → Type: commander takeoff
# → 🚁 DRONE FLIEGT!
```

---

**Status:** Setup läuft, ~10-15 Minuten bis Drohne fliegt
**Bereit für:** Natural Language Control (wie MeArm!)
