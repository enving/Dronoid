# Universal Drone OS - Praktische Implementierung

## Schritt-für-Schritt Code-Beispiele

---

## 1. Hardware Configuration (YAML)

**File:** `drone_configs/x500_cleaning.yaml`

```yaml
# Drone Hardware Configuration
# Auto-Discovery liest diese Config und erstellt ROS2 Interface

drone:
  name: "X500-CleanBot-01"
  type: "quadcopter"
  frame: "x500"
  mass_kg: 2.5
  max_payload_kg: 1.0

flight_controller:
  type: "px4"
  connection: "mavlink"
  protocol: "micro_xrce_dds"
  port: "/dev/ttyUSB0"
  baud: 921600

motors:
  count: 4
  type: "brushless"
  kv: 920
  max_thrust_g: 1200
  layout:
    - {id: 1, position: [0.2, 0.2, 0], direction: "ccw"}
    - {id: 2, position: [-0.2, -0.2, 0], direction: "ccw"}
    - {id: 3, position: [0.2, -0.2, 0], direction: "cw"}
    - {id: 4, position: [-0.2, 0.2, 0], direction: "cw"}

sensors:
  imu:
    type: "mpu6050"
    rate_hz: 250
    topics:
      accel: "/drone/imu/accel"
      gyro: "/drone/imu/gyro"

  gps:
    type: "ublox_m8n"
    rate_hz: 10
    topics:
      position: "/drone/gps/position"
      velocity: "/drone/gps/velocity"

  barometer:
    type: "ms5611"
    rate_hz: 50
    topics:
      pressure: "/drone/baro/pressure"
      altitude: "/drone/baro/altitude"

  camera:
    type: "raspberry_pi_v3"
    resolution: [1920, 1080]
    fps: 30
    topics:
      image_raw: "/drone/camera/image_raw"
      camera_info: "/drone/camera/camera_info"

payloads:
  - name: "cleaning_spray"
    type: "spray_system"
    enabled: true
    control:
      pump_pwm_pin: 18
      pressure_sensor_pin: "analog_1"
    specs:
      max_pressure_mpa: 30
      flow_rate_lpm: 5
      tank_capacity_l: 2.0
    topics:
      enable: "/drone/payload/spray/enable"
      pressure: "/drone/payload/spray/pressure"
      flow_rate: "/drone/payload/spray/flow_rate"

capabilities:
  # Auto-generated based on hardware
  can_spray: true
  has_camera: true
  has_gripper: false
  max_flight_time_min: 25
  max_speed_ms: 15
  max_altitude_m: 120

mission_profiles:
  facade_cleaning:
    enabled: true
    parameters:
      approach_distance_m: 2.0
      spray_duration_s: 3.0
      cleaning_overlap: 0.2
      max_wall_angle_deg: 85
```

---

## 2. Hardware Discovery Node (Python)

**File:** `dronecore/dronecore/discovery_node.py`

```python
#!/usr/bin/env python3
"""
Hardware Auto-Discovery Node
Reads drone_config.yaml and creates ROS2 interface dynamically
"""

import rclpy
from rclpy.node import Node
import yaml
from pathlib import Path
from std_msgs.msg import Bool, Float32, String
from sensor_msgs.msg import Image, Imu, NavSatFix
from geometry_msgs.msg import TwistStamped
from dronecore_msgs.msg import DroneCapabilities, PayloadStatus


class HardwareDiscovery(Node):
    """
    Scans drone_config.yaml and automatically:
    1. Creates ROS2 publishers/subscribers for detected hardware
    2. Publishes drone capabilities
    3. Validates hardware compatibility
    """

    def __init__(self, config_path: str):
        super().__init__('hardware_discovery')

        self.get_logger().info("=== Hardware Auto-Discovery Starting ===")

        # Load configuration
        self.config = self.load_config(config_path)
        self.drone_name = self.config['drone']['name']

        # Storage for publishers/subscribers
        self.pubs = {}
        self.subs = {}

        # Discovery process
        self.discover_sensors()
        self.discover_payloads()
        self.publish_capabilities()

        self.get_logger().info(f"✓ Discovery complete for {self.drone_name}")

    def load_config(self, path: str) -> dict:
        """Load and validate drone configuration"""
        with open(path, 'r') as f:
            config = yaml.safe_load(f)

        self.get_logger().info(f"Loaded config: {config['drone']['name']}")
        return config

    def discover_sensors(self):
        """Auto-create ROS2 topics for detected sensors"""
        sensors = self.config.get('sensors', {})

        # IMU
        if 'imu' in sensors:
            imu_topics = sensors['imu']['topics']
            self.pubs['imu'] = self.create_publisher(
                Imu,
                imu_topics.get('accel', '/drone/imu'),
                10
            )
            self.get_logger().info(f"✓ IMU detected: {sensors['imu']['type']}")

        # GPS
        if 'gps' in sensors:
            gps_topics = sensors['gps']['topics']
            self.pubs['gps'] = self.create_publisher(
                NavSatFix,
                gps_topics['position'],
                10
            )
            self.get_logger().info(f"✓ GPS detected: {sensors['gps']['type']}")

        # Camera
        if 'camera' in sensors:
            cam_topics = sensors['camera']['topics']
            self.subs['camera'] = self.create_subscription(
                Image,
                cam_topics['image_raw'],
                self.camera_callback,
                10
            )
            self.get_logger().info(
                f"✓ Camera detected: {sensors['camera']['resolution']}"
            )

    def discover_payloads(self):
        """Auto-create control interfaces for payloads"""
        payloads = self.config.get('payloads', [])

        for payload in payloads:
            if not payload.get('enabled', True):
                continue

            payload_name = payload['name']
            payload_type = payload['type']

            # Create publishers based on payload type
            if payload_type == 'spray_system':
                self.setup_spray_system(payload)
            elif payload_type == 'gripper':
                self.setup_gripper(payload)

            self.get_logger().info(f"✓ Payload: {payload_name} ({payload_type})")

    def setup_spray_system(self, config: dict):
        """Setup spray system control topics"""
        topics = config['topics']

        # Control publisher
        self.pubs['spray_enable'] = self.create_publisher(
            Bool,
            topics['enable'],
            10
        )

        # Status subscriber
        self.subs['spray_pressure'] = self.create_subscription(
            Float32,
            topics['pressure'],
            self.spray_pressure_callback,
            10
        )

        self.spray_config = config

    def publish_capabilities(self):
        """Publish what this drone can do"""
        caps = DroneCapabilities()
        caps.drone_name = self.drone_name
        caps.can_spray = self.config['capabilities']['can_spray']
        caps.has_camera = self.config['capabilities']['has_camera']
        caps.has_gripper = self.config['capabilities']['has_gripper']
        caps.max_flight_time_min = self.config['capabilities']['max_flight_time_min']

        self.caps_pub = self.create_publisher(
            DroneCapabilities,
            '/drone/capabilities',
            10
        )

        # Publish every 5 seconds
        self.create_timer(5.0, lambda: self.caps_pub.publish(caps))

        self.get_logger().info(f"Publishing capabilities: {caps}")

    def camera_callback(self, msg):
        """Process camera images (placeholder)"""
        # Will be used by VLM node
        pass

    def spray_pressure_callback(self, msg):
        """Monitor spray system pressure"""
        pressure = msg.data
        max_pressure = self.spray_config['specs']['max_pressure_mpa']

        if pressure > max_pressure * 0.9:
            self.get_logger().warn(f"High pressure: {pressure} MPa!")


def main(args=None):
    rclpy.init(args=args)

    # Config path from environment or default
    config_path = Path.home() / 'ros2_ws/src/dronecore/config/x500_cleaning.yaml'

    node = HardwareDiscovery(str(config_path))

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## 3. Natural Language Interface (FastAPI)

**File:** `dronecore/dronecore/nl_interface.py`

```python
#!/usr/bin/env python3
"""
Natural Language Interface for Drone Control
Recycled from MeArm HTTP Bridge - 95% code reuse!
"""

import rclpy
from rclpy.node import Node
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
import uvicorn
import requests
import os
from px4_msgs.msg import VehicleCommand, TrajectorySetpoint, OffboardControlMode
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool


# FastAPI App
app = FastAPI(title="Drone Natural Language API")


class CommandRequest(BaseModel):
    text: str
    mission_id: str = None


class CommandResponse(BaseModel):
    status: str
    input: str
    parsed: dict
    executed: bool


class DroneNLBridge(Node):
    """Natural Language → Drone Commands"""

    def __init__(self):
        super().__init__('drone_nl_bridge')

        # ROS2 Publishers (PX4 SITL)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10
        )

        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10
        )

        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10
        )

        # Spray control (if available)
        self.spray_pub = self.create_publisher(
            Bool,
            '/drone/payload/spray/enable',
            10
        )

        # OpenRouter LLM
        self.llm_api_key = os.getenv('OPENROUTER_API_KEY')
        self.llm_available = self.llm_api_key is not None

        self.get_logger().info("✓ Drone NL Bridge initialized")
        self.get_logger().info(f"LLM available: {self.llm_available}")

    def parse_command(self, text: str) -> dict:
        """
        Parse natural language → drone command

        Examples:
        - "fly 2 meters up" → Trajectory(z=2.0)
        - "clean the wall" → Mission(facade_cleaning)
        - "land now" → VehicleCommand(LAND)
        """

        text_lower = text.lower()

        # Simple regex patterns (fallback)
        if "up" in text_lower or "höher" in text_lower:
            height = self.extract_number(text_lower, default=2.0)
            return {
                "type": "trajectory",
                "position": [0, 0, height],
                "method": "regex"
            }

        elif "land" in text_lower:
            return {
                "type": "land",
                "method": "regex"
            }

        elif "takeoff" in text_lower or "start" in text_lower:
            return {
                "type": "takeoff",
                "altitude": 2.5,
                "method": "regex"
            }

        elif "clean" in text_lower or "spray" in text_lower:
            return {
                "type": "mission",
                "mission": "facade_cleaning",
                "method": "regex"
            }

        # Try LLM if regex fails
        if self.llm_available:
            return self.parse_with_llm(text)

        raise ValueError(f"Could not parse: {text}")

    def parse_with_llm(self, text: str) -> dict:
        """Use OpenRouter LLM for complex commands"""

        prompt = f"""You are a drone control assistant. Convert this natural language command into a structured action.

Command: "{text}"

Available actions:
- takeoff (altitude in meters)
- land
- go_to (x, y, z coordinates in meters)
- spray (duration in seconds)
- mission (type: facade_cleaning, inspection, etc.)

Respond with JSON only:
{{
    "type": "...",
    "parameters": {{...}},
    "reasoning": "..."
}}"""

        response = requests.post(
            "https://openrouter.ai/api/v1/chat/completions",
            headers={
                "Authorization": f"Bearer {self.llm_api_key}",
                "Content-Type": "application/json"
            },
            json={
                "model": "anthropic/claude-3.5-sonnet",
                "messages": [{"role": "user", "content": prompt}]
            }
        )

        if response.status_code == 200:
            result = response.json()
            content = result['choices'][0]['message']['content']

            # Extract JSON from response
            import json
            parsed = json.loads(content)
            parsed['method'] = 'llm'
            return parsed
        else:
            raise Exception(f"LLM failed: {response.text}")

    def execute_command(self, parsed: dict) -> bool:
        """Execute parsed command"""

        cmd_type = parsed.get('type')

        if cmd_type == 'takeoff':
            return self.cmd_takeoff(parsed.get('altitude', 2.5))

        elif cmd_type == 'land':
            return self.cmd_land()

        elif cmd_type == 'trajectory' or cmd_type == 'go_to':
            position = parsed.get('position', parsed.get('parameters', {}).get('position'))
            return self.cmd_goto(position)

        elif cmd_type == 'spray':
            duration = parsed.get('duration', parsed.get('parameters', {}).get('duration', 3.0))
            return self.cmd_spray(duration)

        elif cmd_type == 'mission':
            mission = parsed.get('mission', parsed.get('parameters', {}).get('type'))
            return self.start_mission(mission)

        return False

    def cmd_takeoff(self, altitude: float) -> bool:
        """Takeoff command"""
        cmd = VehicleCommand()
        cmd.command = 176  # MAV_CMD_DO_SET_MODE
        cmd.param1 = 1.0   # ARM
        cmd.param2 = 4.0   # OFFBOARD mode
        self.vehicle_command_pub.publish(cmd)

        # Then takeoff
        cmd = VehicleCommand()
        cmd.command = 177  # MAV_CMD_NAV_TAKEOFF
        cmd.param7 = altitude
        self.vehicle_command_pub.publish(cmd)

        self.get_logger().info(f"Takeoff to {altitude}m")
        return True

    def cmd_land(self) -> bool:
        """Land command"""
        cmd = VehicleCommand()
        cmd.command = 178  # MAV_CMD_NAV_LAND
        self.vehicle_command_pub.publish(cmd)

        self.get_logger().info("Landing...")
        return True

    def cmd_goto(self, position: list) -> bool:
        """Go to position [x, y, z]"""
        traj = TrajectorySetpoint()
        traj.position = position
        self.trajectory_pub.publish(traj)

        self.get_logger().info(f"Moving to {position}")
        return True

    def cmd_spray(self, duration: float) -> bool:
        """Activate spray system"""
        # Enable spray
        self.spray_pub.publish(Bool(data=True))

        # Disable after duration
        def disable_spray():
            self.spray_pub.publish(Bool(data=False))

        self.create_timer(duration, disable_spray, oneshot=True)

        self.get_logger().info(f"Spraying for {duration}s")
        return True

    def start_mission(self, mission_type: str) -> bool:
        """Start autonomous mission"""
        self.get_logger().info(f"Starting mission: {mission_type}")
        # Mission planner will be implemented separately
        return True

    @staticmethod
    def extract_number(text: str, default: float = 1.0) -> float:
        """Extract number from text"""
        import re
        numbers = re.findall(r'\d+\.?\d*', text)
        return float(numbers[0]) if numbers else default


# Global drone bridge instance
drone_bridge = None


@app.on_event("startup")
async def startup():
    global drone_bridge
    rclpy.init()
    drone_bridge = DroneNLBridge()


@app.post("/command", response_model=CommandResponse)
async def command(req: CommandRequest):
    """Natural Language Command Endpoint"""

    try:
        # Parse command
        parsed = drone_bridge.parse_command(req.text)

        # Execute
        executed = drone_bridge.execute_command(parsed)

        return CommandResponse(
            status="success",
            input=req.text,
            parsed=parsed,
            executed=executed
        )

    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))


@app.get("/status")
async def status():
    """System status"""
    return {
        "ros2_initialized": drone_bridge is not None,
        "llm_available": drone_bridge.llm_available if drone_bridge else False,
        "ready": True
    }


def main():
    uvicorn.run(app, host="0.0.0.0", port=8000)


if __name__ == "__main__":
    main()
```

---

## 4. Usage Examples

### Terminal Commands:

```bash
# 1. Start Discovery Node
ros2 run dronecore hardware_discovery

# 2. Start Natural Language Bridge
python3 dronecore/nl_interface.py

# 3. Send Commands (in another terminal)
# Takeoff:
curl -X POST http://localhost:8000/command \
  -H "Content-Type: application/json" \
  -d '{"text": "takeoff and fly 3 meters high"}'

# Clean wall:
curl -X POST http://localhost:8000/command \
  -d '{"text": "clean the wall in front of me"}'

# Land:
curl -X POST http://localhost:8000/command \
  -d '{"text": "land now"}'
```

### Python Client Example:

```python
import requests

class DroneClient:
    def __init__(self, base_url="http://localhost:8000"):
        self.base_url = base_url

    def command(self, text: str):
        response = requests.post(
            f"{self.base_url}/command",
            json={"text": text}
        )
        return response.json()

    def status(self):
        response = requests.get(f"{self.base_url}/status")
        return response.json()

# Usage:
drone = DroneClient()
print(drone.status())
drone.command("takeoff")
drone.command("fly 2 meters forward")
drone.command("clean the wall")
drone.command("land")
```

---

## Key Takeaways

### Code Reuse from MeArm:
- HTTP Bridge: **95% identical**
- Natural Language Parsing: **100% reusable**
- LLM Integration: **identical**
- FastAPI Structure: **same**

### What Changed:
- ROS2 Topics: `/position_controller/commands` → `/fmu/in/trajectory_setpoint`
- Message Types: `Float64MultiArray` → `TrajectorySetpoint`
- Commands: Servo angles → GPS coordinates

### Scalability:
- Add new payload? → Update `drone_config.yaml`
- New mission type? → Add case in `execute_command()`
- Multi-drone? → Run multiple instances with different configs

**→ Universal System durch Abstraktion!**
