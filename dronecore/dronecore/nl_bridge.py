#!/usr/bin/env python3
"""
Natural Language Bridge für Drohnensteuerung
HTTP Server der Text-Kommandos empfängt und an DroneCommander weiterleitet

Beispiel:
  curl -X POST http://localhost:8080/command -d '{"text": "fly 5 meters up"}'
"""

import json
import rclpy
from rclpy.node import Node
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
import uvicorn
import threading
import re
from typing import Optional

# Optional: OpenRouter für LLM-basierte Kommando-Parsing
try:
    import httpx
    HTTPX_AVAILABLE = True
except ImportError:
    HTTPX_AVAILABLE = False
    print("Warning: httpx not installed. LLM parsing disabled.")


class CommandRequest(BaseModel):
    text: str
    use_llm: bool = False


class DroneNLBridge(Node):
    """Natural Language Interface für Drohne"""

    def __init__(self, drone_commander):
        super().__init__('drone_nl_bridge')
        self.drone = drone_commander
        self.openrouter_api_key = None

        self.get_logger().info('Natural Language Bridge initialized')

    def parse_command_simple(self, text: str) -> dict:
        """
        Einfaches Pattern-Matching für Drohnen-Kommandos
        Beispiele:
          "takeoff" / "start" / "fly up" -> takeoff
          "land" / "come down" -> land
          "fly 5 meters up" -> takeoff(5)
          "go to 10 20 5" -> goto(10, 20, -5)
          "arm" -> arm
        """
        text = text.lower().strip()

        # Takeoff detection
        if any(word in text for word in ['takeoff', 'take off', 'start', 'abheben']):
            # Check for altitude
            altitude_match = re.search(r'(\d+\.?\d*)\s*m', text)
            if altitude_match:
                altitude = float(altitude_match.group(1))
                return {'action': 'takeoff', 'altitude': altitude}
            return {'action': 'takeoff', 'altitude': 5.0}

        # Land detection
        if any(word in text for word in ['land', 'landen', 'come down', 'runter']):
            return {'action': 'land'}

        # Arm/Disarm
        if 'arm' in text and 'disarm' not in text:
            return {'action': 'arm'}
        if 'disarm' in text:
            return {'action': 'disarm'}

        # Position control: "go to X Y Z"
        goto_match = re.search(r'go to\s+(-?\d+\.?\d*)\s+(-?\d+\.?\d*)\s+(-?\d+\.?\d*)', text)
        if goto_match:
            x = float(goto_match.group(1))
            y = float(goto_match.group(2))
            z = float(goto_match.group(3))
            return {'action': 'goto', 'x': x, 'y': y, 'z': -abs(z)}  # NED frame: down is negative

        # Move relative: "move forward 5 meters"
        move_match = re.search(r'(forward|backward|left|right|up|down)\s+(\d+\.?\d*)', text)
        if move_match:
            direction = move_match.group(1)
            distance = float(move_match.group(2))
            return {'action': 'move_relative', 'direction': direction, 'distance': distance}

        # Status check
        if any(word in text for word in ['status', 'where', 'position', 'info']):
            return {'action': 'status'}

        return {'action': 'unknown', 'error': f'Could not parse command: {text}'}

    async def parse_command_llm(self, text: str) -> dict:
        """LLM-basierte Kommando-Extraktion via OpenRouter"""
        if not HTTPX_AVAILABLE:
            return self.parse_command_simple(text)

        if not self.openrouter_api_key:
            self.get_logger().warn('OpenRouter API key not set, falling back to simple parsing')
            return self.parse_command_simple(text)

        # OpenRouter API Call
        system_prompt = """Du bist ein Drohnen-Steuerungsparser.
Extrahiere aus natürlichsprachlichen Kommandos strukturierte JSON-Befehle.

Mögliche Actions:
- takeoff (altitude in Metern)
- land
- arm / disarm
- goto (x, y, z in Metern, NED frame)
- move_relative (direction: forward/backward/left/right/up/down, distance in Metern)
- status

Beispiele:
"fliege 3 Meter hoch" -> {"action": "takeoff", "altitude": 3.0}
"landen" -> {"action": "land"}
"gehe zu Position 10 20 5" -> {"action": "goto", "x": 10, "y": 20, "z": -5}

Antworte NUR mit JSON, keine Erklärung."""

        try:
            async with httpx.AsyncClient() as client:
                response = await client.post(
                    'https://openrouter.ai/api/v1/chat/completions',
                    headers={
                        'Authorization': f'Bearer {self.openrouter_api_key}',
                        'Content-Type': 'application/json'
                    },
                    json={
                        'model': 'anthropic/claude-3.5-sonnet',
                        'messages': [
                            {'role': 'system', 'content': system_prompt},
                            {'role': 'user', 'content': text}
                        ]
                    },
                    timeout=10.0
                )
                result = response.json()
                content = result['choices'][0]['message']['content']
                return json.loads(content)
        except Exception as e:
            self.get_logger().error(f'LLM parsing failed: {e}')
            return self.parse_command_simple(text)

    def execute_command(self, cmd: dict) -> dict:
        """Führe geparsten Befehl aus"""
        action = cmd.get('action')

        if action == 'takeoff':
            altitude = cmd.get('altitude', 5.0)
            self.drone.takeoff(altitude)
            return {'status': 'success', 'message': f'Taking off to {altitude}m'}

        elif action == 'land':
            self.drone.land()
            return {'status': 'success', 'message': 'Landing'}

        elif action == 'arm':
            self.drone.arm()
            return {'status': 'success', 'message': 'Armed'}

        elif action == 'disarm':
            self.drone.disarm()
            return {'status': 'success', 'message': 'Disarmed'}

        elif action == 'goto':
            x = cmd.get('x', 0.0)
            y = cmd.get('y', 0.0)
            z = cmd.get('z', -5.0)
            self.drone.goto_position(x, y, z)
            return {'status': 'success', 'message': f'Going to [{x}, {y}, {z}]'}

        elif action == 'move_relative':
            # TODO: Implement relative movement
            return {'status': 'error', 'message': 'Relative movement not yet implemented'}

        elif action == 'status':
            status = self.drone.get_status()
            return {'status': 'success', 'data': status}

        else:
            return {'status': 'error', 'message': cmd.get('error', 'Unknown action')}


# FastAPI Server
app = FastAPI(title="Drone Natural Language Bridge")
nl_bridge = None


@app.post("/command")
async def command(req: CommandRequest):
    """Natural Language Kommando verarbeiten"""
    if nl_bridge is None:
        raise HTTPException(status_code=500, detail="ROS2 bridge not initialized")

    # Parse command
    if req.use_llm:
        parsed = await nl_bridge.parse_command_llm(req.text)
    else:
        parsed = nl_bridge.parse_command_simple(req.text)

    # Execute
    result = nl_bridge.execute_command(parsed)

    return {
        'input': req.text,
        'parsed': parsed,
        'result': result
    }


@app.get("/status")
def status():
    """Drohnen-Status abrufen"""
    if nl_bridge is None:
        raise HTTPException(status_code=500, detail="ROS2 bridge not initialized")

    return nl_bridge.drone.get_status()


@app.get("/health")
def health():
    return {"status": "ok"}


def run_fastapi_server():
    """FastAPI Server in separatem Thread starten"""
    uvicorn.run(app, host="0.0.0.0", port=8080, log_level="info")


def main(args=None):
    global nl_bridge

    # ROS2 initialisieren
    rclpy.init(args=args)

    # Import DroneCommander
    from dronecore.drone_commander import DroneCommander

    # Nodes erstellen
    drone_commander = DroneCommander()
    nl_bridge = DroneNLBridge(drone_commander)

    # FastAPI Server in Thread starten
    server_thread = threading.Thread(target=run_fastapi_server, daemon=True)
    server_thread.start()

    nl_bridge.get_logger().info('Natural Language Bridge running on http://0.0.0.0:8080')
    nl_bridge.get_logger().info('Try: curl -X POST http://localhost:8080/command -d \'{"text": "takeoff"}\'')

    try:
        # ROS2 spinning
        rclpy.spin(drone_commander)
    except KeyboardInterrupt:
        nl_bridge.get_logger().info('Shutting down...')
    finally:
        drone_commander.destroy_node()
        nl_bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
