#!/usr/bin/env python3
"""
Natural Language Drohnensteuerung
Text-Kommandos wie "takeoff", "land", "fly up", etc.
Unterstützt OpenRouter Cloud LLM für fortgeschrittenes NL Understanding
"""

from pymavlink import mavutil
import time
import os
import json
import re
from pathlib import Path

# Load environment variables
try:
    from dotenv import load_dotenv
    load_dotenv()
    OPENROUTER_AVAILABLE = True
except ImportError:
    OPENROUTER_AVAILABLE = False

# OpenRouter Configuration
OPENROUTER_API_KEY = os.getenv('OPENROUTER_API_KEY')
OPENROUTER_BASE_URL = os.getenv('OPENROUTER_BASE_URL', 'https://openrouter.ai/api/v1')
OPENROUTER_MODEL = os.getenv('OPENROUTER_MODEL', 'google/gemini-flash-1.5:free')

# PX4 Verbindung (bidirektional über Port 14580)
print("Verbinde zu PX4...")
master = mavutil.mavlink_connection('udpout:127.0.0.1:14580', source_system=255)

# Robuster Verbindungsaufbau mit Retry
print("Triggere Heartbeat...")
connected = False
max_retries = 10

for retry in range(max_retries):
    # Send REQUEST_DATA_STREAM to trigger response
    master.mav.request_data_stream_send(0, 0, mavutil.mavlink.MAV_DATA_STREAM_ALL, 1, 1)

    # Wait for heartbeat with short timeout
    msg = master.wait_heartbeat(timeout=2)

    if msg:
        connected = True
        break

    if retry < max_retries - 1:
        print(f"  Retry {retry + 1}/{max_retries - 1}...")
        time.sleep(1)

if not connected:
    print("\n❌ FEHLER: Keine Verbindung zu PX4!")
    print("   Mögliche Gründe:")
    print("   1. PX4 läuft nicht → ./start_drone.sh")
    print("   2. PX4 startet noch → warte 10 Sekunden und versuch nochmal")
    print("   3. Port blockiert → killall px4 && ./start_drone.sh")
    exit(1)

print(f"✓ Verbunden mit PX4 System {master.target_system}")

def arm():
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    time.sleep(1)

def takeoff(altitude=5):
    arm()
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, altitude
    )

def land():
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0
    )

def parse_with_llm(text):
    """Parse command using OpenRouter Cloud LLM"""
    if not OPENROUTER_API_KEY:
        return None

    try:
        import requests

        prompt = f"""You are a drone control parser. Convert natural language to drone commands.

User input: "{text}"

Return ONLY a JSON object with this format:
{{"action": "takeoff|land|arm|unknown", "altitude": 5}}

Examples:
- "takeoff" -> {{"action": "takeoff", "altitude": 5}}
- "fly up to 10m" -> {{"action": "takeoff", "altitude": 10}}
- "land" -> {{"action": "land"}}
- "arm motors" -> {{"action": "arm"}}

Be smart about synonyms (abheben=takeoff, landen=land, etc.)"""

        response = requests.post(
            f"{OPENROUTER_BASE_URL}/chat/completions",
            headers={
                "Authorization": f"Bearer {OPENROUTER_API_KEY}",
                "Content-Type": "application/json"
            },
            json={
                "model": OPENROUTER_MODEL,
                "messages": [{"role": "user", "content": prompt}]
            },
            timeout=5
        )

        if response.status_code == 200:
            result = response.json()
            content = result['choices'][0]['message']['content']
            # Extract JSON from response
            json_match = re.search(r'\{.*\}', content, re.DOTALL)
            if json_match:
                return json.loads(json_match.group())

        return None
    except Exception as e:
        print(f"⚠️  LLM Error: {e}")
        return None

def parse_command_regex(text):
    """Parse natural language command with regex (fallback)"""
    text = text.lower().strip()

    # Takeoff
    if any(word in text for word in ['takeoff', 'take off', 'start', 'abheben', 'fliege hoch', 'fly up']):
        altitude_match = re.search(r'(\d+)\s*m', text)
        if altitude_match:
            return {"action": "takeoff", "altitude": float(altitude_match.group(1))}
        else:
            return {"action": "takeoff", "altitude": 5}

    # Land
    elif any(word in text for word in ['land', 'landen', 'come down', 'runter']):
        return {"action": "land"}

    # Arm
    elif 'arm' in text:
        return {"action": "arm"}

    else:
        return {"action": "unknown"}

def parse_command(text):
    """Parse natural language command (tries LLM first, then regex)"""

    # Try LLM first if available
    if OPENROUTER_API_KEY:
        result = parse_with_llm(text)
        if result:
            action = result.get('action')
            altitude = result.get('altitude', 5)

            if action == 'takeoff':
                print(f"✈️  Taking off to {altitude}m... (via LLM)")
                takeoff(altitude)
                return True
            elif action == 'land':
                print("🛬 Landing... (via LLM)")
                land()
                return True
            elif action == 'arm':
                print("🔧 Arming... (via LLM)")
                arm()
                return True

    # Fallback to regex
    result = parse_command_regex(text)
    action = result.get('action')
    altitude = result.get('altitude', 5)

    if action == 'takeoff':
        print(f"✈️  Taking off to {altitude}m...")
        takeoff(altitude)
        return True
    elif action == 'land':
        print("🛬 Landing...")
        land()
        return True
    elif action == 'arm':
        print("🔧 Arming...")
        arm()
        return True
    else:
        print(f"❌ Kommando nicht verstanden: '{text}'")
        print("Verfügbare Kommandos: takeoff, land, arm, fly up, abheben, landen")
        return False

# Hauptprogramm
print("="*50)
print("  Natural Language Drohnensteuerung")
print("="*50)
print()
print("✓ Verbunden mit PX4")

# Show LLM status
if OPENROUTER_API_KEY:
    print(f"🤖 OpenRouter LLM: AKTIV ({OPENROUTER_MODEL})")
else:
    print("⚡ Regex Parser: AKTIV (schnell & leichtgewichtig)")
print()
print("Beispiele:")
print('  • "takeoff"')
print('  • "fly up" / "abheben"')
print('  • "takeoff to 10m"')
print('  • "land" / "landen"')
print('  • "arm"')
print()
print("Schreibe 'q' zum Beenden")
print()

while True:
    try:
        cmd = input("💬 Kommando: ").strip()
        
        if cmd.lower() in ['q', 'quit', 'exit']:
            print("👋 Auf Wiedersehen!")
            break
        
        if cmd:
            parse_command(cmd)
            print()
        
    except KeyboardInterrupt:
        print("\n👋 Auf Wiedersehen!")
        break
