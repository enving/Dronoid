#!/usr/bin/env python3
"""
Natural Language Drohnensteuerung
Text-Kommandos wie "takeoff", "land", "fly up", etc.
"""

from pymavlink import mavutil
import time

# PX4 Verbindung (bidirektional über Port 14580)
print("Verbinde zu PX4...")
master = mavutil.mavlink_connection('udpout:127.0.0.1:14580', source_system=255)

# Trigger Heartbeat durch REQUEST_DATA_STREAM
print("Triggere Heartbeat...")
for i in range(3):
    master.mav.request_data_stream_send(0, 0, mavutil.mavlink.MAV_DATA_STREAM_ALL, 1, 1)
    time.sleep(0.3)

print("Warte auf Heartbeat (5 Sekunden)...")
msg = master.wait_heartbeat(timeout=5)

if not msg:
    print("\n❌ FEHLER: Keine Verbindung zu PX4!")
    print("   Stelle sicher, dass PX4 läuft:")
    print("   → ./start_drone.sh")
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

def parse_command(text):
    """Parse natural language command"""
    text = text.lower().strip()
    
    # Takeoff
    if any(word in text for word in ['takeoff', 'take off', 'start', 'abheben', 'fliege hoch', 'fly up']):
        # Extract altitude if mentioned
        import re
        altitude_match = re.search(r'(\d+)\s*m', text)
        if altitude_match:
            alt = float(altitude_match.group(1))
            print(f"✈️  Taking off to {alt}m...")
            takeoff(alt)
        else:
            print("✈️  Taking off to 5m...")
            takeoff(5)
        return True
    
    # Land
    elif any(word in text for word in ['land', 'landen', 'come down', 'runter']):
        print("🛬 Landing...")
        land()
        return True
    
    # Arm
    elif 'arm' in text:
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
