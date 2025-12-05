#!/usr/bin/env python3
"""
Einfache Drohnensteuerung via MAVLink
"""

from pymavlink import mavutil
import time
import sys

# Verbinde zu PX4 (Port 14580 für Offboard API)
print("Verbinde zu PX4 auf UDP 14580...")
master = mavutil.mavlink_connection('udpout:127.0.0.1:14580', source_system=255)

# Trigger Heartbeat durch REQUEST_DATA_STREAM
print("Trigger Connection...")
for i in range(3):
    master.mav.request_data_stream_send(0, 0, mavutil.mavlink.MAV_DATA_STREAM_ALL, 1, 1)
    time.sleep(0.3)

# Warte auf Heartbeat
print("Warte auf Heartbeat...")
master.wait_heartbeat()
print(f"✓ Verbunden! System {master.target_system}, Component {master.target_component}")
print()

def arm():
    """Drohne armen"""
    print("Arming drone...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    time.sleep(2)
    print("✓ Armed")

def takeoff(altitude=5):
    """Takeoff zu bestimmter Höhe"""
    print(f"Taking off to {altitude}m...")
    arm()
    
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, altitude
    )
    print("✓ Takeoff command sent - check Gazebo!")

def land():
    """Landen"""
    print("Landing...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    print("✓ Land command sent")

def status():
    """Status anzeigen"""
    print("\n=== Drone Status ===")
    print(f"System: {master.target_system}")
    print(f"Component: {master.target_component}")
    
    # Request attitude
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_POSITION,
        1, 1
    )

# Interaktives Menü
if len(sys.argv) > 1:
    cmd = sys.argv[1].lower()
    if cmd == 'takeoff':
        takeoff()
    elif cmd == 'land':
        land()
    elif cmd == 'arm':
        arm()
    elif cmd == 'status':
        status()
    else:
        print(f"Unknown command: {cmd}")
        print("Usage: control_drone.py [takeoff|land|arm|status]")
else:
    print("========================================")
    print("  Drohnensteuerung")
    print("========================================")
    print()
    print("Kommandos:")
    print("  1 - Takeoff (5m)")
    print("  2 - Land")
    print("  3 - Arm")
    print("  4 - Status")
    print("  q - Beenden")
    print()
    
    while True:
        try:
            cmd = input("Kommando: ").strip()
            
            if cmd == '1':
                takeoff()
            elif cmd == '2':
                land()
            elif cmd == '3':
                arm()
            elif cmd == '4':
                status()
            elif cmd.lower() == 'q':
                print("Beendet.")
                break
            else:
                print("Ungültig. Nutze 1, 2, 3, 4 oder q")
            
            print()
        except KeyboardInterrupt:
            print("\nBeendet.")
            break
