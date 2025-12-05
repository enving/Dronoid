#!/bin/bash
# Automatischer Drohnen-Start für Natural Language Control

echo "=========================================="
echo "  Drohnen-System Start"
echo "=========================================="
echo ""

# Cleanup
echo "1. Stoppe alte Prozesse..."
pkill -9 px4 2>/dev/null
pkill -9 gz 2>/dev/null
sleep 2

# Start PX4 + Gazebo
echo "2. Starte PX4 + Gazebo..."
cd /home/tristan/Repos/Humanoid/PX4-Autopilot
source /opt/ros/jazzy/setup.bash
export GZ_SIM_RESOURCE_PATH=$PWD/Tools/simulation/gz/models:$PWD/Tools/simulation/gz/worlds

# Start in background
make px4_sitl gz_x500 > /tmp/px4_run.log 2>&1 &
PX4_PID=$!

echo "   PX4 startet (PID: $PX4_PID)..."
echo "   Warte 15 Sekunden auf Startup..."
sleep 15

# Check if running
if ps -p $PX4_PID > /dev/null; then
    echo "   ✓ PX4 läuft"
else
    echo "   ✗ PX4 Start fehlgeschlagen!"
    tail -20 /tmp/px4_run.log
    exit 1
fi

# Check for "Ready for takeoff"
if grep -q "Ready for takeoff" /tmp/px4_run.log; then
    echo "   ✓ Drohne bereit für Takeoff!"
else
    echo "   ⚠ Drohne noch nicht ready, warte noch 5 Sek..."
    sleep 5
fi

echo ""
echo "=========================================="
echo "  ✓ System bereit!"
echo "=========================================="
echo ""
echo "Jetzt kannst du die Drohne steuern mit:"
echo "  python3 nl_drone.py"
echo ""
echo "Verfügbare Kommandos:"
echo "  • takeoff       - Abheben (5m)"
echo "  • takeoff to 10m - Abheben (10m)"
echo "  • land          - Landen"
echo "  • q             - Beenden"
echo ""
