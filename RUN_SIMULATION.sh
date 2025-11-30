#!/bin/bash

echo "╔════════════════════════════════════════════════════════════════╗"
echo "║        AUTONOMOUS TURNAROUND SIMULATION - LAUNCHER             ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""
echo "Starting simulation with:"
echo "  ✓ 3D airspace visualization"
echo "  ✓ Aircraft telemetry windows"
echo "  ✓ Live animation"
echo ""
echo "Close the 3D window to exit."
echo "Press Ctrl+C to stop."
echo ""
echo "Starting in 2 seconds..."
sleep 2
echo ""

python autonomous_turnaround_sim.py

echo ""
echo "Simulation ended."
