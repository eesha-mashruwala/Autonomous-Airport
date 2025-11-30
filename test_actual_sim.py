#!/usr/bin/env python
"""
Direct test of the autonomous_turnaround_sim with visual confirmation.
This will run the actual simulation and show it's working.
"""

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

print("="*70)
print("AUTONOMOUS TURNAROUND SIMULATION - LIVE TEST")
print("="*70)
print("\nThis will run the ACTUAL simulation with:")
print("  • 3D airspace visualization")
print("  • Aircraft telemetry windows (spawn as planes arrive)")
print("  • Live animation for 500 frames (~50 seconds)")
print("\nYou should see:")
print("  1. Main 3D window opens immediately")
print("  2. Aircraft telemetry windows pop up as planes spawn")
print("  3. Aircraft moving through the airspace")
print("  4. Terminal showing status updates")
print("\nStarting in 3 seconds...")
print("="*70)

import time
time.sleep(3)

# Import and run the actual simulation
from autonomous_turnaround_sim import run_visual_sim

print("\n🚀 Starting simulation...")
print("="*70)

try:
    fig, ani = run_visual_sim(max_frames=500, show=True)
    print("\n✓ Simulation completed successfully!")
    print("✓ Windows opened and animation ran")
except KeyboardInterrupt:
    print("\n\n⚠ Simulation stopped by user (Ctrl+C)")
except Exception as e:
    print(f"\n✗ Error: {e}")
    import traceback
    traceback.print_exc()
