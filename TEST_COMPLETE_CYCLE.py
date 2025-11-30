#!/usr/bin/env python
"""Test showing complete turnaround with full visualization"""

import matplotlib
matplotlib.use('TkAgg')
from autonomous_turnaround_sim import run_visual_sim

print("="*70)
print("COMPLETE TURNAROUND CYCLE TEST")
print("="*70)
print("\nTesting ALL features:")
print("  ✓ Aircraft 3D markers and trails")
print("  ✓ Arrival telemetry (approach, landing)")
print("  ✓ Landing rollout telemetry")
print("  ✓ Gate service")
print("  ✓ Departure telemetry (takeoff, climb)")
print("\nRunning 1500 frames (~150 seconds)...")
print("Close the 3D window to exit.")
print("="*70)

import time
time.sleep(2)

try:
    fig, ani = run_visual_sim(max_frames=1500, show=True)
    print("\n✓ Test complete!")
except KeyboardInterrupt:
    print("\n⚠ Stopped by user")
except Exception as e:
    print(f"\n✗ Error: {e}")
    import traceback
    traceback.print_exc()
