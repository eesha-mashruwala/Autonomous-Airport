#!/usr/bin/env python
"""Quick test to verify visual simulation works correctly."""

import sys
import matplotlib
matplotlib.use('TkAgg')  # Ensure we use a GUI backend
import matplotlib.pyplot as plt

# Import the simulation
from autonomous_turnaround_sim import run_visual_sim

if __name__ == "__main__":
    print("Starting visual simulation test...")
    print("This will run for 100 frames (~10 seconds)")
    print("You should see:")
    print("  1. A 3D visualization window with the airspace")
    print("  2. Aircraft telemetry windows popping up as planes spawn")
    print("  3. Terminal showing status updates only")
    print("\nStarting in 2 seconds...")
    
    import time
    time.sleep(2)
    
    try:
        fig, ani = run_visual_sim(max_frames=100, show=True)
        print("\nSimulation windows opened successfully!")
        print("Close the 3D visualization window to exit.")
    except Exception as e:
        print(f"\nError occurred: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
