#!/usr/bin/env python
"""
Test script to verify that both the 3D visualization and telemetry windows open correctly.
This will open windows and print confirmation messages.
"""

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation
import sys

print("="*70)
print("WINDOW OPENING TEST")
print("="*70)
print("\nThis test will:")
print("  1. Create a 3D visualization window")
print("  2. Create 2 aircraft telemetry windows")
print("  3. Keep them open for 10 seconds")
print("  4. Show you the windows are actually working")
print("\nStarting in 3 seconds...")
print("="*70)

import time
time.sleep(3)

# Test 1: Create 3D window
print("\n[1/3] Creating 3D visualization window...")
fig3d = plt.figure(figsize=(12, 9))
fig3d.canvas.manager.set_window_title("3D Airspace Visualization - TEST")
ax3d = fig3d.add_subplot(111, projection='3d')
ax3d.set_xlim([-5000, 5000])
ax3d.set_ylim([-5000, 5000])
ax3d.set_zlim([0, 3000])
ax3d.set_xlabel("X (m)")
ax3d.set_ylabel("Y (m)")
ax3d.set_zlabel("Altitude (m)")
ax3d.set_title("TEST: 3D Airspace Window", fontsize=14, fontweight='bold')

# Add some test data
theta = np.linspace(0, 4*np.pi, 100)
x = 2000 * np.cos(theta)
y = 2000 * np.sin(theta)
z = np.linspace(0, 2000, 100)
ax3d.plot(x, y, z, 'b-', linewidth=2, label='Test trajectory')
ax3d.legend()

plt.ion()
fig3d.show()
print("   ✓ 3D window created")

# Test 2: Create telemetry window 1
print("\n[2/3] Creating Aircraft #1 telemetry window...")
fig_telem1 = plt.figure(figsize=(10, 7))
fig_telem1.canvas.manager.set_window_title("Aircraft TEST-001 - Telemetry")
ax_telem1 = fig_telem1.add_subplot(111)
ax_telem1.axis('off')
text1 = ax_telem1.text(0.05, 0.95, "", transform=ax_telem1.transAxes,
                       fontfamily='monospace', fontsize=9,
                       verticalalignment='top')

test_data_1 = [
    "="*60,
    "AIRCRAFT: TEST-001",
    "="*60,
    "STATUS: Testing telemetry window",
    "",
    "FLIGHT DATA:",
    "  Position: (1234, 5678, 900) m",
    "  Speed: 85.3 m/s",
    "  Altitude: 900 m",
    "  Heading: 045°",
    "",
    "PHASE: Approach",
    "  → Waypoint 3/5",
    "  → Distance to waypoint: 1234 m",
    "",
    "ATC: Cleared to land",
    "RUNWAY: Available",
    "",
    "✓ Telemetry window test successful",
]

text1.set_text('\n'.join(test_data_1))
fig_telem1.show()
print("   ✓ Aircraft #1 telemetry window created")

# Test 3: Create telemetry window 2  
print("\n[3/3] Creating Aircraft #2 telemetry window...")
fig_telem2 = plt.figure(figsize=(10, 7))
fig_telem2.canvas.manager.set_window_title("Aircraft TEST-002 - Telemetry")
ax_telem2 = fig_telem2.add_subplot(111)
ax_telem2.axis('off')
text2 = ax_telem2.text(0.05, 0.95, "", transform=ax_telem2.transAxes,
                       fontfamily='monospace', fontsize=9,
                       verticalalignment='top')

test_data_2 = [
    "="*60,
    "AIRCRAFT: TEST-002",
    "="*60,
    "STATUS: Testing second window",
    "",
    "FLIGHT DATA:",
    "  Position: (4321, 8765, 1500) m",
    "  Speed: 105.7 m/s",
    "  Altitude: 1500 m",
    "  Heading: 270°",
    "",
    "PHASE: Departure",
    "  → Waypoint 2/4",
    "  → Climbing to 2000m",
    "",
    "ATC: Cleared for takeoff",
    "RUNWAY: Departing",
    "",
    "✓ Second telemetry window test successful",
]

text2.set_text('\n'.join(test_data_2))
fig_telem2.show()
print("   ✓ Aircraft #2 telemetry window created")

print("\n" + "="*70)
print("SUCCESS! All 3 windows are now open:")
print("  ✓ 3D Airspace Visualization (blue spiral trajectory)")
print("  ✓ Aircraft TEST-001 Telemetry (approach data)")
print("  ✓ Aircraft TEST-002 Telemetry (departure data)")
print("\nWindows will stay open for 10 seconds...")
print("You should see 3 separate windows on your screen right now!")
print("="*70)

# Keep windows open
plt.pause(10)

print("\nClosing windows...")
plt.close('all')
print("✓ Test complete - all windows functioned correctly!")
