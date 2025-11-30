#!/usr/bin/env python3
"""
Test script to validate takeoff and climb performance.
Tests that aircraft reach rotation speed and maintain/increase speed during climb.
"""

import numpy as np
import matplotlib.pyplot as plt
from aircraft import AirbusA220, EmbraerE170, Dash8_400, ATR72_600
from flight_dynamics import FlightDynamics


def test_aircraft_takeoff_climb(plane):
    """Test takeoff and climb for a specific aircraft."""
    print(f"\n{'='*60}")
    print(f"Testing: {plane.plane_type}")
    print(f"{'='*60}")
    print(f"Mass: {plane.mass:.0f} kg")
    print(f"Static Thrust: {plane._T0:.0f} N")
    print(f"Thrust/Weight: {plane._T0 / (plane.mass * 9.81):.3f}")
    
    # Create flight dynamics instance
    fd = FlightDynamics(plane)
    
    # Test takeoff
    print(f"\n--- TAKEOFF PHASE ---")
    print(f"Target rotation speed: 70 m/s")
    
    takeoff_result = fd.ground_roll_takeoff(dt=0.05, Vr=70.0)
    
    print(f"Final speed: {takeoff_result['V_rot']:.2f} m/s")
    print(f"Distance: {takeoff_result['distance']:.1f} m")
    print(f"Time: {takeoff_result['time']:.2f} s")
    
    if takeoff_result['V_rot'] >= 70.0:
        print("✓ SUCCESS: Reached rotation speed!")
    else:
        print("✗ FAILED: Did not reach rotation speed")
    
    # Get takeoff data
    takeoff_data = fd.get_history()
    
    # Test climb
    print(f"\n--- CLIMB PHASE ---")
    print(f"Target altitude: 500 m, Target speed: 85 m/s")
    
    # Reset for climb
    fd = FlightDynamics(plane)
    fd.V = 70.0  # Start from rotation speed
    
    climb_result = fd.climb_phase(
        t_end=120.0, 
        dt=0.05, 
        target_altitude=500.0,
        gamma=0.087,  # ~5 degrees
        V_target=85.0
    )
    
    print(f"Final speed: {climb_result['V']:.2f} m/s")
    print(f"Final altitude: {climb_result['z']:.1f} m")
    print(f"Time: {climb_result['time']:.2f} s")
    
    climb_data = fd.get_history()
    
    # Check if speed increased
    speed_at_start = climb_data['V'][0]
    speed_at_end = climb_data['V'][-1]
    
    if speed_at_end > speed_at_start:
        print(f"✓ SUCCESS: Speed increased from {speed_at_start:.2f} to {speed_at_end:.2f} m/s")
    else:
        print(f"✗ FAILED: Speed decreased from {speed_at_start:.2f} to {speed_at_end:.2f} m/s")
    
    return takeoff_data, climb_data


def plot_performance(aircraft_data):
    """Plot takeoff and climb performance for all aircraft."""
    
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Takeoff and Climb Performance Analysis', fontsize=16, fontweight='bold')
    
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728']
    
    # Plot 1: Takeoff - Speed vs Time
    ax1 = axes[0, 0]
    for i, (name, data) in enumerate(aircraft_data.items()):
        takeoff = data['takeoff']
        ax1.plot(takeoff['t'], takeoff['V'], label=name, color=colors[i], linewidth=2)
    
    ax1.axhline(y=70, color='red', linestyle='--', linewidth=1.5, label='Vr = 70 m/s', alpha=0.7)
    ax1.set_xlabel('Time (s)', fontsize=11)
    ax1.set_ylabel('Speed (m/s)', fontsize=11)
    ax1.set_title('Takeoff: Speed vs Time', fontsize=12, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend(fontsize=9)
    
    # Plot 2: Takeoff - Speed vs Distance
    ax2 = axes[0, 1]
    for i, (name, data) in enumerate(aircraft_data.items()):
        takeoff = data['takeoff']
        ax2.plot(takeoff['x'], takeoff['V'], label=name, color=colors[i], linewidth=2)
    
    ax2.axhline(y=70, color='red', linestyle='--', linewidth=1.5, label='Vr = 70 m/s', alpha=0.7)
    ax2.axvline(x=1508, color='orange', linestyle='--', linewidth=1.5, label='Runway end', alpha=0.7)
    ax2.set_xlabel('Distance (m)', fontsize=11)
    ax2.set_ylabel('Speed (m/s)', fontsize=11)
    ax2.set_title('Takeoff: Speed vs Distance', fontsize=12, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.legend(fontsize=9)
    
    # Plot 3: Climb - Speed vs Time
    ax3 = axes[1, 0]
    for i, (name, data) in enumerate(aircraft_data.items()):
        climb = data['climb']
        ax3.plot(climb['t'], climb['V'], label=name, color=colors[i], linewidth=2)
    
    ax3.axhline(y=85, color='green', linestyle='--', linewidth=1.5, label='Target = 85 m/s', alpha=0.7)
    ax3.set_xlabel('Time (s)', fontsize=11)
    ax3.set_ylabel('Speed (m/s)', fontsize=11)
    ax3.set_title('Climb: Speed vs Time', fontsize=12, fontweight='bold')
    ax3.grid(True, alpha=0.3)
    ax3.legend(fontsize=9)
    
    # Plot 4: Climb - Altitude vs Distance
    ax4 = axes[1, 1]
    for i, (name, data) in enumerate(aircraft_data.items()):
        climb = data['climb']
        distance = np.sqrt(climb['x']**2 + climb['y']**2)
        ax4.plot(distance, climb['z'], label=name, color=colors[i], linewidth=2)
    
    ax4.axhline(y=500, color='green', linestyle='--', linewidth=1.5, label='Target = 500 m', alpha=0.7)
    ax4.set_xlabel('Horizontal Distance (m)', fontsize=11)
    ax4.set_ylabel('Altitude (m)', fontsize=11)
    ax4.set_title('Climb: Altitude Profile', fontsize=12, fontweight='bold')
    ax4.grid(True, alpha=0.3)
    ax4.legend(fontsize=9)
    
    plt.tight_layout()
    return fig


def main():
    """Main test function."""
    print("\n" + "="*60)
    print("AIRCRAFT PERFORMANCE VALIDATION")
    print("Testing takeoff and climb for all aircraft types")
    print("="*60)
    
    # Create aircraft instances
    aircraft = {
        'AirbusA220': AirbusA220(),
        'EmbraerE170': EmbraerE170(),
        'Dash8_400': Dash8_400(),
        'ATR72_600': ATR72_600()
    }
    
    # Test each aircraft
    aircraft_data = {}
    for name, plane in aircraft.items():
        takeoff_data, climb_data = test_aircraft_takeoff_climb(plane)
        aircraft_data[name] = {
            'takeoff': takeoff_data,
            'climb': climb_data
        }
    
    # Plot results
    print(f"\n{'='*60}")
    print("Generating performance plots...")
    print("="*60)
    
    fig = plot_performance(aircraft_data)
    plt.show()
    
    print("\nTest complete!")


if __name__ == '__main__':
    main()
