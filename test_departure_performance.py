#!/usr/bin/env python3
"""
Test script to evaluate departure performance for each aircraft type.
Checks final speed at airspace boundary for all aircraft.
"""

import random
from departure_routes import generate_departure_routes
from aircraft import EmbraerE170, AirbusA220, Dash8_400, ATR72_600
from departure_live_sim import DepartureSimulator

def test_aircraft_departure(plane_class, route):
    """Test a single aircraft departure and return performance metrics."""
    plane_id = f"{plane_class.__name__}-TEST"
    plane = plane_class(plane_id)
    
    print(f"\n{'='*60}")
    print(f"Testing: {plane.plane_type}")
    print(f"{'='*60}")
    print(f"Mass: {plane.mass:.0f} kg")
    print(f"Static Thrust: {plane._T0:.0f} N")
    print(f"Thrust/Weight: {plane._T0 / (plane.mass * 9.81):.3f}")
    print(f"Ct (thrust decay): {plane._Ct}")
    
    # Create simulator
    simulator = DepartureSimulator(route, plane)
    
    # Run simulation
    try:
        for pos in simulator:
            pass  # Just iterate through
    except StopIteration:
        pass
    
    # Get final metrics
    final_speed = simulator.speed_hist[-1]
    final_altitude = simulator.altitude_hist[-1]
    total_time = simulator.time_hist[-1]
    
    # Speed statistics
    speeds = simulator.speed_hist
    min_speed = min(speeds)
    max_speed = max(speeds)
    avg_speed = sum(speeds) / len(speeds)
    
    print(f"\n--- PERFORMANCE RESULTS ---")
    print(f"Takeoff distance: {simulator.ground_roll_data['x'][-1]:.1f} m")
    print(f"Rotation speed: {simulator.ground_roll_data['V'][-1]:.1f} m/s")
    print(f"Speed range: {min_speed:.1f} - {max_speed:.1f} m/s")
    print(f"Average speed: {avg_speed:.1f} m/s")
    print(f"Final speed: {final_speed:.1f} m/s")
    print(f"Final altitude: {final_altitude:.1f} m")
    print(f"Total time: {total_time:.1f} s")
    
    # Check if meets 105 m/s target
    target_speed = 105.0
    if final_speed >= target_speed:
        print(f"✓ PASS: Final speed {final_speed:.1f} m/s >= {target_speed} m/s")
        status = "PASS"
    else:
        deficit = target_speed - final_speed
        print(f"✗ FAIL: Final speed {final_speed:.1f} m/s < {target_speed} m/s (deficit: {deficit:.1f} m/s)")
        status = "FAIL"
    
    return {
        'aircraft': plane.plane_type,
        'mass': plane.mass,
        'T0': plane._T0,
        'Ct': plane._Ct,
        'TW_ratio': plane._T0 / (plane.mass * 9.81),
        'takeoff_dist': simulator.ground_roll_data['x'][-1],
        'final_speed': final_speed,
        'final_altitude': final_altitude,
        'total_time': total_time,
        'status': status
    }


def main():
    print("\n" + "="*60)
    print("DEPARTURE PERFORMANCE EVALUATION")
    print("Testing all aircraft types to 105 m/s target")
    print("="*60)
    
    # Generate a test route (use straight ahead for consistency)
    routes = generate_departure_routes()
    test_route = routes[0]  # DEP01 - straight ahead
    
    print(f"\nUsing route: {test_route['name']} (heading {test_route['heading']}°)")
    
    # Test each aircraft type
    aircraft_classes = [
        AirbusA220,
        EmbraerE170,
        Dash8_400,
        ATR72_600
    ]
    
    results = []
    for aircraft_class in aircraft_classes:
        result = test_aircraft_departure(aircraft_class, test_route)
        results.append(result)
    
    # Summary table
    print("\n" + "="*60)
    print("SUMMARY TABLE")
    print("="*60)
    print(f"{'Aircraft':<15} {'T/W':>6} {'T0 (kN)':>9} {'Final V':>9} {'Status':>8}")
    print("-"*60)
    for r in results:
        print(f"{r['aircraft']:<15} {r['TW_ratio']:>6.3f} {r['T0']/1000:>9.1f} "
              f"{r['final_speed']:>9.1f} {r['status']:>8}")
    
    # Analysis
    print("\n" + "="*60)
    print("ANALYSIS & RECOMMENDATIONS")
    print("="*60)
    
    failures = [r for r in results if r['status'] == 'FAIL']
    if failures:
        print(f"\n{len(failures)} aircraft failed to reach 105 m/s:")
        for r in failures:
            deficit = 105.0 - r['final_speed']
            # Calculate required thrust increase
            # Rough estimate: dV = (T - D) / m * dt
            # For 10 m/s increase over ~60s climb, need ~0.17 m/s² more acceleration
            required_accel = deficit / 60.0  # m/s²
            additional_thrust = required_accel * r['mass']
            thrust_increase_pct = (additional_thrust / r['T0']) * 100
            
            print(f"\n{r['aircraft']}:")
            print(f"  Current T0: {r['T0']/1000:.1f} kN")
            print(f"  Speed deficit: {deficit:.1f} m/s")
            print(f"  Estimated additional thrust needed: {additional_thrust/1000:.1f} kN ({thrust_increase_pct:.1f}%)")
            print(f"  Recommended T0: {(r['T0'] + additional_thrust)/1000:.1f} kN")
    else:
        print("\n✓ All aircraft meet the 105 m/s target!")


if __name__ == '__main__':
    main()
