"""
Simulation Module

Main simulation loop for London City Airport operations.
Handles event scheduling, aircraft spawning, and simulation orchestration.
"""

import random
from aircraft import Plane, AirbusA220, EmbraerE170, Dash8_400, ATR72_600
from ground_operations import GroundOperations
from flight_dynamics import FlightDynamics


def test_ground_operations():
    """
    Simple test of ground operations system.
    Demonstrates runway, gate, and taxi functionality.
    """
    print("=" * 60)
    print("TESTING GROUND OPERATIONS")
    print("=" * 60)
    
    # Create airport ground system (London City Airport configuration)
    ops = GroundOperations(runway_length=1508, num_gates=19)
    
    # Create test planes
    plane_a = EmbraerE170("BA001")
    plane_b = AirbusA220("BA002")
    
    # Schedule landing for plane A
    ops.request_landing_slot(plane_a)
    
    # Simulation loop
    current_time = 0.0
    timestep = 5.0  # 5 second increments
    
    print(f"\nSimulating {20 * timestep} seconds of airport operations...\n")
    
    for step in range(20):
        print(f"\n--- TIME = {current_time:.1f}s ---")
        ops.update(current_time)
        
        # Print debug state
        print(f"Runway busy: {ops.runway_busy}")
        print(f"Landing queue: {[p.id for p in ops.landing_queue]}")
        print(f"Takeoff queue: {[p.id for p in ops.takeoff_queue]}")
        print(f"Taxiing planes: {len(ops.taxiing_planes)}")
        
        # Show occupied gates
        occupied = sum(1 for g in ops.gates if g is not None)
        print(f"Gates occupied: {occupied}/{len(ops.gates)}")
        
        current_time += timestep
    
    print("\n" + "=" * 60)
    print("Ground operations test complete!")
    print("=" * 60)


def test_flight_dynamics():
    """
    Test flight dynamics for a single aircraft.
    Simulates takeoff, climb, and landing phases.
    """
    print("\n" + "=" * 60)
    print("TESTING FLIGHT DYNAMICS")
    print("=" * 60)
    
    # Create an aircraft
    plane = EmbraerE170("TEST001")
    dynamics = FlightDynamics(plane)
    
    print(f"\nAircraft: {plane.plane_type}")
    print(f"Mass: {plane.mass:.0f} kg")
    print(f"Wing area: {plane.wing_area:.2f} m²")
    print(f"Wingspan: {plane.wingspan:.2f} m")
    print(f"Stall speed: {plane.stall_speed():.2f} m/s")
    
    # Test takeoff
    print("\n--- TAKEOFF PHASE ---")
    takeoff_result = dynamics.ground_roll_takeoff(dt=0.1, Vr=70.0)
    print(f"Rotation speed: {takeoff_result['V_rot']:.2f} m/s")
    print(f"Takeoff distance: {takeoff_result['distance']:.2f} m")
    print(f"Takeoff time: {takeoff_result['time']:.2f} s")
    print(f"Success: {takeoff_result['success']}")
    
    # Test climb
    print("\n--- CLIMB PHASE ---")
    dynamics.V = 80.0  # initial climb speed
    climb_result = dynamics.climb_phase(t_end=60.0, dt=0.5, target_altitude=1000.0, gamma=0.1)
    print(f"Final altitude: {abs(climb_result['z']):.2f} m")
    print(f"Final speed: {climb_result['V']:.2f} m/s")
    print(f"Horizontal distance: {climb_result['x']:.2f} m")
    print(f"Climb time: {climb_result['time']:.2f} s")
    
    # Test landing
    print("\n--- LANDING PHASE ---")
    dynamics.V = 70.0  # approach speed
    landing_result = dynamics.ground_roll_landing(dt=0.1, V_touchdown=65.0, 
                                                    T_reverse=50000.0)
    print(f"Stopping distance: {landing_result['x_stop']:.2f} m")
    print(f"Stopping time: {landing_result['t_stop']:.2f} s")
    print(f"Success: {landing_result['stopped']}")
    
    print("\n" + "=" * 60)
    print("Flight dynamics test complete!")
    print("=" * 60)


def test_aircraft_types():
    """
    Display specifications for all aircraft types.
    """
    print("\n" + "=" * 60)
    print("AIRCRAFT SPECIFICATIONS")
    print("=" * 60)
    
    aircraft_types = [
        AirbusA220("A220-TEST"),
        EmbraerE170("E170-TEST"),
        Dash8_400("D8-TEST"),
        ATR72_600("ATR-TEST")
    ]
    
    for plane in aircraft_types:
        print(f"\n{plane.plane_type}:")
        print(f"  Mass: {plane.mass:.0f} kg")
        print(f"  Wing area: {plane.wing_area:.2f} m²")
        print(f"  Wingspan: {plane.wingspan:.2f} m")
        print(f"  Aspect ratio: {plane.aspect_ratio():.2f}")
        print(f"  CL_max: {plane.CL_max:.2f}")
        print(f"  CD0: {plane.CD0:.4f}")
        print(f"  Oswald efficiency: {plane.e:.2f}")
        print(f"  Stall speed: {plane.stall_speed():.2f} m/s")
    
    print("\n" + "=" * 60)


def run_weekly_simulation():
    """
    Run a week-long simulation of London City Airport operations.
    This is a placeholder for the full simulation implementation.
    """
    print("\n" + "=" * 60)
    print("WEEKLY SIMULATION - PLACEHOLDER")
    print("=" * 60)
    
    print("\nThis will implement the full simulation as per pseudocode:")
    print("- 7-day simulation period")
    print("- Poisson arrival process (~122 arrivals/day)")
    print("- Service time modeling")
    print("- Queue management")
    print("- Statistics collection")
    print("\n(To be implemented)")
    
    print("\n" + "=" * 60)


if __name__ == '__main__':
    """
    Main entry point for simulation tests.
    Uncomment the test functions you want to run.
    """
    
    # Run all tests
    test_aircraft_types()
    test_ground_operations()
    test_flight_dynamics()
    
    # Placeholder for full simulation
    # run_weekly_simulation()
