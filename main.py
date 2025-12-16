"""
London City Airport Simulation - Main Entry Point

This module serves as the main entry point for the airport simulation.
All classes have been modularised into separate files:

- aircraft.py: Plane classes and aircraft specifications
- flight_dynamics.py: Flight physics and dynamics (ODEs)
- ground_operations.py: Airport ground operations (runway, gates, taxi)
- arrival_routes.py: Standard Terminal Arrival Routes (STARs)
- departure_routes.py: Standard Instrument Departures (SIDs)
- arrival_live_sim.py: Live arrival flight simulation with visualisation
- departure_live_sim.py: Live departure flight simulation with visualisation
- autonomous_turnaround_sim.py: Complete turnaround cycle simulation
- emergency_landing_sim.py: Emergency priority landing scenarios

Run different simulation modules directly or use this file for basic tests.

London City Airport (IATA: LCY, ICAO: EGLC):
- Single runway 09/27: 1,508 metres
- Steep approach angle: 5.5\u00b0 (vs standard 3\u00b0)
- 19 aircraft stands
- ~122 daily movements (arrivals + departures)
- Serves regional jets and turboprops only
"""

import numpy as np
import math
import random

from aircraft import Plane, AirbusA220, EmbraerE170, Dash8_400, ATR72_600
from flight_dynamics import FlightDynamics
from ground_operations import GroundOperations
from simulation import (test_aircraft_types, test_ground_operations, 
                       test_flight_dynamics, run_weekly_simulation)


if __name__ == '__main__':
    """
    Main entry point for the London City Airport simulation.
    
    Run different test modules:
    - Aircraft specifications
    - Ground operations
    - Flight dynamics
    - Full weekly simulation (to be implemented)
    """
    
    print("=" * 70)
    print(" LONDON CITY AIRPORT SIMULATION")
    print("=" * 70)
    print("\nModular Architecture:")
    print("  - aircraft.py: Aircraft classes and specifications")
    print("  - flight_dynamics.py: Flight physics and ODEs")
    print("  - ground_operations.py: Airport ground operations")
    print("  - arrival_routes.py: STAR (arrival) route generation")
    print("  - departure_routes.py: SID (departure) route generation")
    print("  - arrival_live_sim.py: Live arrival simulations")
    print("  - departure_live_sim.py: Live departure simulations")
    print("  - autonomous_turnaround_sim.py: Full turnaround cycles")
    print("  - emergency_landing_sim.py: Emergency scenarios")
    print("=" * 70)
    
    # Run all tests
    test_aircraft_types(run various simulation scenarios!")
    print("\nAvailable Simulations:")
    print("  • python arrival_live_sim.py - Watch aircraft land")
    print("  • python departure_live_sim.py - Watch aircraft take off")
    print("  • python autonomous_turnaround_sim.py - Full operational cycle")
    print("  • python emergency_landing_sim.py - Priority landing scenarios")
    print("  • python intersecting_runways_preview.py - Multi-runway concept")
    print("\nSimulation Features:")
    print("  1. Initialise airport (runway, 19 gates)")
    print("  2. Generate arrivals using Poisson process (~122/day)")
    print("  3. Realistic flight dynamics with wind effects")
    print("  4. Track runway usage, gate occupancy, queues")
    print("  5. Handle priority clearances and holding patterns")
    print("  6. Collect telemetry and statist 19 gates)")
    print("  2. Generate arrivals using Poisson process (~122/day)")
    print("  3. Simulate 7 days of operations")
    print("  4. Track runway usage, gate occupancy, queues")
    print("  5. Handle go-arounds, taxi operations")
    print("  6. Collect statistics and metrics")
    
    print("\n" + "=" * 70)
    print("Simulation setup complete. Ready for next phase!")
    print("=" * 70)
