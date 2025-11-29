# London City Airport Simulation - Project Structure

## Overview
This project simulates aircraft operations at London City Airport (LCY), including flight dynamics, ground operations, runway management, and gate allocation.

## File Structure

### Core Modules

#### `aircraft.py`
Defines aircraft types and their physical properties.
- **`Plane`** (parent class): Base aircraft with aerodynamic calculations
- **`AirbusA220`**: Regional jet (70,900 kg)
- **`EmbraerE170`**: Regional jet (37,200 kg)
- **`Dash8_400`**: Turboprop (29,574 kg)
- **`ATR72_600`**: Turboprop (22,800 kg)

**Key Features:**
- Aerodynamic force calculations (lift, drag, thrust)
- Rolling friction and braking models
- Stall speed computation
- Protected attributes with getters/setters

#### `flight_dynamics.py`
Implements physics-based flight simulation using ODEs.

**Flight Phases:**
- `ground_roll_takeoff()`: Runway acceleration to rotation speed
- `climb_phase()`: Ascent with 3D kinematics
- `holding_phase()`: Constant altitude/speed flight
- `final_approach()`: Descent to runway (5.5° for LCY)
- `ground_roll_landing()`: Deceleration with braking and thrust reversal

**Key Features:**
- Numerical integration (Euler method)
- Telemetry recording for analysis
- Configurable parameters (thrust, runway slope, etc.)

#### `ground_operations.py`
Manages all ground-based airport operations.

**Components:**
- **Runway Management**: Scheduling, busy status, priority queuing
- **Gate Management**: 19 gates, allocation/release
- **Taxi Operations**: Event-based taxi timing with Gaussian distribution
- **Queue System**: Separate landing and takeoff queues with priority logic

**Key Features:**
- Landing priority over takeoffs
- Gate availability checking before landing (LCY constraint)
- Realistic taxi times (75s gate→runway, 90s runway→gate)

#### `simulation.py`
Main simulation orchestration and testing framework.

**Test Functions:**
- `test_aircraft_types()`: Display all aircraft specifications
- `test_ground_operations()`: Demonstrate runway/gate/taxi functionality
- `test_flight_dynamics()`: Test all flight phases
- `run_weekly_simulation()`: Placeholder for full 7-day simulation

#### `main.py`
Entry point that imports all modules and runs tests.

### Backup Files
- `main_backup.py`: Original monolithic version (for reference)

## Running the Simulation

```bash
python main.py
```

This will run all test modules and display:
- Aircraft specifications for all types
- Ground operations simulation (100s)
- Flight dynamics demonstrations (takeoff, climb, landing)

## Next Steps

The modular structure is now ready for implementing the full weekly simulation:

1. **Event Scheduling**: Implement Poisson arrival process (~122 arrivals/day)
2. **State Machine**: Track aircraft through states (approaching, landing, at_gate, departing)
3. **Statistics Collection**: Queue lengths, runway utilization, delays, go-arounds
4. **Integration**: Connect FlightDynamics with GroundOperations
5. **Validation**: Compare with real LCY operational data

## Key Design Decisions

- **Separation of Concerns**: Each module has a single responsibility
- **Reusability**: Classes can be used independently for testing
- **Extensibility**: Easy to add new aircraft types or operations
- **Maintainability**: Clear structure makes debugging easier
- **Physics-Based**: Uses real aerodynamic equations and airport constraints

## Dependencies

```
numpy
math
random
```

## London City Airport Constraints

- **Runway**: 1,508m long with 5.5° approach slope
- **Gates**: 19 aircraft stands
- **Operations**: ~122 movements per day
- **Aircraft Mix**: Regional jets and turboprops only
- **Steep Approach**: Requires special pilot certification

## Notes

- The physics models use SI units (kg, m, m/s, N)
- Telemetry can be exported for further analysis
- Some parameters (thrust coefficients) may need tuning based on real data
- Current implementation uses simplified thrust model (will be refined)
