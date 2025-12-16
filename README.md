# London City Airport Simulation

## Overview

This project provides a comprehensive physics-based simulation of aircraft operations at London City Airport (LCY), one of London's most operationally challenging airports due to its steep 5.5° approach angle and short 1,508-metre runway.

The simulation models:
- **Flight dynamics** using ordinary differential equations (ODEs)
- **Arrival and departure routes** (STARs and SIDs)
- **Ground operations** (runway management, gate allocation, taxi operations)
- **Complete turnaround cycles** (arrival → gate → departure)
- **Emergency scenarios** with priority landing procedures

## Table of Contents

1. [Project Structure](#project-structure)
2. [Core Modules](#core-modules)
3. [Aircraft Fleet](#aircraft-fleet)
4. [Flight Phases](#flight-phases)
5. [Running Simulations](#running-simulations)
6. [Dependencies](#dependencies)
7. [London City Airport Specifications](#london-city-airport-specifications)
8. [Extending the Simulation](#extending-the-simulation)
9. [Physical Models](#physical-models)
10. [Future Enhancements](#future-enhancements)

---

## Project Structure

```
MS_Coursework/
├── aircraft.py                      # Aircraft classes and aerodynamic properties
├── flight_dynamics.py               # ODE-based flight physics engine
├── ground_operations.py             # Runway, gate, and taxi management
├── arrival_routes.py                # STAR generation (20 arrival routes)
├── departure_routes.py              # SID generation (5 departure routes)
├── arrival_live_sim.py              # Live arrival simulation with visualisation
├── departure_live_sim.py            # Live departure simulation with visualisation
├── main.py                          # Complete turnaround cycle orchestration and main entry point
├── emergency_landing_sim.py         # Priority landing and holding patterns
├── intersecting_runways_preview.py  # Multi-runway concept visualisation
└── README.md                        # This file
```

---

## Core Modules

### 1. `aircraft.py` - Aircraft Definitions

Defines the `Plane` base class and four specific regional aircraft types commonly operating at LCY.

**Aircraft Types:**
- `AirbusA220`: 70,900 kg, 100-130 passengers
- `EmbraerE170`: 37,200 kg, 70-80 passengers
- `Dash8_400`: 29,574 kg, 70-78 passengers (turboprop)
- `ATR72_600`: 22,800 kg, 68-74 passengers (turboprop)

**Key Features:**
- Aerodynamic force calculations (lift, drag, thrust)
- Rolling friction and braking models
- Stall speed computation
- Protected attributes with validation

---

### 2. `flight_dynamics.py` - Physics Engine

Implements numerical integration of flight equations using the Euler method.

**Flight Phases:**
1. Ground roll (takeoff)
2. Climb
3. Cruise/Holding
4. Approach (5.5° for LCY)
5. Landing rollout

**Coordinate System:**
- x: East (runway direction)
- y: North (lateral)
- z: Up (altitude, positive upwards)

---

### 3. `ground_operations.py` - Airport Operations

Manages runway scheduling, gate allocation, and taxi operations.

**Features:**
- 19 gates with availability tracking
- Priority system (arrivals before departures)
- Realistic taxi times (75s gate→runway, 90s runway→gate)
- Queue management
- Conflict detection

---

### 4. Simulation Modules

**`arrival_live_sim.py`**: Real-time 3D arrival animation  
**`departure_live_sim.py`**: Real-time 3D departure animation  
**`main.py`**: Full turnaround cycles and main entry point  
**`emergency_landing_sim.py`**: Priority and holding scenarios

---

## Aircraft Fleet

### Comparison Table

| Aircraft      | Mass (kg) | Wingspan (m) | Engines       | Passengers | Type      |
|---------------|-----------|--------------|---------------|------------|-----------|
| Airbus A220   | 70,900    | 35.1         | PW1500G (×2)  | 100-130    | Jet       |
| Embraer E170  | 37,200    | 28.65        | CF34-8E (×2)  | 70-80      | Jet       |
| Dash 8-400    | 29,574    | 28.42        | PW150A (×2)   | 70-78      | Turboprop |
| ATR 72-600    | 22,800    | 27.05        | PW127M (×2)   | 68-74      | Turboprop |

---

## Flight Phases

### Takeoff
- Ground roll until rotation speed (~70 m/s)
- Typical distance: 800-1,200 m
- Forces: Thrust, drag, rolling friction

### Climb
- 10° climb angle
- Target speed: 85 m/s
- 3D kinematics with bank angles for turns

### Approach
- 5.5° glide slope (steeper than standard 3°)
- Progressive speed reduction
- Final approach: 65-75 m/s

### Landing
- Touchdown at ~65 m/s
- Braking + thrust reversers
- Stopping distance: 600-1,100 m

---

## Running Simulations

### Quick Start

```bash
# Watch an arrival
python arrival_live_sim.py

# Watch a departure
python departure_live_sim.py

# Full operations (multiple aircraft)
python main.py --max-aircraft 4 --arrivals 8

# Emergency scenario
python emergency_landing_sim.py
```

---

## Dependencies

```bash
pip install numpy matplotlib
```

**Requirements:**
- Python 3.8+
- NumPy ≥ 1.20.0
- Matplotlib ≥ 3.3.0

---

## London City Airport Specifications

### Runway
- **Designation**: 09/27
- **Length**: 1,508 metres
- **Approach angle**: 5.5° (requires special certification)

### Operations
- **Daily movements**: ~122
- **Gates**: 19
- **Annual passengers**: ~5 million

### Airspace
- **Control zone**: 5.5 km radius, surface to 500 m
- **20 arrival routes** (STARs)
- **5 departure routes** (SIDs)

---

## Extending the Simulation

### Adding a New Aircraft

```python
# In aircraft.py
class Boeing737_700(Plane):
    def __init__(self, identifier: str = 'B737'):
        super().__init__(
            identifier=identifier,
            mass=70080.0,
            S=124.6,
            b=34.3,
            CL_max=1.7,
            CD0=0.022,
            e=0.8
        )
        self.plane_type = 'B737_700'
        self._T0 = 242000.0
        self._Ct = 0.0002
```

### Adding Weather Effects

```python
# Enhanced wind model
weather = WeatherSystem()
wind_model = WindModel(
    min_speed=0,
    max_speed=20,        # Stronger winds
    gust_sigma=2.0       # More turbulence
)
```

### Performance Metrics

```python
class PerformanceMetrics:
    def record_flight(self, aircraft, telemetry):
        # Calculate takeoff distance
        # Calculate landing distance
        # Estimate fuel consumption
        # Track flight times
```

### Multi-Day Scheduling

```python
class AirportSchedule:
    def generate_week(self):
        # Generate 7-day flight schedule
        # Peak hours: 06:00-09:00, 17:00-20:00
        # Off-peak: reduced frequency
```

### Noise Modelling

```python
class NoiseModel:
    def compute_noise_level(self, aircraft, altitude, distance):
        # LCY limit: 94 dB
        # Model: L = L₀ - 20·log₁₀(distance) - altitude_factor
```

---

## Physical Models

### Aerodynamics

**Lift:**
```
L = ½ρV²S·CL
CL = 2mg·cos(γ)·cos(φ) / (ρV²S)
```

**Drag:**
```
D = ½ρV²S·CD
CD = CD₀ + CL²/(πARe)
```

**Thrust:**
```
T(V) = T₀(1 - Ct·V)
```

### Ground Dynamics

**Takeoff:**
```
m·dV/dt = T - D - μrr(mg - L)
```

**Landing:**
```
m·dV/dt = -D - μrr(mg - L) - β·μb·mg - Treverse
```

Where:
- μrr: Rolling resistance (~0.02)
- μb: Braking coefficient (0.3-0.4 dry)
- β: Brake application (0-1)

---

## Future Enhancements

### Short-Term
1. Wind shear and microbursts
2. Enhanced cockpit visualisation
3. Performance optimisation

### Medium-Term
4. 6-DOF flight dynamics
5. AI-based ATC decisions
6. Economic model (fuel, fees, delays)

### Long-Term
7. Machine learning for optimal approaches
8. Multi-airport network simulation
9. Historical weather data integration
10. Passenger flow simulation

---

## Troubleshooting

**Aircraft overshoots runway:**
- Increase `T_reverse=80000` in landing simulation
- Reduce `FINAL_SPEED` for approach

**Takeoff distance too long:**
- Verify aircraft mass matches MTOW
- Increase thrust `_T0` value
- Check rolling friction coefficient

**Animation runs slowly:**
- Increase timestep `DT = 0.5`
- Reduce `TRAIL_LENGTH = 100`
- Disable telemetry windows

---

## References

- Anderson, J. D. (2016). *Fundamentals of Aerodynamics*
- Stevens, B. L., & Lewis, F. L. (2003). *Aircraft Control and Simulation*
- London City Airport Noise Action Plan 2024-2029
- CAA UK AIP - EGLC London City Airport

---

## Licence

**Author:** Aryan Patel & Eesha Mashruwala
**Institution:** University College London  
**Course:** MEng Robotics & Artificial Intelligence - Modelling and Simulation  
**Date:** December 2025

---

*Last updated: 16 December 2025*
