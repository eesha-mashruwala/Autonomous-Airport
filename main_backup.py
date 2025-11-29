"""
London City Airport Simulation - Main Entry Point

This module serves as the main entry point for the airport simulation.
All classes have been modularized into separate files:

- aircraft.py: Plane classes and aircraft specifications
- flight_dynamics.py: Flight physics and dynamics (ODEs)
- ground_operations.py: Airport ground operations (runway, gates, taxi)
- simulation.py: Simulation orchestration and testing

Run this file to execute the simulation tests.
"""

import numpy as np
import math
import random

from aircraft import Plane, AirbusA220, EmbraerE170, Dash8_400, ATR72_600
from flight_dynamics import FlightDynamics
from ground_operations import GroundOperations
from simulation import (test_aircraft_types, test_ground_operations, 
                       test_flight_dynamics, run_weekly_simulation)



# All classes have been moved to separate modules
# See imports at the top of this file


if __name__ == '__main__':
    """
    Parent class 
    """

    def __init__(self, identifier: str, mass: float, S: float, b: float,
    CL_max: float = 1.5, CD0: float = 0.02, e: float = 0.8,
    rho: float = 1.225):
    
        """
        Initialize a Plane instance.


        Parameters
        ----------
        identifier : str
        Unique plane ID.
        mass : float
        Mass (kg).
        S : float
        Wing area (m^2).
        b : float
        Wingspan (m).
        CL_max : float
        Maximum lift coefficient.
        CD0 : float
        Zero-lift drag coefficient.
        e : float
        Oswald efficiency factor (dimensionless).
        rho : float
        Air density (kg/m^3).
        """
        
        # protected attributes
        self._mass = float(mass)
        self._S = float(S)
        self._b = float(b)
        self._CL_max = float(CL_max)
        self._CD0 = float(CD0)
        self._e = float(e)
        self._rho = float(rho)


        # private identifier
        self.__id = str(identifier)


        # public
        self.gate: Optional[str] = None
        self.plane_type: str = "GenericPlane"


    # ------------------ getters and setters ---------------------------------
    @property
    def id(self):
        """Return the private plane identifier."""
        return self.__id

    @property
    def mass(self):
        """Get aircraft mass (kg)."""
        return self._mass

    @mass.setter
    def mass(self, value: float):
        """Set mass with validation."""
        if value <= 0:
            raise ValueError("Mass must be positive")
        self._mass = float(value)

    @property
    def wing_area(self):
        """Get wing area S (m^2)."""
        return self._S

    @wing_area.setter
    def wing_area(self, value: float):
        """Set wing area with validation."""
        if value <= 0:
            raise ValueError("Wing area must be positive")
        self._S = float(value)

    @property
    def wingspan(self):
        """Get wingspan b (m)."""
        return self._b

    @property
    def CL_max(self):
        """Get CL_max."""
        return self._CL_max

    @property
    def CD0(self):
        """Get zero-lift drag coefficient."""
        return self._CD0

    @property
    def e(self):
        """Get Oswald efficiency factor."""
        return self._e

    @property
    def rho(self):
        """Get ambient air density used by the plane."""
        return self._rho

    # ------------------ AERODYNAMIC HELPERS ---------------------------------
    # ------------------------ Aspect Ratio ----------------------------------
    def aspect_ratio(self):
            """Compute wing aspect ratio AR = b^2 / S."""
            return (self._b ** 2) / self._S

    # ------------------------ Thrust model ----------------------------------
    def compute_thrust(self, V):
        """
        T(V)= T_0 * (1- C_t*V)
        Thrust model used in:
        -- ground_roll_takeoff()
        -- climb_phase()
        -- holding_phase()
        -- final_approach()
        This thrust model is used in EVERY stage of flight EXCEPT ground_roll_landing()

        """
        return max(0.0, self._T0 * (1.0 - self._Ct * V))

    # ------------------------ Aerodynamic Drag model ----------------------------------
    def compute_drag(self, V, gamma=0.0, phi=0.0):
        """
        D = 1/2*ρ*V^2*S*C_D
        Aerodynamic Drag used in:
        -- ground_roll_takeoff()
        -- climb_phase()
        -- holding_phase()
        -- final_approach()
        -- ground_roll_landing()
        This model is used in EVERY stage of flight

        """
        CD = self.compute_drag_coeff(V, gamma, phi)
        return 0.5 * self._rho * V**2 * self._S * CD


    def compute_drag_coeff(self, V, gamma=0.0, phi=0.0):
        """
        C_D = C_D0+(S*C_L^2)/(π*b^2*e)
        Aerodynamic Drag Coefficient(C_D) used in the Aerodynamic Drag model from compute_drag():
        To see which functions it is used see : compute_drag()

        """
        CL = self.compute_lift(V, gamma, phi)
        induced = (CL**2) / (math.pi * self._AR * self._e)
        return self._CD0 + induced


    def compute_lift(self, V, gamma=0.0, phi=0.0):
        """
        C_L^2 = (4m^2*g^2*(cos⁡)^2*γ)/(ρ^2*V^4*S^2*(cos⁡)^2*ϕ) - note we calc and return C_L here
        Lift (C_L) used in the Aerodynamic Drag Coefficient model from compute_drag_coeff():
        To see which functions it is used see : compute_drag_coeff()

        """
        num = 2 * self._mass * 9.81 * math.cos(gamma) * math.cos(phi)
        den = self._rho * V**2 * self._S
        return num / den

    # ------------------------ Rolling Friction -------------------------
    def compute_rolling_friction(self, V):
        """
        F_rr = μ_rr*(mg - F_L)
        Aerodynamic Drag used in:
        -- ground_roll_takeoff()
        -- ground_roll_landing()

        """
        L = self.compute_aerodynamic_lift(V)
        return self._mu_rr * (self._mass * 9.81 - L)

    # ------------------------ Aerodynamic lift ----------------------------------
    def compute_aerodynamic_lift(self, V, gamma=0.0, phi=0.0):
        """
        F_L= 1/2*ρ*V^2*S*C_L
        Aerodynamic Lift (F_L) used in the Rolling Friction model from compute_rolling_friction():
        To see which functions it is used see : compute_rolling_friction()

        """
        CL = self.compute_lift(V, gamma, phi)
        return 0.5 * self._rho * V**2 * self._S * CL

    # ------------------------ Braking Force ----------------------------------
    def compute_braking_force():
        """
        F_brake (t)= β * μ_b * mg
        Braking Force(F_brake) used in the Ground Roll stage of flight - see ground_roll_landing()

        """

        beta = 1.0  # full brakes
        # fill in eqn
        return F_brake

    # ------------------------ Stall Speed ----------------------------------
    def stall_speed(self):
        """Return an estimate of stall speed V_stall (m/s) using CL_max.

        V_stall = sqrt(2*m*g / (rho * S * CL_max))
        """

        return np.sqrt((2.0 * self._mass * g) / (self._rho * self._S * self._CL_max))



# ------ PLANE SUBCLASSES --------------------------------------------------------------

class AirbusA220(Plane):
    """ Specific plane data for Airbus A220-100.
        Inheriting from Plane class         """

    def __init__(self, identifier: str = 'AirbusA220'):
        super().__init__(identifier=identifier, mass=70900.0, S=112.3, b=35.1,
                         CL_max=1.6, CD0=0.02, e=0.8)
        self.plane_type = 'AirbusA220'


class EmbraerE170(Plane):
    """ Specific plane data for Embraer E170.
        Inheriting from Plane class         """

    def __init__(self, identifier: str = 'EmbraerE170'):
        super().__init__(identifier=identifier, mass=37200.0, S=72.72, b=28.65,
                         CL_max=1.5, CD0=0.019, e=0.82)
        self.plane_type = 'EmbraerE170'


class Dash8_400(Plane):
    """ Specific plane data for Dash 8-400.
        Inheriting from Plane class         """

    def __init__(self, identifier: str = 'Dash8_400'):
        super().__init__(identifier=identifier, mass=29574.0, S=63.1, b=28.42,
                         CL_max=1.5, CD0=0.019, e=0.82)
        self.plane_type = 'Dash8_400'


class ATR72_600(Plane):
    """ Specific plane data for ATR 72-600.
        Inheriting from Plane class         """

    def __init__(self, identifier: str = 'ATR72_600'):
        super().__init__(identifier=identifier, mass=22800.0, S=61.0, b=27.05,
                         CL_max=1.5, CD0=0.019, e=0.82)
        self.plane_type = 'ATR72_600'

# ------ FLIGHT DYNAMICS CLASSES --------------------------------------------------------------

class FlightDynamics:
    """
    Flight dynamics class that implements ode's for each instance of the plane class at every time step
    This class collects telemetry and provides configurable parameters.

    """

    def __init__(self, plane: Plane):
        """
        Initialise FlightDynamics attached to a Plane.

        Parameters
        ----------
        plane : Plane
            The physical plane object containing geometry and mass.
        """
        self._plane = plane  # protected
        self._telemetry: List[Dict] = []  # stores dictionaries of telemetry

        # Default engine/thrust parameters (can be tuned)
        self._T0 = 120000.0  # static thrust total (N)
        self._Ct = 0.0       # thrust decay coefficient (per m/s)

        # Rolling friction coefficient
        self._mu_rr = 0.02

        # Runway slope angle (radians)
        self._theta = 0.0


    # -------------------- Co-orindate/Telemetry management -------------------------------
    def _record(self):
        """
        Record the current simulation state (forces, kinematics, controls)
        and store it in the telemetry buffer.
        """
        entry = {
            "time": self.t,
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "V": self.V,
            "gamma": self.gamma,
            "psi": self.psi,
            "phi": self.phi,
            "thrust": self.T,
            "drag": self.D,
            "lift": self.L,
            "rolling_friction": self.F_rr,
            "brake_force": self.F_brake,
            "throttle": self.throttle_cmd,
            "brake_cmd": self.brake_cmd,
        }

        self._telemetry.append(entry)


    def telemetry(self) -> List[Dict]:
        """
        Return collected telemetry entries as a list of dicts.
        Each entry contains forces, states, and control inputs.
        """
        return list(self._telemetry)


    # ------------------------ Velocity/ True Airspeed -------------------------
    def compute_airspeed(self, V, gamma, thrust):
        """
        m * dV/dt = T - D - mgsin⁡γ
        True Airspeed used in:
        -- climb_phase()
        -- holding_phase()
        -- final_approach()

        """
        D = self.plane.compute_drag(V, gamma)
        return (thrust - D - self.plane.mass * 9.81 * math.sin(gamma)) / self.plane.mass


    # ------------------------ Heading Rate -------------------------
    def compute_heading_rate(self, V, phi):
        """
        dψ/dt=g/V tan⁡ϕ
        Heading Rate used in:
        -- climb_phase()
        -- holding_phase()
        -- final_approach()

        """
        return 9.81 / V * math.tan(phi)

    
    # ------------------------ Ground roll (takeoff) -------------------------
    def ground_roll_takeoff(self, dt=0.1, Vr=70):
        """
        Simulate ground roll (1D along runway) using Newton's 2nd law.
        m * dV/dt = T - D - F_rr - mgsin⁡θ

        """
        # integrate until V reaches rotation speed Vr
        V = 0.0
        x = 0.0

        while V < Vr:
            T = self.plane.compute_thrust(V)
            D = self.plane.compute_drag(V)
            L = self.plane.compute_aerodynamic_lift(V)
            F_rr = self.plane.compute_rolling_friction(L)

            a = (T - D - F_rr - self.plane.m * 9.81 * math.sin(self.theta)) / self.plane.m
            
            V += a * dt
            x += V * dt

        return {"V_rot": V, "distance": x}

    # ------------------------------ Climb ----------------------------------
    def climb_phase(self, t_end=60, dt=0.1):
        """
        Simplified climb phase using 2D kinematics and dynamics.
        dx/dt = Vcos⁡γcos⁡ψ
        dy/dt = Vcos⁡γsin⁡ψ
        dz/dt = -Vsin⁡γ
        """

        x = y = z = 0
        t = 0

        while t < t_end:
            T = self.plane.compute_thrust(self.V)
            D = self.plane.compute_drag(self.V)
            
            dVdt = (T - D - self.plane.m * 9.81 * math.sin(self.gamma)) / self.plane.m
            self.V += dVdt * dt

            x += self.V * math.cos(self.gamma) * math.cos(self.psi) * dt
            y += self.V * math.cos(self.gamma) * math.sin(self.psi) * dt
            z += -self.V * math.sin(self.gamma) * dt

            t += dt

        return {"x": x, "y": y, "z": z, "V": self.V}

    # ------------------------------ Holding ---------------------------------
    def holding_phase(self):
        """
        The holding phase is where the plane spawns and flies before moving into the final_approach stage
        dx/dt=V_holding*cos⁡γ*cos⁡ψ
        dy/dt=V_holding*cos⁡γ*sin⁡ψ
        dz/dt=-V_holding*sin⁡γ

        """
        x = y = z = 0

        for _ in range(int(t_hold/dt)):
            x += self.V_hold * cos(self.gamma) * cos(self.psi) * dt
            y += self.V_hold * cos(self.gamma) * sin(self.psi) * dt
            z += -self.V_hold * sin(self.gamma) * dt

        return {"x": x, "y": y, "z": z}

    # -------------------------- Approach & Landing ---------------------------
    def final_approach(self, dt=0.1, t_end=40):
        """
        Final approach typically maintains a constant flight path angle and uses 
        the same kinematic and dynamic equations as the climb phase.
        γ = -5.5 deg (slope used for London City Airport approach)
        dx/dt = Vcos⁡γcos⁡ψ
        dy/dt = Vcos⁡γsin⁡ψ
        dz/dt = -Vsin⁡γ

        """
        gamma = -5.5 * deg
        x=y=z=0

        for _ in range(int(t_end/dt)):
            T = self.plane.compute_thrust(self.V_idle)
            D = self.plane.compute_drag(self.V_idle)

            dVdt = (T - D - self.plane.m*9.81*sin(gamma)) / self.plane.m
            V += dVdt*dt

            x += V*math.cos(gamma)*math.cos(self.psi)*dt
            y += V*math.cos(gamma)*math.sin(self.psi)*dt
            z += -V*math.sin(gamma)*dt

        return {"trajectory": (x,y,z)}

    # ------------------------ Ground roll after landing ---------------------
    def ground_roll_landing(self, dt=0.1):
        """
        m * dV/dt = -D - F_rr - F_brake - mgsin⁡θ - T_rev

        """
        V = self.V_touchdown
        x = 0
        t = 0

        while V > 0:
            D = self.plane.compute_drag(V)
            L = self.plane.compute_aerodynamic_lift(V)
            F_rr = self.plane.compute_rolling_friction(L)
            F_brake = self.compute_braking_force()
            T_rev = self.T_reverse

            a = (-D - F_rr - F_brake - self.plane.m*9.81*math.sin(self.theta) - T_rev) / self.plane.m
            
            V += a*dt
            x += V*dt
            t += dt

        return {"stopped": True, "t_stop": t, "x_stop": x}


# ------ GROUND OPERATION CLASSES --------------------------------------------------------------
class GroundOperations:

    def __init__(self, runway_length: float, num_gates: int, runway_slope: float = 0.0):
        """
        Initialise ground operations system.

        Parameters
        ----------
        runway_length : float
            Length of the runway in meters.
        num_gates : int
            Total number of aircraft gates available at the airport.
        runway_slope : float
            Runway slope angle (theta) in radians.
        """
        self.runway_length = runway_length
        self.runway_slope = runway_slope

        # gates: None if empty, or plane object assigned
        self.gates = [None] * num_gates

        # state flags
        self.runway_busy = False  
        self.runway_free_time = 0.0  # when the runway becomes free

        # queues
        self.takeoff_queue = []
        self.landing_queue = []
        self.taxiing_planes = []

        # Additional features
        # Taxi time from gate to runway (per gate)
        # Taxi time from runway to gate (per gate)

# ------ RUNWAY LOGIC --------------------------------------------------------------
    def request_takeoff_slot(self, plane):
        """
        Add a plane to the takeoff queue.
        """
        self.takeoff_queue.append(plane)

    def request_landing_slot(self, plane, priority=False):
        """
        Add a plane to the landing queue.
        Adds priority logic 
        """
        if priority:
            self.landing_queue.insert(0, plane)
        else:
            self.landing_queue.append(plane)

    def is_runway_available(self, current_time: float) -> bool:
        """
        Check if the runway is currently free for use.
        """
        return (not self.runway_busy) or (current_time >= self.runway_free_time)

    def mark_runway_busy(self, plane, duration: float, current_time: float):
        """
        Mark the runway as occupied for a given duration.
        """
        self.runway_busy = True
        self.runway_free_time = current_time + duration
        self.current_runway_plane = plane


    def release_runway(self):
        """
        Immediately free the runway (used for forced overrides).
        """
        self.runway_busy = False
        self.current_runway_plane = None


    # ------ GATE LOGIC --------------------------------------------------------------

    def any_gate_available(self) -> bool:
        """
        Returns True if at least one gate is empty.
        """
        return any(gate is None for gate in self.gates)

    def get_free_gates(self) -> list:
        """
        Returns a list of indices for all free gates.
        """
        return [i for i, gate in enumerate(self.gates) if gate is None]

    def is_gate_free(self, gate_index: int) -> bool:
        """
        Check whether a specific gate is available.
        """
        if 0 <= gate_index < len(self.gates):
            return self.gates[gate_index] is None
        return False
    
    def allocate_gate(self, plane):
        """
        Assign a plane to the first available gate.
        Returns the gate index, or None if full.
        """
        for i, gate in enumerate(self.gates):
            if gate is None:
                self.gates[i] = plane
                plane.assigned_gate = i  # optional but very useful
                return i
        return None  # no available gates

    def release_gate(self, gate_index: int):
        """
        Free the given gate.
        """
        if 0 <= gate_index < len(self.gates):
            self.gates[gate_index] = None


# ------ TAXI LOGIC --------------------------------------------------------------

    def estimate_taxi_time(self, plane, target: str) -> float:
        """
        Estimate taxi duration based on target location.
        Could be improved with real LCY geometry.
        """
        if target == "runway":
            base = 75   # mean seconds gate → runway
            sigma = 12
        elif target == "gate":
            base = 90   # mean seconds runway → gate
            sigma = 15
        else:
            raise ValueError("target must be 'gate' or 'runway'")

        duration = random.normalvariate(base, sigma)
        return max(20, duration)  # minimum safety lower bound
    
    def start_taxi(self, plane, current_time: float, target: str):
        """
        Begin a taxi operation for a plane.
        This schedules an event instead of using time-step logic.
        """
        duration = self.estimate_taxi_time(plane, target)

        self.taxiing_planes.append({
            "plane": plane,
            "target": target,
            "finish_time": current_time + duration
        })

    def poll_taxi_events(self, current_time: float):
        """
        Returns a list of planes whose taxi operation completed this tick.
        Removes them from the taxi list.
        """
        completed = []

        for entry in list(self.taxiing_planes):
            if current_time >= entry["finish_time"]:
                self.taxiing_planes.remove(entry)
                completed.append(entry)

        return completed


# ------ FULL AIRPORT UPDATE LOOP --------------------------------------------------------------
    def update(self, current_time: float):
        """
        Main airport update loop.
        Processes:
        1. Taxi completions
        2. Runway release
        3. Scheduling next runway operation (landing priority)
        """

        # ----------------------------------------------------------------------
        # 1. HANDLE TAXI COMPLETIONS
        # ----------------------------------------------------------------------
        completed = self.poll_taxi_events(current_time)

        for entry in completed:
            plane = entry["plane"]
            target = entry["target"]

            if target == "gate":
                # Plane finished taxiing FROM runway TO gate
                assigned_gate = self.allocate_gate(plane)

                if assigned_gate is None:
                    print(f"[WARN] No gate available for {plane.callsign}")
                else:
                    plane.state = "at_gate"
                    plane.assigned_gate = assigned_gate

            elif target == "runway":
                # Plane finished taxiing FROM gate TO runway
                if plane not in self.takeoff_queue:
                    self.takeoff_queue.append(plane)
                plane.state = "waiting_takeoff"

        # ----------------------------------------------------------------------
        # 2. FREE RUNWAY IF BUSY PERIOD IS OVER
        # ----------------------------------------------------------------------
        if self.runway_busy and current_time >= self.runway_free_time:
            self.release_runway()

        # ----------------------------------------------------------------------
        # 3. IF RUNWAY IS FREE → START NEXT EVENT
        #    LANDING PRIORITY (typical real-world rule + LCY constraint)
        # ----------------------------------------------------------------------
        if not self.runway_busy:

            # --------------------------------------------------------------
            # 3A. LANDING PRIORITY
            # --------------------------------------------------------------
            if len(self.landing_queue) > 0:

                # LCY requires a gate BEFORE landing
                if self.any_gate_available():
                    plane = self.landing_queue.pop(0)

                    # Placeholder (do NOT use FlightDynamics yet)
                    duration = 40.0

                    self.mark_runway_busy(plane, duration, current_time)
                    plane.state = "landing"

                    # After landing → taxi to gate
                    landing_finish = current_time + duration
                    self.start_taxi(plane, landing_finish, target="gate")

                # else: no gate → landing delayed, nothing happens this tick

            # --------------------------------------------------------------
            # 3B. TAKEOFF (only if no landings)
            # --------------------------------------------------------------
            elif len(self.takeoff_queue) > 0:

                plane = self.takeoff_queue.pop(0)

                # Placeholder duration
                duration = 35.0

                self.mark_runway_busy(plane, duration, current_time)
                plane.state = "taking_off"

                # After takeoff → plane leaves ops, no taxi needed


# ------ GLOBAL FUNCTIONS --------------------------------------------------------------
# if the global functions can be moved into appropriate classes, or a new one (for encapsualtion purposes) this should be done


# ------ MAIN BODY --------------------------------------------------------------

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
    print("  - ground_operations.py: Airport operations")
    print("  - simulation.py: Simulation orchestration")
    print("=" * 70)
    
    # Run all tests
    test_aircraft_types()
    test_ground_operations()
    test_flight_dynamics()
    
    # Placeholder for full weekly simulation
    print("\n\nReady to implement full weekly simulation!")
    print("See simulation.py for the run_weekly_simulation() function.")
    print("\nPseudocode outline:")
    print("  1. Initialize airport (runway, 19 gates)")
    print("  2. Generate arrivals using Poisson process (~122/day)")
    print("  3. Simulate 7 days of operations")
    print("  4. Track runway usage, gate occupancy, queues")
    print("  5. Handle go-arounds, taxi operations")
    print("  6. Collect statistics and metrics")
    
    print("\n" + "=" * 70)
    print("Simulation setup complete. Ready for next phase!")
    print("=" * 70)