"""
Flight Dynamics Module

Implements the physics of aircraft flight through various phases:
- Ground roll (takeoff)
- Climb
- Cruise/Holding
- Approach
- Landing

Uses numerical integration to solve ODEs for aircraft motion.
"""

import numpy as np
import math
from typing import List, Dict, Optional
from aircraft import Plane


class FlightDynamics:
    """
    Flight dynamics class that implements ODEs for each flight phase.
    Collects telemetry data and provides configurable flight parameters.
    """

    def __init__(self, plane: Plane):
        """
        Initialize FlightDynamics attached to a Plane.

        Parameters
        ----------
        plane : Plane
            The physical plane object containing geometry and mass.
        """
        self._plane = plane  # protected reference to the aircraft
        self._telemetry: List[Dict] = []  # stores dictionaries of telemetry

        # Flight state variables
        self.t = 0.0        # time (s)
        self.x = 0.0        # position x (m)
        self.y = 0.0        # position y (m)
        self.z = 0.0        # altitude (m)
        self.V = 0.0        # velocity (m/s)
        self.gamma = 0.0    # flight path angle (radians)
        self.psi = 0.0      # heading angle (radians)
        self.phi = 0.0      # bank angle (radians)

        # Current forces
        self.T = 0.0        # thrust (N)
        self.D = 0.0        # drag (N)
        self.L = 0.0        # lift (N)
        self.F_rr = 0.0     # rolling friction (N)
        self.F_brake = 0.0  # braking force (N)

        # Control inputs
        self.throttle_cmd = 0.0
        self.brake_cmd = 0.0

        # Runway parameters
        self._theta = 0.0   # runway slope angle (radians)

    @property
    def plane(self) -> Plane:
        """Get reference to the aircraft."""
        return self._plane

    # -------------------- TELEMETRY MANAGEMENT -------------------------------
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

    def clear_telemetry(self):
        """Clear all telemetry data."""
        self._telemetry = []

    # ------------------------ VELOCITY/TRUE AIRSPEED -------------------------
    def compute_airspeed_derivative(self, V, gamma, thrust):
        """
        Airspeed rate of change: m * dV/dt = T - D - mg*sin(γ)
        
        Used in climb, holding, and approach phases.
        
        Parameters
        ----------
        V : float
            Current velocity (m/s)
        gamma : float
            Flight path angle (radians)
        thrust : float
            Thrust force (N)
            
        Returns
        -------
        float
            dV/dt (m/s^2)
        """
        D = self._plane.compute_drag(V, gamma)
        return (thrust - D - self._plane.mass * 9.81 * math.sin(gamma)) / self._plane.mass

    # ------------------------ HEADING RATE -------------------------
    def compute_heading_rate(self, V, phi):
        """
        Heading rate: dψ/dt = g/V * tan(ϕ)
        
        Used in climb, holding, and approach phases.
        
        Parameters
        ----------
        V : float
            Velocity (m/s)
        phi : float
            Bank angle (radians)
            
        Returns
        -------
        float
            dψ/dt (radians/s)
        """
        if V < 1.0:  # avoid division by zero
            return 0.0
        return 9.81 / V * math.tan(phi)

    # ------------------------ GROUND ROLL (TAKEOFF) -------------------------
    def ground_roll_takeoff(self, dt=0.1, Vr=70.0, max_time=120.0):
        """
        Simulate ground roll (1D along runway) using Newton's 2nd law.
        
        Equation: m * dV/dt = T - D - F_rr - mg*sin(θ)
        
        Integrates until velocity reaches rotation speed Vr.
        
        Parameters
        ----------
        dt : float
            Time step (s)
        Vr : float
            Rotation speed (m/s) - speed at which aircraft lifts off
        max_time : float
            Maximum simulation time (s) to prevent infinite loops
            
        Returns
        -------
        dict
            Contains V_rot (rotation velocity), distance (takeoff distance),
            time (duration), and success (bool)
        """
        V = 0.0
        x = 0.0
        t = 0.0

        self.clear_telemetry()

        while V < Vr and t < max_time:
            # Compute forces
            self.T = self._plane.compute_thrust(V)
            self.D = self._plane.compute_drag(V)
            self.L = self._plane.compute_aerodynamic_lift(V)
            self.F_rr = self._plane.compute_rolling_friction(V)

            # Net acceleration
            a = (self.T - self.D - self.F_rr - self._plane.mass * 9.81 * math.sin(self._theta)) / self._plane.mass

            # Euler integration
            V += a * dt
            x += V * dt
            t += dt

            # Update state for recording
            self.t = t
            self.V = V
            self.x = x
            self.throttle_cmd = 1.0
            self._record()

        success = V >= Vr
        return {
            "V_rot": V,
            "distance": x,
            "time": t,
            "success": success
        }

    # ------------------------------ CLIMB ----------------------------------
    def climb_phase(self, t_end=60.0, dt=0.1, target_altitude=1000.0, gamma=0.1):
        """
        Simplified climb phase using 3D kinematics and dynamics.
        
        Equations:
        - dx/dt = V*cos(γ)*cos(ψ)
        - dy/dt = V*cos(γ)*sin(ψ)
        - dz/dt = -V*sin(γ)  (note: z-axis points down in aviation convention)
        - m*dV/dt = T - D - mg*sin(γ)
        
        Parameters
        ----------
        t_end : float
            Duration of climb phase (s)
        dt : float
            Time step (s)
        target_altitude : float
            Target altitude to reach (m)
        gamma : float
            Climb angle (radians, positive = climbing)
            
        Returns
        -------
        dict
            Final position (x, y, z) and velocity
        """
        self.gamma = gamma
        x = y = z = 0.0
        t = 0.0

        while t < t_end and abs(z) < target_altitude:
            # Compute thrust and drag
            self.T = self._plane.compute_thrust(self.V)
            self.D = self._plane.compute_drag(self.V, self.gamma)

            # Velocity change
            dVdt = self.compute_airspeed_derivative(self.V, self.gamma, self.T)
            self.V += dVdt * dt

            # Position updates
            x += self.V * math.cos(self.gamma) * math.cos(self.psi) * dt
            y += self.V * math.cos(self.gamma) * math.sin(self.psi) * dt
            z += -self.V * math.sin(self.gamma) * dt  # negative because z-down

            t += dt

            # Update state
            self.t = t
            self.x = x
            self.y = y
            self.z = z
            self.throttle_cmd = 0.8
            self._record()

        return {
            "x": x,
            "y": y,
            "z": z,
            "V": self.V,
            "time": t
        }

    # ------------------------------ HOLDING ---------------------------------
    def holding_phase(self, t_hold=120.0, dt=0.1, V_hold=80.0, altitude=1000.0):
        """
        Holding phase where plane maintains constant altitude and speed.
        Can fly in circles or straight path.
        
        Equations:
        - dx/dt = V_holding*cos(γ)*cos(ψ)
        - dy/dt = V_holding*cos(γ)*sin(ψ)
        - dz/dt = -V_holding*sin(γ)
        
        Parameters
        ----------
        t_hold : float
            Duration of holding (s)
        dt : float
            Time step (s)
        V_hold : float
            Holding velocity (m/s)
        altitude : float
            Holding altitude (m)
            
        Returns
        -------
        dict
            Final position (x, y, z)
        """
        self.V = V_hold
        self.gamma = 0.0  # level flight
        x = y = 0.0
        z = -altitude  # negative because z-down convention
        t = 0.0

        while t < t_hold:
            # Constant velocity holding
            x += self.V * math.cos(self.gamma) * math.cos(self.psi) * dt
            y += self.V * math.cos(self.gamma) * math.sin(self.psi) * dt
            z += -self.V * math.sin(self.gamma) * dt

            t += dt

            # Update state
            self.t = t
            self.x = x
            self.y = y
            self.z = z
            self.throttle_cmd = 0.5
            self._record()

        return {
            "x": x,
            "y": y,
            "z": z
        }

    # -------------------------- APPROACH & LANDING ---------------------------
    def final_approach(self, dt=0.1, t_end=40.0, gamma_approach=-0.096):
        """
        Final approach maintains a constant descent angle.
        London City Airport uses γ = -5.5° ≈ -0.096 radians (steeper than typical)
        
        Equations:
        - dx/dt = V*cos(γ)*cos(ψ)
        - dy/dt = V*cos(γ)*sin(ψ)
        - dz/dt = -V*sin(γ)
        - m*dV/dt = T - D - mg*sin(γ)
        
        Parameters
        ----------
        dt : float
            Time step (s)
        t_end : float
            Duration of approach (s)
        gamma_approach : float
            Approach angle in radians (negative = descending)
            Default: -5.5 degrees for London City Airport
            
        Returns
        -------
        dict
            Trajectory and final state
        """
        self.gamma = gamma_approach
        x = y = z = 0.0
        t = 0.0

        while t < t_end:
            # Idle thrust during approach
            self.T = self._plane.compute_thrust(self.V) * 0.3  # reduced thrust
            self.D = self._plane.compute_drag(self.V, self.gamma)

            # Velocity change
            dVdt = self.compute_airspeed_derivative(self.V, self.gamma, self.T)
            self.V += dVdt * dt

            # Position updates
            x += self.V * math.cos(self.gamma) * math.cos(self.psi) * dt
            y += self.V * math.cos(self.gamma) * math.sin(self.psi) * dt
            z += -self.V * math.sin(self.gamma) * dt

            t += dt

            # Update state
            self.t = t
            self.x = x
            self.y = y
            self.z = z
            self.throttle_cmd = 0.3
            self._record()

        return {
            "trajectory": (x, y, z),
            "V_touchdown": self.V,
            "time": t
        }

    # ------------------------ GROUND ROLL AFTER LANDING ---------------------
    def ground_roll_landing(self, dt=0.1, V_touchdown=65.0, max_time=120.0,
                           T_reverse=15000.0, beta=0.5, mu_b=0.18):
        """
        Ground roll deceleration after touchdown.
        
        Equation: m * dV/dt = -D - F_rr - F_brake - mg*sin(θ) - T_rev
        
        Parameters
        ----------
        dt : float
            Time step (s)
        V_touchdown : float
            Initial velocity at touchdown (m/s)
        max_time : float
            Maximum simulation time (s)
        T_reverse : float
            Thrust reverser force (N)
        beta : float
            Brake application factor (0-1)
        mu_b : float
            Braking friction coefficient
            
        Returns
        -------
        dict
            Stopping time, distance, and success flag
        """
        V = V_touchdown
        x = 0.0
        t = 0.0

        self.clear_telemetry()

        while V > 0.1 and t < max_time:  # stop when nearly stopped
            # Compute forces
            self.D = self._plane.compute_drag(V)
            self.L = self._plane.compute_aerodynamic_lift(V)
            self.F_rr = self._plane.compute_rolling_friction(V)
            self.F_brake = self._plane.compute_braking_force(beta, mu_b)

            # Net deceleration (all forces oppose motion)
            a = (-self.D - self.F_rr - self.F_brake 
                 - self._plane.mass * 9.81 * math.sin(self._theta) 
                 - T_reverse) / self._plane.mass

            # Euler integration
            V += a * dt
            V = max(0.0, V)  # ensure non-negative
            x += V * dt
            t += dt

            # Update state
            self.t = t
            self.V = V
            self.x = x
            self.throttle_cmd = 0.0
            self.brake_cmd = beta
            self._record()

        success = V <= 0.1
        return {
            "stopped": success,
            "t_stop": t,
            "x_stop": x
        }

    # ------------------------ UTILITY METHODS -------------------------------
    def set_runway_slope(self, theta: float):
        """
        Set runway slope angle.
        
        Parameters
        ----------
        theta : float
            Runway slope angle in radians
        """
        self._theta = theta

    def reset_state(self):
        """Reset all state variables to initial conditions."""
        self.t = 0.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.V = 0.0
        self.gamma = 0.0
        self.psi = 0.0
        self.phi = 0.0
        self.T = 0.0
        self.D = 0.0
        self.L = 0.0
        self.F_rr = 0.0
        self.F_brake = 0.0
        self.throttle_cmd = 0.0
        self.brake_cmd = 0.0
        self.clear_telemetry()
