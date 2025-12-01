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
import random
from typing import List, Dict, Optional, Tuple
from aircraft import Plane


def _wrap_angle(angle: float) -> float:
    """Normalize angle to [0, 2π)."""
    return angle % (2 * math.pi)


def _angle_difference(target: float, current: float) -> float:
    """Shortest signed angular difference."""
    return math.atan2(math.sin(target - current), math.cos(target - current))


class WindModel:
    """
    Simple stochastic wind field for airborne phases.

    Produces a slowly varying horizontal wind vector with occasional small
    adjustments in speed and direction plus light gust noise.
    """

    def __init__(
        self,
        min_speed: float = 0.0,
        max_speed: float = 11.0,
        change_interval: Tuple[float, float] = (25.0, 60.0),
        max_dir_step_deg: float = 8.0,
        max_speed_step: float = 2.0,
        smoothing: float = 6.0,
        gust_sigma: float = 0.5,
        vertical_sigma: float = 0.15,
    ):
        self.min_speed = min_speed
        self.max_speed = max_speed
        self.change_interval = change_interval
        self.max_dir_step = math.radians(max_dir_step_deg)
        self.max_speed_step = max_speed_step
        self.smoothing = max(1.0, smoothing)
        self.gust_sigma = gust_sigma
        self.vertical_sigma = vertical_sigma

        self.direction = random.uniform(0.0, 2 * math.pi)
        self.speed = random.uniform(self.min_speed, self.max_speed)
        self.target_direction = self.direction
        self.target_speed = self.speed

        self.time_since_change = 0.0
        self.next_change = random.uniform(*self.change_interval)
        self.vector = np.zeros(3)

    def update(self, dt: float) -> np.ndarray:
        """Advance the wind state and return the current wind vector (m/s)."""
        self.time_since_change += dt
        if self.time_since_change >= self.next_change:
            self.time_since_change = 0.0
            self.next_change = random.uniform(*self.change_interval)
            dir_delta = random.uniform(-self.max_dir_step, self.max_dir_step)
            self.target_direction = _wrap_angle(self.direction + dir_delta)
            speed_delta = random.uniform(-self.max_speed_step, self.max_speed_step)
            self.target_speed = min(
                self.max_speed, max(self.min_speed, self.speed + speed_delta)
            )

        blend = min(1.0, dt / self.smoothing)
        dir_error = _angle_difference(self.target_direction, self.direction)
        self.direction = _wrap_angle(self.direction + dir_error * blend)
        self.speed += (self.target_speed - self.speed) * blend

        # Small continuous jitter to avoid perfectly smooth flow
        jitter_dir = math.radians(random.uniform(-0.2, 0.2))
        self.direction = _wrap_angle(self.direction + jitter_dir * dt)
        gust = random.gauss(0.0, self.gust_sigma)
        effective_speed = max(0.0, self.speed + gust)

        vx = effective_speed * math.cos(self.direction)
        vy = effective_speed * math.sin(self.direction)
        vz = random.gauss(0.0, self.vertical_sigma)
        self.vector = np.array([vx, vy, vz], dtype=float)
        return self.vector.copy()

    def current(self) -> np.ndarray:
        """Return the current wind vector without advancing the model."""
        return self.vector.copy()


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
    
    def get_history(self):
        """
        Get telemetry data as numpy arrays for plotting.
        
        Returns
        -------
        dict
            Dictionary with arrays for each telemetry field (t, x, y, z, V, etc.)
        """
        import numpy as np
        if not self._telemetry:
            return {}
        
        # Extract all keys from first entry
        keys = self._telemetry[0].keys()
        
        # Create dictionary of numpy arrays
        history = {}
        for key in keys:
            if key == "time":
                history['t'] = np.array([entry[key] for entry in self._telemetry])
            else:
                history[key] = np.array([entry[key] for entry in self._telemetry])
        
        return history

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
        
        Equation: m * dV/dt = T - D - F_rr
        
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
        
        # Simplified constants for ground roll
        mu_rr = 0.02  # rolling resistance coefficient
        
        while V < Vr and t < max_time:
            # Compute forces
            self.T = self._plane.compute_thrust(V)
            
            # Simple drag during ground roll (low speed, use baseline CD)
            if V < 1.0:
                self.D = 0.0
            else:
                # Use simplified drag with CD0 + small induced component
                CL_ground = min(0.3, 2 * self._plane.mass * 9.81 / (self._plane._rho * V**2 * self._plane._S))
                CD_ground = self._plane._CD0 + 0.1 * CL_ground**2
                self.D = 0.5 * self._plane._rho * V**2 * self._plane._S * CD_ground
            
            # Rolling friction (reduces as lift increases)
            self.L = 0.5 * self._plane._rho * V**2 * self._plane._S * min(0.3, CL_ground) if V >= 1.0 else 0.0
            self.F_rr = mu_rr * (self._plane.mass * 9.81 - self.L)

            # Net acceleration (assume flat runway, theta=0)
            a = (self.T - self.D - self.F_rr) / self._plane.mass

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
    def climb_phase(self, t_end=60.0, dt=0.1, target_altitude=1000.0, gamma=0.1, V_target=85.0):
        """
        Simplified climb phase using 3D kinematics and dynamics.
        
        Equations:
        - dx/dt = V*cos(γ)*cos(ψ)
        - dy/dt = V*cos(γ)*sin(ψ)
        - dz/dt = V*sin(γ)  (positive z = up)
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
        V_target : float
            Target climb speed (m/s)
            
        Returns
        -------
        dict
            Final position (x, y, z) and velocity
        """
        self.gamma = gamma
        x = y = z = 0.0
        t = 0.0

        while t < t_end and z < target_altitude:
            # Use high thrust during climb to maintain/increase speed
            # Full throttle until we reach target speed, then moderate
            if self.V < V_target:
                throttle_factor = 0.95  # near full thrust for acceleration
            else:
                throttle_factor = 0.75  # maintain speed
            
            self.T = self._plane.compute_thrust(self.V) * throttle_factor
            self.D = self._plane.compute_drag(self.V, self.gamma)

            # Velocity change using ODE
            dVdt = self.compute_airspeed_derivative(self.V, self.gamma, self.T)
            self.V += dVdt * dt

            # Position updates (positive z = up)
            x += self.V * math.cos(self.gamma) * math.cos(self.psi) * dt
            y += self.V * math.cos(self.gamma) * math.sin(self.psi) * dt
            z += self.V * math.sin(self.gamma) * dt  # positive z = climbing

            t += dt

            # Update state
            self.t = t
            self.x = x
            self.y = y
            self.z = z
            self.throttle_cmd = throttle_factor
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
