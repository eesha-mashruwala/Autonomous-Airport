"""
Aircraft Module

Defines the Plane parent class and specific aircraft subclasses used in the simulation.
Each plane has geometric and mass properties, plus aerodynamic helper methods.
"""

import numpy as np
import math
from typing import Optional


class Plane:
    """
    Parent class for all aircraft types.
    Handles geometry, mass, aerodynamic properties, and basic physics calculations.
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
        
        # Protected attributes
        self._mass = float(mass)
        self._S = float(S)
        self._b = float(b)
        self._CL_max = float(CL_max)
        self._CD0 = float(CD0)
        self._e = float(e)
        self._rho = float(rho)

        # Private identifier
        self.__id = str(identifier)

        # Public attributes
        self.gate: Optional[str] = None
        self.plane_type: str = "GenericPlane"
        self.state: str = "init"
        self.assigned_gate: Optional[int] = None

        # Engine/thrust parameters (defaults, can be overridden)
        self._T0 = 120000.0  # static thrust total (N)
        self._Ct = 0.0       # thrust decay coefficient (per m/s)

        # Rolling friction coefficient
        self._mu_rr = 0.02

        # Computed once
        self._AR = self.aspect_ratio()

    # ------------------ GETTERS AND SETTERS ---------------------------------
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
    def aspect_ratio(self):
        """Compute wing aspect ratio AR = b^2 / S."""
        return (self._b ** 2) / self._S

    def compute_thrust(self, V):
        """
        Thrust model: T(V) = T_0 * (1 - C_t * V)
        
        Used in all flight phases except ground_roll_landing().
        
        Parameters
        ----------
        V : float
            Velocity (m/s)
            
        Returns
        -------
        float
            Thrust force (N)
        """
        return max(0.0, self._T0 * (1.0 - self._Ct * V))

    def compute_drag(self, V, gamma=0.0, phi=0.0):
        """
        Aerodynamic Drag: D = 1/2 * ρ * V^2 * S * C_D
        
        Used in all flight phases.
        
        Parameters
        ----------
        V : float
            Velocity (m/s)
        gamma : float
            Flight path angle (radians)
        phi : float
            Bank angle (radians)
            
        Returns
        -------
        float
            Drag force (N)
        """
        if V < 0.1:  # avoid numerical issues at very low speeds
            return 0.0
        CD = self.compute_drag_coeff(V, gamma, phi)
        return 0.5 * self._rho * V**2 * self._S * CD

    def compute_drag_coeff(self, V, gamma=0.0, phi=0.0):
        """
        Drag Coefficient: C_D = C_D0 + (S * C_L^2) / (π * b^2 * e)
        
        Parameters
        ----------
        V : float
            Velocity (m/s)
        gamma : float
            Flight path angle (radians)
        phi : float
            Bank angle (radians)
            
        Returns
        -------
        float
            Drag coefficient (dimensionless)
        """
        if V < 0.1:  # avoid numerical issues at low speeds
            return self._CD0
        CL = self.compute_lift_coeff(V, gamma, phi)
        induced = (CL**2) / (math.pi * self._AR * self._e)
        return self._CD0 + induced

    def compute_lift_coeff(self, V, gamma=0.0, phi=0.0):
        """
        Lift Coefficient: C_L = (2 * m * g * cos(γ) * cos(φ)) / (ρ * V^2 * S)
        
        Parameters
        ----------
        V : float
            Velocity (m/s)
        gamma : float
            Flight path angle (radians)
        phi : float
            Bank angle (radians)
            
        Returns
        -------
        float
            Lift coefficient (dimensionless)
        """
        if V < 0.1:  # avoid division by zero at low speeds
            return 0.0
        num = 2 * self._mass * 9.81 * math.cos(gamma) * math.cos(phi)
        den = self._rho * V**2 * self._S
        return num / den

    def compute_aerodynamic_lift(self, V, gamma=0.0, phi=0.0):
        """
        Aerodynamic Lift: F_L = 1/2 * ρ * V^2 * S * C_L
        
        Used in rolling friction calculations.
        
        Parameters
        ----------
        V : float
            Velocity (m/s)
        gamma : float
            Flight path angle (radians)
        phi : float
            Bank angle (radians)
            
        Returns
        -------
        float
            Lift force (N)
        """
        CL = self.compute_lift_coeff(V, gamma, phi)
        return 0.5 * self._rho * V**2 * self._S * CL

    def compute_rolling_friction(self, V):
        """
        Rolling Friction: F_rr = μ_rr * (mg - F_L)
        
        Used during ground roll (takeoff and landing).
        
        Parameters
        ----------
        V : float
            Velocity (m/s)
            
        Returns
        -------
        float
            Rolling friction force (N)
        """
        L = self.compute_aerodynamic_lift(V)
        return self._mu_rr * (self._mass * 9.81 - L)

    def compute_braking_force(self, beta=1.0, mu_b=0.3):
        """
        Braking Force: F_brake(t) = β * μ_b * mg
        
        Used during landing ground roll.
        
        Parameters
        ----------
        beta : float
            Brake application factor (0-1, where 1 is full brakes)
        mu_b : float
            Braking friction coefficient
            
        Returns
        -------
        float
            Braking force (N)
        """
        return beta * mu_b * self._mass * 9.81

    def stall_speed(self):
        """
        Stall Speed: V_stall = sqrt(2 * m * g / (ρ * S * CL_max))
        
        Returns
        -------
        float
            Stall speed (m/s)
        """
        return np.sqrt((2.0 * self._mass * 9.81) / (self._rho * self._S * self._CL_max))

    def __repr__(self):
        return f"<{self.plane_type} {self.__id}, state={self.state}, gate={self.assigned_gate}>"


# ------ AIRCRAFT SUBCLASSES --------------------------------------------------------------

class AirbusA220(Plane):
    """
    Airbus A220-100 specific configuration.
    Regional jet commonly used at London City Airport.
    """

    def __init__(self, identifier: str = 'AirbusA220'):
        super().__init__(
            identifier=identifier,
            mass=70900.0,
            S=112.3,
            b=35.1,
            CL_max=1.6,
            CD0=0.02,
            e=0.8
        )
        self.plane_type = 'AirbusA220'


class EmbraerE170(Plane):
    """
    Embraer E170 specific configuration.
    Regional jet commonly used at London City Airport.
    """

    def __init__(self, identifier: str = 'EmbraerE170'):
        super().__init__(
            identifier=identifier,
            mass=37200.0,
            S=72.72,
            b=28.65,
            CL_max=1.5,
            CD0=0.019,
            e=0.82
        )
        self.plane_type = 'EmbraerE170'


class Dash8_400(Plane):
    """
    Dash 8-400 specific configuration.
    Turboprop aircraft used at London City Airport.
    """

    def __init__(self, identifier: str = 'Dash8_400'):
        super().__init__(
            identifier=identifier,
            mass=29574.0,
            S=63.1,
            b=28.42,
            CL_max=1.5,
            CD0=0.019,
            e=0.82
        )
        self.plane_type = 'Dash8_400'


class ATR72_600(Plane):
    """
    ATR 72-600 specific configuration.
    Turboprop aircraft used at London City Airport.
    """

    def __init__(self, identifier: str = 'ATR72_600'):
        super().__init__(
            identifier=identifier,
            mass=22800.0,
            S=61.0,
            b=27.05,
            CL_max=1.5,
            CD0=0.019,
            e=0.82
        )
        self.plane_type = 'ATR72_600'
