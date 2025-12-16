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
        Initialise a Plane instance.

        Parameters
        ----------
        identifier : str
            Unique plane ID (e.g., 'BA123', 'EZY456').
        mass : float
            Aircraft mass in kilogrammes (kg).
        S : float
            Wing planform area in square metres (m²).
        b : float
            Wingspan in metres (m).
        CL_max : float
            Maximum lift coefficient before stall (dimensionless).
        CD0 : float
            Zero-lift drag coefficient (parasite drag).
        e : float
            Oswald efficiency factor accounting for induced drag (dimensionless, typically 0.7-0.9).
        rho : float
            Air density in kg/m³ (standard sea level: 1.225 kg/m³).
        """
        
        # Protected attributes (aircraft geometry and aerodynamic properties)
        self._mass = float(mass)
        self._S = float(S)
        self._b = float(b)
        self._CL_max = float(CL_max)
        self._CD0 = float(CD0)
        self._e = float(e)
        self._rho = float(rho)

        # Private identifier (immutable once set)
        self.__id = str(identifier)

        # Public attributes (mutable operational state)
        self.gate: Optional[str] = None
        self.plane_type: str = "GenericPlane"
        self.state: str = "init"  # Current operational state (init, approaching, landed, at_gate, departing)
        self.assigned_gate: Optional[int] = None

        # Engine/thrust parameters (defaults, overridden in subclasses)
        self._T0 = 120000.0  # Static thrust total in newtons (N) at zero velocity
        self._Ct = 0.00015   # Thrust decay coefficient (per m/s) - models thrust reduction with speed

        # Rolling friction coefficient (tyre-runway interaction)
        self._mu_rr = 0.02

        # Computed once and cached for efficiency
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
        Thrust model: T(V) = T₀ * (1 - Cₜ * V)
        
        Simple linear thrust decay model approximating jet engine behaviour.
        Thrust decreases with forward velocity due to reduced intake momentum.
        Used in all flight phases except ground_roll_landing().
        
        Parameters
        ----------
        V : float
            True airspeed in metres per second (m/s)
            
        Returns
        -------
        float
            Available thrust force in newtons (N), clamped to non-negative values
        """
        return max(0.0, self._T0 * (1.0 - self._Ct * V))

    def compute_drag(self, V, gamma=0.0, phi=0.0):
        """
        Aerodynamic Drag: D = ½ρV²SC_D
        
        Computes total aerodynamic drag using the drag equation.
        Includes both parasite drag (C_D0) and induced drag from lift generation.
        Used in all flight phases to model air resistance.
        
        Parameters
        ----------
        V : float
            True airspeed in metres per second (m/s)
        gamma : float
            Flight path angle in radians (positive = climbing)
        phi : float
            Bank angle in radians (positive = right bank)
            
        Returns
        -------
        float
            Total drag force in newtons (N)
        """
        if V < 0.1:  # Avoid numerical issues at very low speeds
            return 0.0
        CD = self.compute_drag_coeff(V, gamma, phi)
        return 0.5 * self._rho * V**2 * self._S * CD

    def compute_drag_coeff(self, V, gamma=0.0, phi=0.0):
        """
        Drag Coefficient: C_D = C_D0 + C_L²/(πARe)
        
        Total drag comprises parasite drag (C_D0) and induced drag.
        Induced drag arises from wingtip vortices and increases with lift coefficient.
        
        Parameters
        ----------
        V : float
            True airspeed in metres per second (m/s)
        gamma : float
            Flight path angle in radians
        phi : float
            Bank angle in radians
            
        Returns
        -------
        float
            Total drag coefficient (dimensionless)
        """
        if V < 0.1:  # Avoid numerical issues at low speeds
            return self._CD0
        CL = self.compute_lift_coeff(V, gamma, phi)
        # Induced drag: C_Di = C_L² / (π * AR * e)
        induced = (CL**2) / (math.pi * self._AR * self._e)
        return self._CD0 + induced

    def compute_lift_coeff(self, V, gamma=0.0, phi=0.0):
        """
        Lift Coefficient: C_L = 2mg⋅cos(γ)⋅cos(φ) / (ρV²S)
        
        Derived from equilibrium flight condition: Lift = Weight⋅cos(γ)⋅cos(φ)
        In level flight (γ=0, φ=0), lift exactly equals weight.
        During manoeuvres, additional lift is required.
        Capped at CL_max to prevent stall and numerical instability.
        
        Parameters
        ----------
        V : float
            True airspeed in metres per second (m/s)
        gamma : float
            Flight path angle in radians (positive = climbing)
        phi : float
            Bank angle in radians (positive = right bank)
            
        Returns
        -------
        float
            Required lift coefficient (dimensionless), capped at CL_max
        """
        if V < 0.1:  # Avoid division by zero at low speeds
            return 0.0
        num = 2 * self._mass * 9.81 * math.cos(gamma) * math.cos(phi)
        den = self._rho * V**2 * self._S
        CL = num / den
        # Cap at CL_max to prevent stall and unrealistic induced drag
        return min(CL, self._CL_max)

    def compute_aerodynamic_lift(self, V, gamma=0.0, phi=0.0):
        """
        Aerodynamic Lift: L = ½ρV²SC_L
        
        Computes upward force generated by airflow over wings.
        During ground roll, lift reduces the normal force on landing gear,
        thereby reducing rolling friction.
        
        Parameters
        ----------
        V : float
            True airspeed in metres per second (m/s)
        gamma : float
            Flight path angle in radians
        phi : float
            Bank angle in radians
            
        Returns
        -------
        float
            Lift force in newtons (N)
        """
        CL = self.compute_lift_coeff(V, gamma, phi)
        return 0.5 * self._rho * V**2 * self._S * CL

    def compute_rolling_friction(self, V):
        """
        Rolling Friction: F_rr = μ_rr × (mg - L)
        
        Models tyre-runway friction during ground operations.
        Normal force decreases as lift increases with speed,
        reducing rolling resistance during takeoff acceleration.
        
        Parameters
        ----------
        V : float
            Ground speed in metres per second (m/s)
            
        Returns
        -------
        float
            Rolling friction force in newtons (N), always non-negative
        """
        L = self.compute_aerodynamic_lift(V)
        # Normal force = Weight - Lift (reduces as speed increases)
        friction = self._mu_rr * (self._mass * 9.81 - L)
        return max(0.0, friction)

    def compute_braking_force(self, beta=1.0, mu_b=0.3):
        """
        Braking Force: F_brake = β × μ_b × mg
        
        Models wheel brakes applied during landing rollout.
        Beta controls brake pedal position (0=released, 1=maximum).
        Typical values: μ_b ≈ 0.3-0.4 for dry runway, 0.1-0.2 for wet.
        
        Parameters
        ----------
        beta : float
            Brake application factor (0-1, where 1 is full brakes)
        mu_b : float
            Braking friction coefficient (runway-dependent)
            
        Returns
        -------
        float
            Braking force in newtons (N)
        """
        return beta * mu_b * self._mass * 9.81

    def stall_speed(self):
        """
        Stall Speed: V_stall = √(2mg / ρS⋅CL_max)
        
        Minimum airspeed at which level flight can be maintained.
        Below this speed, wings cannot generate sufficient lift.
        Approach speeds typically 1.3-1.5 × V_stall for safety margin.
        
        Returns
        -------
        float
            Stall speed in metres per second (m/s)
        """
        return np.sqrt((2.0 * self._mass * 9.81) / (self._rho * self._S * self._CL_max))

    def __repr__(self):
        return f"<{self.plane_type} {self.__id}, state={self.state}, gate={self.assigned_gate}>"


# ------ AIRCRAFT SUBCLASSES --------------------------------------------------------------

class AirbusA220(Plane):
    """
    Airbus A220-100 specific configuration.
    
    Modern regional jet optimised for short runways like London City Airport.
    Features advanced geared turbofan engines for improved fuel efficiency.
    Typical capacity: 100-130 passengers.
    
    Key Specifications:
    - Mass: 70,900 kg (maximum takeoff weight)
    - Wing area: 112.3 m²
    - Wingspan: 35.1 m
    - Engines: 2× Pratt & Whitney PW1500G geared turbofan
    """

    def __init__(self, identifier: str = 'AirbusA220'):
        super().__init__(
            identifier=identifier,
            mass=70900.0,       # kg - Maximum takeoff weight
            S=112.3,            # m² - Wing planform area
            b=35.1,             # m - Wingspan
            CL_max=1.6,         # Maximum lift coefficient (high-lift devices deployed)
            CD0=0.02,           # Zero-lift drag coefficient
            e=0.8               # Oswald efficiency factor
        )
        self.plane_type = 'AirbusA220'
        # Pratt & Whitney PW1500G engines (2× 10,600 kgf ≈ 208,000 N total)
        # Increased to 240,000 N for climb performance to 105 m/s
        self._T0 = 240000.0     # Static thrust (N) - tuned for simulation performance
        self._Ct = 0.0002       # Thrust decay coefficient


class EmbraerE170(Plane):
    """
    Embraer E170 specific configuration.
    
    Compact regional jet ideal for London City's steep approach requirements.
    Part of the E-Jet family, certified for London City operations.
    Typical capacity: 70-80 passengers.
    
    Key Specifications:
    - Mass: 37,200 kg (maximum takeoff weight)
    - Wing area: 72.72 m²
    - Wingspan: 28.65 m
    - Engines: 2× GE CF34-8E turbofan
    """

    def __init__(self, identifier: str = 'EmbraerE170'):
        super().__init__(
            identifier=identifier,
            mass=37200.0,       # kg - Maximum takeoff weight
            S=72.72,            # m² - Wing planform area
            b=28.65,            # m - Wingspan
            CL_max=1.5,         # Maximum lift coefficient
            CD0=0.019,          # Zero-lift drag coefficient
            e=0.82              # Oswald efficiency factor (good wing design)
        )
        self.plane_type = 'EmbraerE170'
        # GE CF34-8E engines (2× 6,830 kgf ≈ 134,000 N total)
        self._T0 = 134000.0     # Static thrust (N)
        self._Ct = 0.00018      # Thrust decay coefficient


class Dash8_400(Plane):
    """
    Bombardier Dash 8-400 (Q400) specific configuration.
    
    High-performance turboprop optimised for short-haul regional routes.
    Known for fuel efficiency and low operating costs.
    "Q" stands for "Quiet" - features noise suppression technology.
    Typical capacity: 70-78 passengers.
    
    Key Specifications:
    - Mass: 29,574 kg (maximum takeoff weight)
    - Wing area: 63.1 m²
    - Wingspan: 28.42 m
    - Engines: 2× Pratt & Whitney Canada PW150A turboprop
    """

    def __init__(self, identifier: str = 'Dash8_400'):
        super().__init__(
            identifier=identifier,
            mass=29574.0,       # kg - Maximum takeoff weight
            S=63.1,             # m² - Wing planform area
            b=28.42,            # m - Wingspan
            CL_max=1.5,         # Maximum lift coefficient
            CD0=0.019,          # Zero-lift drag coefficient
            e=0.82              # Oswald efficiency factor
        )
        self.plane_type = 'Dash8_400'
        # PW150A engines (2× 5,071 shp ≈ 75,000 N equivalent thrust)
        # Increased to 97,000 N for climb performance to 105 m/s
        self._T0 = 97000.0      # Static thrust (N) - turboprop equivalent
        self._Ct = 0.00015      # Thrust decay coefficient (lower for turboprops)


class ATR72_600(Plane):
    """
    ATR 72-600 specific configuration.
    
    Popular regional turboprop renowned for exceptional fuel efficiency.
    Well-suited for short runways and regional connectivity.
    Typical capacity: 68-74 passengers.
    
    Key Specifications:
    - Mass: 22,800 kg (maximum takeoff weight)
    - Wing area: 61.0 m²
    - Wingspan: 27.05 m
    - Engines: 2× Pratt & Whitney Canada PW127M turboprop
    """

    def __init__(self, identifier: str = 'ATR72_600'):
        super().__init__(
            identifier=identifier,
            mass=22800.0,       # kg - Maximum takeoff weight
            S=61.0,             # m² - Wing planform area
            b=27.05,            # m - Wingspan
            CL_max=1.5,         # Maximum lift coefficient
            CD0=0.019,          # Zero-lift drag coefficient
            e=0.82              # Oswald efficiency factor
        )
        self.plane_type = 'ATR72_600'
        # PW127M engines (2× 2,750 shp ≈ 55,000 N equivalent thrust)
        # Increased to 76,000 N for climb performance to 105 m/s
        self._T0 = 76000.0      # Static thrust (N) - turboprop equivalent
        self._Ct = 0.00012      # Thrust decay coefficient (lower for turboprops)
