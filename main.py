"""
Airport Flight Dynamics Module

This file provides a skeleton overview containing two major classes:

1) Plane classes: Plane (parent) and plane-specific subclasses (AirbusA220,
   EmbraerE170, Dash8-400, ATR72-600). Plane objects hold geometric and mass-related properties,
   plus simple getters/setters and helper aerodynamic properties.

2) FlightDynamics: a parent class that implements flight-phase dynamics (ground
   roll takeoff, climb, cruise, approach, landing roll) using the equations
   provided within. 
   
FlightDynamics operates on a Plane instance and collects telemetry (fill in data)

"""


# ------ PLANE CLASSES --------------------------------------------------------------

class Plane:
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

    # ------------------ aerodynamic helpers ---------------------------------
    def aspect_ratio(self):
        """Compute wing aspect ratio AR = b^2 / S."""
        return (self._b ** 2) / self._S

    def lift(self, V: float, CL: float):
        """Compute lift force (N) at speed V and lift coefficient CL."""
        return 0.5 * self._rho * V ** 2 * self._S * CL

    def drag_coefficient(self, CL: float):
        """Compute drag coefficient CD from CL using a simple polar.
        CD = CD0 + (CL^2) / (pi * AR * e)
        """
        
        AR = self.aspect_ratio()
        return self._CD0 + (CL ** 2) / (np.pi * AR * self._e)

    def drag(self, V: float, CL: float):
        """Compute drag force at speed V using CL to estimate induced drag."""
        CD = self.drag_coefficient(CL)
        return 0.5 * self._rho * V ** 2 * self._S * CD

    def stall_speed(self):
        """Return a simple estimate of stall speed V_stall (m/s) using CL_max.

        V_stall = sqrt(2*m*g / (rho * S * CL_max))
        """
        g = 9.80665
        return np.sqrt((2.0 * self._mass * g) / (self._rho * self._S * self._CL_max))


# ------ PLANE SUBCLASSES --------------------------------------------------------------

class AirbusA220(Plane):
    """ Specific plane data for Airbus A220-100.
        Inheriting from Plane class         """

    def __init__(self, identifier: str = 'AirbusA220'):
        super().__init__(identifier=identifier, mass=60000.0, S=124.0, b=35.8,
                         CL_max=1.6, CD0=0.02, e=0.8)
        self.plane_type = 'AirbusA220'


class EmbraerE170(Plane):
    """ Specific plane data for Embraer E170.
        Inheriting from Plane class         """

    def __init__(self, identifier: str = 'EmbraerE170'):
        super().__init__(identifier=identifier, mass=68000.0, S=122.6, b=34.1,
                         CL_max=1.5, CD0=0.019, e=0.82)
        self.plane_type = 'EmbraerE170'


class Dash8_400(Plane):
    """ Specific plane data for Dash 8-400.
        Inheriting from Plane class         """

    def __init__(self, identifier: str = 'Dash8_400'):
        super().__init__(identifier=identifier, mass=68000.0, S=122.6, b=34.1,
                         CL_max=1.5, CD0=0.019, e=0.82)
        self.plane_type = 'Dash8_400'


class ATR72_600(Plane):
    """ Specific plane data for ATR 72-600.
        Inheriting from Plane class         """

    def __init__(self, identifier: str = 'ATR72_600'):
        super().__init__(identifier=identifier, mass=68000.0, S=122.6, b=34.1,
                         CL_max=1.5, CD0=0.019, e=0.82)
        self.plane_type = 'ATR72_600'

# ------ FLIGHT DYNAMICS CLASSES --------------------------------------------------------------

class FlightDynamics:
    """
    Flight dynamics class that implements phase solvers for a single Plane on a
    single runway: ground roll (takeoff), climb, cruise, approach and landing
    roll. This class collects telemetry and provides configurable parameters.

    Important: this class is phase-focused and intentionally separates
    aircraft properties (Plane) from dynamic integration logic.
    """

    def __init__(self, plane: Plane):
        """
        Initialize FlightDynamics attached to a Plane.

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

        # landing gear height used for touchdown detection
        self._h_LG = 1.5

    # -------------------- Telemetry management -------------------------------
    def _record(self, t: float, x: float, y: float, z: float, V: float,
                phase: str, extra: Optional[Dict] = None):
        """
        Record a telemetry point with standardized fields.

        Parameters
        ----------
        t, x, y, z, V : float
            Time (s), positions (m) and speed (m/s).
        phase : str
            Name of the flight phase.
        extra : dict, optional
            Additional fields to attach.
        """
        entry = {
            'time': float(t),
            'x': float(x),
            'y': float(y),
            'z': float(z),
            'V': float(V),
            'phase': phase,
            'plane_id': self._plane.id,
            'plane_type': self._plane.plane_type,
            'gate': self._plane.gate,
        }
        if extra:
            entry.update(extra)
        self._telemetry.append(entry)

    def telemetry(self) -> List[Dict]:
        """Return collected telemetry as a list of dicts."""
        return list(self._telemetry)

    # ------------------------ Thrust model ----------------------------------
    def thrust(self, V: float, t: float) -> float:
        """
        Simple thrust model T(V,t) = T0 * (1 - Ct * V). Ensures non-negative
        thrust; subclasses or user code can override _T0/_Ct to tune behaviour.

        Parameters
        ----------
        V : float
            Speed (m/s).
        t : float
            Time (s).

        Returns
        -------
        float
            Thrust along runway/flight axis (N).
        """
        return max(0.0, self._T0 * (1.0 - self._Ct * V))

    # ------------------------ Ground roll (takeoff) -------------------------
    def run_ground_roll_takeoff(self, V0: float = 0.0, x0: float = 0.0,
                                t_span: Tuple[float, float] = (0.0, 60.0),
                                V_rot: Optional[float] = None,
                                t_eval: Optional[np.ndarray] = None) -> Dict:
        """
        Simulate ground roll (1D along runway) using Newton's 2nd law.

        The ODE state is [x, V] where dx/dt = V and m dV/dt = T(V,t) - FD(V) -
        Frr(V) - m g sin(theta). The lift and drag depend on a quasi-steady
        CL computed from weight and speed.

        Parameters
        ----------
        V0 : float
            Initial ground speed (m/s).
        x0 : float
            Initial runway position (m).
        t_span : (t0, tf)
            Time integration span.
        V_rot : float or None
            Rotation speed threshold (m/s). If None, computed as 1.05*V_stall.
        t_eval : ndarray or None
            Times to sample the solution.

        Returns
        -------
        dict
            A summary dict containing the solver result and rotation event info.
        """
        m = self._plane.mass

        # compute stall and rotation speeds
        Vs = self._plane.stall_speed()
        if V_rot is None:
            V_rot = 1.05 * Vs

        # ODE system
        def f(t, s):
            x, V = s
            # avoid zero-inversion: enforce minimum V when computing CL
            V_for_CL = max(V, 0.1)
            # estimate CL required to support weight component normal to flight
            W = m * 9.80665
            CL = (2.0 * W) / (self._plane.rho * V_for_CL ** 2 * self._plane.wing_area)
            # aerodynamic forces
            FD = self._plane.drag(V_for_CL, CL)
            FL = self._plane.lift(V_for_CL, CL)
            Frr = self._mu_rr * (W - FL)
            T = self.thrust(V, t)
            dVdt = (T - FD - Frr - W * np.sin(self._theta)) / m
            dxdt = V
            return [dxdt, dVdt]

        # event to detect rotation speed
        def evt_rot(t, s):
            return s[1] - V_rot

        evt_rot.terminal = True
        evt_rot.direction = 1

        if t_eval is None:
            t_eval = np.linspace(t_span[0], t_span[1], 600)

        sol = solve_ivp(f, t_span, [x0, V0], events=[evt_rot], t_eval=t_eval, max_step=0.5)

        # record telemetry for the ground run
        for ti, xi, Vi in zip(sol.t, sol.y[0], sol.y[1]):
            self._record(ti, xi, 0.0, 0.0, Vi, 'ground_roll')

        rotation_reached = len(sol.t_events[0]) > 0
        t_rot = float(sol.t_events[0][0]) if rotation_reached else None
        x_rot = float(np.interp(t_rot, sol.t, sol.y[0])) if rotation_reached else None
        V_rot_actual = float(np.interp(t_rot, sol.t, sol.y[1])) if rotation_reached else None

        return {
            'solution': sol,
            'rotation_reached': rotation_reached,
            't_rot': t_rot,
            'x_rot': x_rot,
            'V_rot': V_rot_actual,
            'V_stall': Vs,
            'V_rot_threshold': V_rot,
        }

    # ------------------------------ Climb ----------------------------------
    def run_climb(self, state0: Tuple[float, float, float, float],
                  t_span: Tuple[float, float], t_eval: Optional[np.ndarray] = None,
                  gamma0: float = 5.0 * np.pi / 180.0, phi: float = 0.0) -> Dict:
        """
        Simulate a simplified climb phase using 2D kinematics and dynamics.

        state0 is (x0, z0, V0, psi0) representing initial position along runway
        (x), altitude z, speed V, and heading psi. The model integrates
        [x, z, V, psi] with equations provided earlier.

        Parameters and returns are analogous to other phase methods.
        """
        m = self._plane.mass
        x0, z0, V0, psi0 = state0

        def f(t, s):
            x, z, V, psi = s
            V_for_CL = max(V, 0.1)
            CL = (2.0 * m * 9.80665 * np.cos(gamma0)) / (self._plane.rho * V_for_CL ** 2 * self._plane.wing_area)
            D = self._plane.drag(V_for_CL, CL)
            T = self.thrust(V, t)
            dVdt = (T - D - m * 9.80665 * np.sin(gamma0)) / m
            dxdt = V * np.cos(gamma0) * np.cos(psi)
            dzdt = V * np.sin(gamma0)
            dpsidt = 9.80665 * np.tan(phi) / max(V, 1e-3)
            return [dxdt, dzdt, dVdt, dpsidt]

        if t_eval is None:
            t_eval = np.linspace(t_span[0], t_span[1], 400)

        sol = solve_ivp(f, t_span, [x0, z0, V0, psi0], t_eval=t_eval, max_step=1.0)

        # record telemetry for climb
        for ti, xi, zi, Vi in zip(sol.t, sol.y[0], sol.y[1], sol.y[2]):
            self._record(ti, xi, 0.0, zi, Vi, 'climb')

        return {'solution': sol}

    # ------------------------------ Cruise ---------------------------------
    def run_cruise(self, state0: Tuple[float, float, float, float],
                   t_span: Tuple[float, float], V_cruise: float,
                   t_eval: Optional[np.ndarray] = None) -> Dict:
        """
        Simulate a simple cruise where speed tends toward V_cruise and heading
        is approximately constant.
        """
        x0, z0, V0, psi0 = state0

        def f(t, s):
            x, z, V, psi = s
            k = 0.5
            dVdt = k * (V_cruise - V)
            gamma = 0.0
            dxdt = V * np.cos(gamma) * np.cos(psi)
            dzdt = V * np.sin(gamma)
            dpsidt = 0.0
            return [dxdt, dzdt, dVdt, dpsidt]

        if t_eval is None:
            t_eval = np.linspace(t_span[0], t_span[1], 400)

        sol = solve_ivp(f, t_span, [x0, z0, V0, psi0], t_eval=t_eval, max_step=5.0)

        for ti, xi, zi, Vi in zip(sol.t, sol.y[0], sol.y[1], sol.y[2]):
            self._record(ti, xi, 0.0, zi, Vi, 'cruise')

        return {'solution': sol}

    # -------------------------- Approach & Landing ---------------------------
    def run_approach_and_landing(self, state0: Tuple[float, float, float, float],
                                 t_span: Tuple[float, float], gamma: float = -5.5 * np.pi / 180.0,
                                 t_eval: Optional[np.ndarray] = None) -> Dict:
        """
        Simulate final approach until touchdown event: z == h_LG (approx).

        Returns touchdown time, position and speed in the summary dict.
        """
        m = self._plane.mass
        x0, z0, V0, psi0 = state0

        def f(t, s):
            x, z, V, psi = s
            V_for_CL = max(V, 0.1)
            CL = (2.0 * m * 9.80665 * np.cos(gamma)) / (self._plane.rho * V_for_CL ** 2 * self._plane.wing_area)
            D = self._plane.drag(V_for_CL, CL)
            T = self.thrust(V, t)
            dVdt = (T - D - m * 9.80665 * np.sin(gamma)) / m
            dxdt = V * np.cos(gamma) * np.cos(psi)
            dzdt = V * np.sin(gamma)
            dpsidt = 0.0
            return [dxdt, dzdt, dVdt, dpsidt]

        def evt_touchdown(t, s):
            return s[1] - self._h_LG

        evt_touchdown.terminal = True
        evt_touchdown.direction = -1

        if t_eval is None:
            t_eval = np.linspace(t_span[0], t_span[1], 400)

        sol = solve_ivp(f, t_span, [x0, z0, V0, psi0], events=[evt_touchdown], t_eval=t_eval, max_step=0.5)

        # record approach telemetry
        for ti, xi, zi, Vi in zip(sol.t, sol.y[0], sol.y[1], sol.y[2]):
            self._record(ti, xi, 0.0, zi, Vi, 'approach')

        touchdown = len(sol.t_events[0]) > 0
        t_touch = float(sol.t_events[0][0]) if touchdown else None
        x_touch = float(np.interp(t_touch, sol.t, sol.y[0])) if touchdown else None
        V_touch = float(np.interp(t_touch, sol.t, sol.y[2])) if touchdown else None

        return {
            'solution': sol,
            'touchdown': touchdown,
            't_touch': t_touch,
            'x_touch': x_touch,
            'V_touch': V_touch,
        }

    # ------------------------ Ground roll after landing ---------------------
    def run_ground_roll_landing(self, V0: float, x0: float = 0.0,
                                t_span: Tuple[float, float] = (0.0, 120.0),
                                t_eval: Optional[np.ndarray] = None,
                                beta: float = 1.0, mu_b: float = 0.5) -> Dict:
        """
        Simulate the rollout after touchdown using braking and aerodynamic
        deceleration until stop condition V < V_stop (~0.5 m/s).
        """
        m = self._plane.mass

        def f(t, s):
            x, V = s
            V_for_CL = max(V, 0.1)
            CL = (2.0 * m * 9.80665) / (self._plane.rho * V_for_CL ** 2 * self._plane.wing_area)
            FD = self._plane.drag(V_for_CL, CL)
            FL = self._plane.lift(V_for_CL, CL)
            Frr = self._mu_rr * (m * 9.80665 - FL)
            Fbrake = beta * mu_b * m * 9.80665
            Trev = 0.0
            dVdt = (-FD - Frr - Fbrake - m * 9.80665 * np.sin(self._theta) - Trev) / m
            dxdt = V
            return [dxdt, dVdt]

        def evt_stop(t, s):
            return s[1] - 0.5

        evt_stop.terminal = True
        evt_stop.direction = -1

        if t_eval is None:
            t_eval = np.linspace(t_span[0], t_span[1], 800)

        sol = solve_ivp(f, t_span, [x0, V0], events=[evt_stop], t_eval=t_eval, max_step=0.2)

        for ti, xi, Vi in zip(sol.t, sol.y[0], sol.y[1]):
            self._record(ti, xi, 0.0, 0.0, Vi, 'landing_roll')

        stopped = len(sol.t_events[0]) > 0
        t_stop = float(sol.t_events[0][0]) if stopped else None
        x_stop = float(np.interp(t_stop, sol.t, sol.y[0])) if stopped else None

        return {'solution': sol, 'stopped': stopped, 't_stop': t_stop, 'x_stop': x_stop}


# ------ GLOBAL FUNCTIONS --------------------------------------------------------------
# if the global functions can be moved into appropriate classes, or a new one (for encapsualtion purposes) this should be done


# ------ MAIN BODY --------------------------------------------------------------

if __name__ == '__main__':
    # test functions within here
    # add the logic according to the pseudocode shown below. This is just sample pseudocode and may need large changes - append all correct sections/classes:
    """

    // ---- Initialization ----
    SIM_START = Monday 06:30    // start of week simulation
    SIM_END   = SIM_START + 7 days
    CURRENT_TIME = SIM_START
    TIME_STEP = 1 minute       // simulation tick (can be smaller if needed)

    runway_busy = false
    last_landing_time = -∞

    // Queues and lists to track planes
    arrival_queue = empty queue   // planes waiting to land
    departure_queue = empty queue // planes waiting for takeoff (after taxi & pushback)
    planes_in_air = []   // list of planes currently approaching (outside airport)
    planes_on_ground = [] // list of planes taxiing or being serviced at gates

    // Function to log output at each step
    function log_status():
        print(CURRENT_TIME, 
            "ArrQ=", arrival_queue.size(), 
            "DepQ=", departure_queue.size(), 
            "InAir=", len(planes_in_air), 
            "OnGrd=", len(planes_on_ground), 
            "RunwayBusy=", runway_busy)

    // Pre-generate or schedule arrivals (optionally from data)
    next_arrival_time = SIM_START
    while next_arrival_time < SIM_END:
        // Schedule next arrival (e.g., Poisson with mean ~122/day)
        schedule at next_arrival_time:
            create plane with:
                plane.type = random_choice({EJet, A318, A220}) 
                plane.distance = R   // spawn at airspace boundary
                plane.speed = SPEED_BY_TYPE[plane.type]
                plane.status = "approaching"
            add this plane to planes_in_air
        // compute next arrival
        next_arrival_time += random_exponential(mean_interarrival)

    // (Similarly, schedule departures when flights complete service)

    // ---- Main simulation loop ----
    while CURRENT_TIME < SIM_END:
        // 1. Process approaching planes
        for each plane in planes_in_air:
            // Move plane closer: simple motion towards runway (ODE step)
            plane.distance -= plane.speed * TIME_STEP
            if plane.distance <= 0:
                // Arrived at runway threshold
                if runway_busy or CURRENT_TIME - last_landing_time < MIN_SEPARATION:
                    // Runway not available or spacing not met -> go-around
                    plane.distance = R    // send it back out to hold at boundary
                    // Optionally schedule next try: we just loop again
                else:
                    // Move plane to arrival queue to land
                    arrival_queue.enqueue(plane)
                    remove plane from planes_in_air

        // 2. Check runway for landing or takeoff
        if NOT runway_busy:
            if NOT arrival_queue.isEmpty() and planes_arrived_in_a_row <=5:
                // Land next arrival
                plane = arrival_queue.dequeue()
                plane.status = "landing"
                runway_busy = true
                last_landing_time = CURRENT_TIME
                // Schedule runway free event after landing duration
                runway_release_time = CURRENT_TIME + LANDING_TIME_BY_TYPE[plane.type]
                schedule event at runway_release_time:
                    runway_busy = false
                    // After landing, plane will taxi to gate
                    plane.status = "taxiing_to_gate"
                    add plane to planes_on_ground
            else if NOT departure_queue.isEmpty():
                // Depart next plane
                plane = departure_queue.dequeue()
                plane.status = "taking_off"
                runway_busy = true
                // Schedule runway free after takeoff duration
                runway_release_time = CURRENT_TIME + TAKEOFF_TIME
                schedule event at runway_release_time:
                    runway_busy = false
                    // After takeoff, remove plane from simulation
                    remove plane

        // 3. Taxi and gate handling
        for each plane in planes_on_ground:
            if plane.status == "taxiing_to_gate":
                // Immediately assign gate if free (assume short taxi time)
                if gate_available():
                    assign gate to plane
                    plane.status = "servicing"
                    // Initialize service timer
                    plane.service_time = random_service_time(plane.type)
                else:
                    // No gate free: plane holds on taxiway (or in apron queue)
                    continue
            else if plane.status == "servicing":
                // Decrement service time
                plane.service_time -= TIME_STEP
                if plane.service_time <= 0:
                    // Service complete
                    plane.status = "ready_for_departure"
                    // After pushback/taxi, enter departure queue
                    departure_queue.enqueue(plane)
                    remove plane from planes_on_ground

        // 4. Advance time and log status
        log_status()
        CURRENT_TIME += TIME_STEP

    """