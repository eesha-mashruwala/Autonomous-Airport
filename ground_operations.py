"""
Ground Operations Module

Handles all ground-based airport operations including:
- Runway scheduling and management
- Gate allocation and management
- Aircraft taxiing operations
- Queue management for takeoffs and landings

Designed for London City Airport (LCY) constraints.
"""

import random
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from typing import Optional, List, Dict, Tuple
from aircraft import Plane

# London City Airport runway dimensions
RUNWAY_LENGTH = 1508  # meters
RUNWAY_THRESHOLD = np.array([0.0, 0.0, 0.0])
RUNWAY_END = np.array([RUNWAY_LENGTH, 0.0, 0.0])
AIRSPACE_RADIUS = 5500  # meters


def generate_gate_positions(num_gates: int = 19, runway_length: float = RUNWAY_LENGTH) -> np.ndarray:
    """
    Generate gate positions alongside the runway.
    Gates are positioned on both sides of the runway with even spacing.
    
    Parameters
    ----------
    num_gates : int
        Number of gates to generate (default 19)
    runway_length : float
        Length of runway in meters
        
    Returns
    -------
    np.ndarray
        Array of shape (num_gates, 3) with gate positions [x, y, z]
    """
    gates = []
    gate_spacing = runway_length / (num_gates // 2 + 1)
    lateral_offset = 80  # meters from runway centerline
    
    # Distribute gates on both sides of the runway
    for i in range(num_gates):
        # Alternate sides: even indices on north (+y), odd indices on south (-y)
        side = 1 if i % 2 == 0 else -1
        
        # Position along runway (staggered)
        x_position = gate_spacing * (i // 2 + 1)
        y_position = side * lateral_offset
        z_position = 0.0
        
        gates.append([x_position, y_position, z_position])
    
    return np.array(gates)


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
            Runway slope angle (theta) in radians.S
        """
        self.runway_length = runway_length
        self.runway_slope = runway_slope
        self.num_gates = num_gates

        # gates: None if empty, or plane object assigned
        self.gates = [None] * num_gates
        
        # Generate gate positions
        self.gate_positions = generate_gate_positions(num_gates, runway_length)

        # state flags
        self.runway_busy = False  
        self.runway_free_time = 0.0  # when the runway becomes free
        self.current_runway_plane = None  # which plane is currently using runway

        # queues
        self.takeoff_queue = []
        self.landing_queue = []
        self.taxiing_planes = []
        
        # Landing clearance tracking - maps plane to clearance status
        self.landing_clearances = {}  # {plane: {'cleared': bool, 'time': float}}

        # Track inbound arrival ETAs and cached takeoff durations
        self.inbound_arrivals: Dict[Plane, Dict[str, float]] = {}
        self.takeoff_time_cache: Dict[str, float] = {}

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
        self.runway_free_time = 0.0

    # ------ ARRIVAL / DEPARTURE CONFLICT MANAGEMENT ------------------------

    def update_arrival_eta(self, plane: Plane, eta_to_threshold: float, current_time: float):
        """
        Register or refresh the estimated arrival time to the runway threshold.
        """
        if eta_to_threshold is None:
            return
        self.inbound_arrivals[plane] = {
            "eta": current_time + eta_to_threshold,
            "updated": current_time,
        }

    def clear_arrival_eta(self, plane: Plane):
        """Remove a plane from inbound tracking (used after landing)."""
        if plane in self.inbound_arrivals:
            del self.inbound_arrivals[plane]

    def _next_arrival_eta(self, current_time: float) -> Optional[float]:
        """Return soonest arrival ETA (absolute time) and prune stale entries."""
        stale_cutoff = current_time - 180.0
        for plane, data in list(self.inbound_arrivals.items()):
            if data["eta"] < current_time - 10.0 or data["updated"] < stale_cutoff:
                del self.inbound_arrivals[plane]
        if not self.inbound_arrivals:
            return None
        return min(data["eta"] for data in self.inbound_arrivals.values())

    def has_arrival_conflict(self, current_time: float, required_window: float, buffer: float = 5.0) -> bool:
        """
        Determine if an arrival will reach the threshold before a departure can
        finish its runway use.
        """
        next_eta = self._next_arrival_eta(current_time)
        if next_eta is None:
            return False
        return next_eta <= current_time + required_window + buffer

    def get_next_arrival_eta(self, current_time: float) -> Optional[float]:
        """Public helper to expose the soonest inbound ETA for logging/UI."""
        return self._next_arrival_eta(current_time)

    def estimate_takeoff_runway_time(self, plane: Plane) -> float:
        """
        Estimate how long this aircraft type will occupy the runway for takeoff.
        Uses a cached ground-roll simulation plus a buffer to clear the surface.
        """
        plane_type = getattr(plane, "plane_type", plane.__class__.__name__)
        if plane_type in self.takeoff_time_cache:
            return self.takeoff_time_cache[plane_type]

        # Default if dynamics fail for any reason
        estimated_time = 35.0
        try:
            from flight_dynamics import FlightDynamics  # local import to avoid cycle

            dyn = FlightDynamics(plane)
            result = dyn.ground_roll_takeoff(dt=0.05, Vr=70.0, max_time=120.0)
            roll_time = result.get("time", estimated_time)
            estimated_time = max(roll_time, 5.0) + 7.0  # add buffer to clear runway
        except Exception:
            estimated_time = 35.0

        self.takeoff_time_cache[plane_type] = estimated_time
        return estimated_time

    # ------ LANDING CLEARANCE LOGIC (ATC) ----------------------------------------------
    
    def request_landing_clearance(self, plane, current_time: float, on_final_approach: bool = False) -> bool:
        """
        Request landing clearance for an aircraft.
        
        When an aircraft is on final approach (between last waypoint and runway),
        it should be granted clearance and the runway marked busy immediately
        to prevent conflicts.
        
        Parameters
        ----------
        plane : Plane
            Aircraft requesting clearance
        current_time : float
            Current simulation time
        on_final_approach : bool
            True if aircraft has reached the second-to-last waypoint (final approach)
            
        Returns
        -------
        bool
            True if clearance granted, False otherwise
        """
        # Check if already cleared
        if plane in self.landing_clearances and self.landing_clearances[plane]['cleared']:
            return True
        
        # If on final approach and runway is available, grant immediate clearance
        if on_final_approach:
            if self.is_runway_available(current_time):
                # Mark runway busy for the landing operation
                # Estimate 60 seconds for final approach + touchdown + rollout + vacate
                landing_duration = 60.0
                self.mark_runway_busy(plane, landing_duration, current_time)
                
                # Grant clearance
                self.landing_clearances[plane] = {
                    'cleared': True,
                    'time': current_time
                }
                return True
            else:
                # Runway busy - must hold (shouldn't happen with proper sequencing)
                return False
        
        return False
    
    def is_cleared_to_land(self, plane) -> bool:
        """
        Check if a plane has landing clearance.
        
        Parameters
        ----------
        plane : Plane
            Aircraft to check
            
        Returns
        -------
        bool
            True if cleared to land
        """
        return plane in self.landing_clearances and self.landing_clearances[plane]['cleared']
    
    def revoke_landing_clearance(self, plane):
        """
        Revoke landing clearance (e.g., go-around scenario).
        
        Parameters
        ----------
        plane : Plane
            Aircraft whose clearance to revoke
        """
        if plane in self.landing_clearances:
            del self.landing_clearances[plane]
    
    def complete_landing(self, plane):
        """
        Called when aircraft has completed landing and vacated runway.
        Clears the landing clearance record.
        
        Parameters
        ----------
        plane : Plane
            Aircraft that completed landing
        """
        if plane in self.landing_clearances:
            del self.landing_clearances[plane]


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
    
    def allocate_gate(self, plane, reference_position: Optional[np.ndarray] = None) -> Optional[int]:
        """
        Assign a plane to the nearest available gate.
        
        Parameters
        ----------
        plane : Plane
            Aircraft requesting a gate
        reference_position : np.ndarray, optional
            Position to measure distance from (defaults to runway threshold)
        """
        free_gates = self.get_free_gates()
        if not free_gates:
            return None

        if reference_position is None:
            reference_position = RUNWAY_THRESHOLD

        ref = np.array(reference_position, dtype=float)
        nearest_gate = min(
            free_gates,
            key=lambda idx: np.linalg.norm(self.gate_positions[idx] - ref)
        )
        self.gates[nearest_gate] = plane
        plane.assigned_gate = nearest_gate
        return nearest_gate

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
    
    def get_aircraft_position(self, plane: Plane) -> Optional[np.ndarray]:
        """
        Get the 3D position of an aircraft based on its current state.
        
        Parameters
        ----------
        plane : Plane
            The aircraft to locate
            
        Returns
        -------
        np.ndarray or None
            Position [x, y, z] or None if aircraft not on ground
        """
        if hasattr(plane, 'assigned_gate') and plane.assigned_gate is not None:
            return self.gate_positions[plane.assigned_gate].copy()
        return None
    
    def get_all_parked_aircraft(self) -> List[Tuple[int, Plane, np.ndarray]]:
        """
        Get all aircraft currently parked at gates.
        
        Returns
        -------
        List of (gate_number, plane, position) tuples
        """
        parked = []
        for gate_num, plane in enumerate(self.gates):
            if plane is not None:
                position = self.gate_positions[gate_num]
                parked.append((gate_num, plane, position))
        return parked
    
    def land_aircraft_to_gate(self, plane: Plane) -> Optional[int]:
        """
        Instantly assign a landed aircraft to an available gate.
        Simulates taxiing without animation - aircraft just appears at gate.
        
        Parameters
        ----------
        plane : Plane
            The aircraft that has landed
            
        Returns
        -------
        int or None
            Gate number assigned, or None if no gates available
        """
        gate = self.allocate_gate(plane)
        if gate is not None:
            plane.state = "at_gate"
            plane.assigned_gate = gate
            print(f"  ✈ {plane.plane_type} ({plane.id}) landed → assigned to Gate {gate+1}")
        else:
            print(f"  ⚠ {plane.plane_type} ({plane.id}) landed → NO GATES AVAILABLE")
        return gate
    
    def depart_aircraft_from_gate(self, gate_number: int) -> Optional[Plane]:
        """
        Remove an aircraft from a gate for departure.
        
        Parameters
        ----------
        gate_number : int
            The gate number to clear
            
        Returns
        -------
        Plane or None
            The aircraft that departed, or None if gate was empty
        """
        if 0 <= gate_number < self.num_gates:
            plane = self.gates[gate_number]
            if plane is not None:
                self.release_gate(gate_number)
                plane.state = "departing"
                print(f"  ✈ {plane.plane_type} ({plane.id}) departing from Gate {gate_number+1}")
                return plane
        return None


# ------ VISUALIZATION FUNCTION --------------------------------------------------------------
def visualize_ground_operations_live(ground_ops: GroundOperations, 
                                     duration: float = 60.0,
                                     show_arrivals: bool = True,
                                     show_departures: bool = True):
    """
    Create a live 3D visualization of ground operations showing:
    - Runway (1508m)
    - 19 gate positions alongside runway
    - Aircraft at gates (as colored dots)
    - Real-time updates as aircraft land and depart
    
    Parameters
    ----------
    ground_ops : GroundOperations
        The ground operations system to visualize
    duration : float
        Simulation duration in seconds
    show_arrivals : bool
        Whether to show arrival routes (from arrival_live_sim style)
    show_departures : bool
        Whether to show departure routes (from departure_live_sim style)
    """
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Draw runway
    ax.plot([RUNWAY_THRESHOLD[0], RUNWAY_END[0]],
            [RUNWAY_THRESHOLD[1], RUNWAY_END[1]],
            [RUNWAY_THRESHOLD[2], RUNWAY_END[2]],
            color='gray', linewidth=10, label='Runway (1508m)', alpha=0.8, zorder=1)
    
    # Draw gate positions
    gate_positions = ground_ops.gate_positions
    ax.scatter(gate_positions[:, 0], gate_positions[:, 1], gate_positions[:, 2],
               c='lightblue', s=100, marker='s', alpha=0.6, label='Gates (19)', zorder=2)
    
    # Label each gate
    for i, pos in enumerate(gate_positions):
        ax.text(pos[0], pos[1], pos[2] + 20, f'G{i+1}', 
                fontsize=7, ha='center', color='darkblue', alpha=0.7)
    
    # Initialize aircraft markers (will be updated in animation)
    aircraft_scatter = ax.scatter([], [], [], c='red', s=150, marker='o', 
                                  label='Aircraft', zorder=3)
    
    # Set up 3D view
    ax.set_xlim([-200, RUNWAY_LENGTH + 200])
    ax.set_ylim([-200, 200])
    ax.set_zlim([0, 300])
    ax.set_xlabel('X (m)', fontsize=11)
    ax.set_ylabel('Y (m)', fontsize=11)
    ax.set_zlabel('Altitude (m)', fontsize=11)
    ax.set_title('Ground Operations - London City Airport\n19 Gates | 1508m Runway', 
                 fontsize=14, fontweight='bold')
    ax.legend(loc='upper left', fontsize=10)
    ax.grid(True, alpha=0.3)
    
    # Set viewing angle
    ax.view_init(elev=30, azim=-60)
    
    # Animation update function
    time_counter = [0.0]
    
    def update(frame):
        # Update time
        time_counter[0] += 0.1
        current_time = time_counter[0]
        
        # Get all parked aircraft
        parked = ground_ops.get_all_parked_aircraft()
        
        if len(parked) > 0:
            positions = np.array([pos for _, _, pos in parked])
            aircraft_scatter._offsets3d = (positions[:, 0], 
                                          positions[:, 1], 
                                          positions[:, 2] + 5)  # Slightly elevated for visibility
        else:
            aircraft_scatter._offsets3d = ([], [], [])
        
        # Update title with time and stats
        ax.set_title(f'Ground Operations - London City Airport | Time: {current_time:.1f}s\n'
                    f'Gates Occupied: {len(parked)}/19 | Runway: {"BUSY" if ground_ops.runway_busy else "FREE"}',
                    fontsize=14, fontweight='bold')
        
        return aircraft_scatter,
    
    # Create animation
    frames = int(duration / 0.1)
    ani = animation.FuncAnimation(fig, update, frames=frames, 
                                 interval=100, blit=False, repeat=False)
    
    plt.tight_layout()
    return fig, ani


# ------ DEMO/TEST FUNCTION --------------------------------------------------------------
if __name__ == "__main__":
    """
    Demo: Show ground operations with gates and sample aircraft.
    Simulates arrivals landing and being assigned to gates dynamically.
    """
    from aircraft import EmbraerE170, AirbusA220, Dash8_400, ATR72_600
    
    print("="*70)
    print("GROUND OPERATIONS VISUALIZATION DEMO")
    print("="*70)
    print(f"Runway: {RUNWAY_LENGTH}m")
    print(f"Gates: 19 positions alongside runway")
    print("="*70)
    
    # Create ground operations system
    ground_ops = GroundOperations(runway_length=RUNWAY_LENGTH, num_gates=19)
    
    # Add some initial aircraft to gates
    aircraft_types = [EmbraerE170, AirbusA220, Dash8_400, ATR72_600]
    
    print("\nInitial aircraft at gates:")
    for i in range(5):  # Start with 5 aircraft
        plane_cls = random.choice(aircraft_types)
        plane_id = f"{plane_cls.__name__}-{random.randint(100, 999)}"
        plane = plane_cls(plane_id)
        
        gate = ground_ops.allocate_gate(plane)
        if gate is not None:
            plane.state = "at_gate"
            plane.assigned_gate = gate
            position = ground_ops.gate_positions[gate]
            print(f"  ✓ {plane.plane_type} ({plane.id}) → Gate {gate+1}")
    
    # Schedule some arrivals
    arrival_schedule = []
    for i in range(6):
        time = (i + 1) * 5.0  # Every 5 seconds
        plane_cls = random.choice(aircraft_types)
        plane_id = f"{plane_cls.__name__}-{random.randint(100, 999)}"
        plane = plane_cls(plane_id)
        arrival_schedule.append((time, plane))
    
    print(f"\nScheduled arrivals: {len(arrival_schedule)} aircraft")
    
    # Schedule some departures
    departure_schedule = []
    for i in range(3):
        time = (i + 1) * 10.0  # Every 10 seconds
        gate_num = i * 2  # Depart from gates 0, 2, 4
        departure_schedule.append((time, gate_num))
    
    print(f"Scheduled departures: {len(departure_schedule)} aircraft\n")
    
    print("="*70)
    print("Starting live visualization...")
    print("Watch aircraft land and be assigned to gates!")
    print("Close the plot window to end the simulation.")
    print("="*70 + "\n")
    
    # Create enhanced animation with scheduled events
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Draw runway
    ax.plot([RUNWAY_THRESHOLD[0], RUNWAY_END[0]],
            [RUNWAY_THRESHOLD[1], RUNWAY_END[1]],
            [RUNWAY_THRESHOLD[2], RUNWAY_END[2]],
            color='gray', linewidth=10, label='Runway (1508m)', alpha=0.8, zorder=1)
    
    # Draw gate positions
    gate_positions = ground_ops.gate_positions
    ax.scatter(gate_positions[:, 0], gate_positions[:, 1], gate_positions[:, 2],
               c='lightblue', s=100, marker='s', alpha=0.6, label='Gates (19)', zorder=2)
    
    # Label each gate
    for i, pos in enumerate(gate_positions):
        ax.text(pos[0], pos[1], pos[2] + 20, f'G{i+1}', 
                fontsize=7, ha='center', color='darkblue', alpha=0.7)
    
    # Initialize aircraft markers
    aircraft_scatter = ax.scatter([], [], [], c='red', s=150, marker='o', 
                                  label='Aircraft', zorder=3)
    
    # Set up 3D view
    ax.set_xlim([-200, RUNWAY_LENGTH + 200])
    ax.set_ylim([-200, 200])
    ax.set_zlim([0, 300])
    ax.set_xlabel('X (m)', fontsize=11)
    ax.set_ylabel('Y (m)', fontsize=11)
    ax.set_zlabel('Altitude (m)', fontsize=11)
    ax.legend(loc='upper left', fontsize=10)
    ax.grid(True, alpha=0.3)
    ax.view_init(elev=30, azim=-60)
    
    # Animation state
    time_counter = [0.0]
    arrival_index = [0]
    departure_index = [0]
    
    def update(frame):
        # Update time
        time_counter[0] += 0.1
        current_time = time_counter[0]
        
        # Process scheduled arrivals
        while (arrival_index[0] < len(arrival_schedule) and 
               arrival_schedule[arrival_index[0]][0] <= current_time):
            _, plane = arrival_schedule[arrival_index[0]]
            ground_ops.land_aircraft_to_gate(plane)
            arrival_index[0] += 1
        
        # Process scheduled departures
        while (departure_index[0] < len(departure_schedule) and 
               departure_schedule[departure_index[0]][0] <= current_time):
            _, gate_num = departure_schedule[departure_index[0]]
            ground_ops.depart_aircraft_from_gate(gate_num)
            departure_index[0] += 1
        
        # Get all parked aircraft
        parked = ground_ops.get_all_parked_aircraft()
        
        if len(parked) > 0:
            positions = np.array([pos for _, _, pos in parked])
            aircraft_scatter._offsets3d = (positions[:, 0], 
                                          positions[:, 1], 
                                          positions[:, 2] + 5)
        else:
            aircraft_scatter._offsets3d = ([], [], [])
        
        # Update title
        arrivals_remaining = len(arrival_schedule) - arrival_index[0]
        departures_remaining = len(departure_schedule) - departure_index[0]
        ax.set_title(f'Ground Operations - London City Airport | Time: {current_time:.1f}s\n'
                    f'Gates Occupied: {len(parked)}/19 | Pending: {arrivals_remaining} arrivals, {departures_remaining} departures',
                    fontsize=13, fontweight='bold')
        
        return aircraft_scatter,
    
    # Create animation (50 seconds)
    frames = 500
    ani = animation.FuncAnimation(fig, update, frames=frames, 
                                 interval=100, blit=False, repeat=False)
    
    plt.tight_layout()
    plt.show()
    
    print("\n" + "="*70)
    print("DEMO COMPLETE")
    print(f"Final gate occupancy: {len(ground_ops.get_all_parked_aircraft())}/19")
    print("="*70)
