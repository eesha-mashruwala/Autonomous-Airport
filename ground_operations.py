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
from typing import Optional, List, Dict
from aircraft import Plane


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
