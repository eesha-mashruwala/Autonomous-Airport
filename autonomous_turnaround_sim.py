"""
Single-screen autonomous airport turnaround demo.

Flow:
1. Spawn one aircraft on a random arrival route (arrival_live_sim ArrivalSimulator).
2. Require the runway to be free before clearing to land; block it for rollout.
3. After rollout, snap the aircraft to a free gate (ground_operations visuals).
4. Apply a random gate service time (20-35s), then a taxi-to-runway timer.
5. Spawn at the runway threshold when taxi completes, wait for runway clearance,
   then run the live departure simulator to take off and climb out.

All phases are shown on one 3D plot with console logs for each transition.
"""

import argparse
import json
import math
import random
from typing import Optional
import numpy as np
import matplotlib
matplotlib.use('TkAgg')  # Force interactive backend
import matplotlib.pyplot as plt
from matplotlib import animation

from arrival_live_sim import (
    ArrivalSimulator,
    DT as ARRIVAL_DT,
    CRUISE_SPEED,
    APPROACH_SPEED,
    FINAL_SPEED,
    HEADING_GAIN,
    GAMMA_GAIN,
    MAX_BANK_RAD,
    V_THROTTLE_GAIN,
    clamp,
)
from arrival_routes import (
    generate_arrival_routes,
    RUNWAY_THRESHOLD,
    RUNWAY_END,
    AIRSPACE_RADIUS,
)
from departure_live_sim import DepartureSimulator, ANIMATION_DT as DEPARTURE_DT
from departure_routes import generate_departure_routes
from ground_operations import GroundOperations, RUNWAY_LENGTH
from aircraft import EmbraerE170, AirbusA220, Dash8_400, ATR72_600
from flight_dynamics import WindModel, FlightDynamics

# Service time bounds (seconds)
SERVICE_TIME_RANGE = (20, 35)
EMERGENCY_SERVICE_TIME_RANGE = (40, 60)  # Emergency aircraft need longer service time

# Fleet to sample from
FLEET = [EmbraerE170, AirbusA220, Dash8_400, ATR72_600]

# Temporary forced taxi duration (debugging convenience)
FORCED_TAXI_TIME = 10.0
FINAL_APPROACH_TARGET = 65.0
INTER_ARRIVAL_MEAN = 60.0  # seconds between new arrivals (mean)
MIN_SPAWN_INTERVAL = 20.0   # seconds, safety lower bound
MAX_ACTIVE_AIRCRAFT = 4
GLOBAL_DT = DEPARTURE_DT  # base time step for orchestration
HOLD_ALTITUDE_THRESHOLD = 200.0  # meters
LANDING_CLEAR_BUFFER = 10.0  # seconds runway occupancy buffer for arrival ahead
ARRIVAL_RUNWAY_CLEARANCE_TIME = 25.0  # seconds to clear runway after touchdown
EMERGENCY_MIN_LEAD_TIME = 5.0  # seconds runway must be free before emergency arrives
EMERGENCY_CLEARANCE_BUFFER = 15.0  # extra time emergency needs after threshold crossing


class AircraftInfoWindow:
    """Separate matplotlib window for each aircraft's detailed telemetry."""

    def __init__(self, plane_id: str, enabled: bool = True):
        self.plane_id = plane_id
        self.enabled = enabled
        self.lines = []
        self.max_lines = 50
        self.update_counter = 0
        
        # Commented out to prevent opening telemetry windows per aircraft
        # if self.enabled:
        #     self.fig = plt.figure(figsize=(10, 7))
        #     self.fig.canvas.manager.set_window_title(f"Aircraft {plane_id} - Telemetry")
        #     self.ax = self.fig.add_subplot(111)
        #     self.ax.axis('off')
        #     self.text = self.ax.text(0.05, 0.95, "", transform=self.ax.transAxes,
        #                              fontfamily='monospace', fontsize=9,
        #                              verticalalignment='top', wrap=True)
        #     plt.ion()
        #     self.fig.show()
        # else:
        #     self.fig = None
        #     self.ax = None
        #     self.text = None
        
        # Always set to None to prevent window creation
        self.fig = None
        self.ax = None
        self.text = None

    def add_line(self, line: str):
        """Add a new line to the info window."""
        self.lines.append(line)
        if len(self.lines) > self.max_lines:
            self.lines.pop(0)
        
        # Commented out to prevent updating telemetry windows
        # if self.enabled:
        #     self.update_counter += 1
        #     if self.update_counter % 10 == 0:
        #         self.text.set_text('\n'.join(self.lines))
        #         self.fig.canvas.draw_idle()
        #     # self.fig.canvas.flush_events()

    def close(self):
        """Close the window."""
        # Commented out to prevent closing telemetry windows (none are created)
        # if self.enabled and self.fig is not None:
        #     plt.close(self.fig)
        pass


class ATCConsoleWindow:
    """Dedicated matplotlib window for ATC/terminal updates."""

    def __init__(self, enabled: bool = True, title: str = "ATC Control"):
        self.enabled = enabled
        self.lines: list[str] = []
        self.max_lines = 120
        if self.enabled:
            self.fig = plt.figure(figsize=(8, 6))
            self.fig.canvas.manager.set_window_title(title)
            self.ax = self.fig.add_subplot(111)
            self.ax.axis("off")
            self.text = self.ax.text(
                0.02,
                0.98,
                "",
                transform=self.ax.transAxes,
                fontfamily="monospace",
                fontsize=9,
                verticalalignment="top",
                wrap=True,
            )
            plt.ion()
            self.fig.show()
        else:
            self.fig = None
            self.ax = None
            self.text = None

    def add_line(self, line: str):
        self.lines.append(line)
        if len(self.lines) > self.max_lines:
            self.lines.pop(0)

        if self.enabled and self.text is not None:
            self.text.set_text("\n".join(self.lines))
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()

    def close(self):
        if self.enabled and self.fig is not None:
            plt.close(self.fig)


class TerminalFeed:
    """Manages terminal output for status messages only."""

    def __init__(self, console_window: Optional[ATCConsoleWindow] = None):
        self.banner_printed = False
        self.console_window = console_window

    def _emit(self, prefix: str, text: str):
        line = f"{prefix} | {text}"
        print(line, flush=True)
        if self.console_window is not None:
            self.console_window.add_line(line)

    def announce_clearance(self, text: str):
        self._emit("ATC", text)

    def runway_status(self, busy: bool, reason: str):
        state = "BUSY" if busy else "FREE"
        self._emit("ATC", f"Runway {state} → {reason}")

    def gate_event(self, text: str):
        self._emit("GATE", text)

    def status_update(self, text: str):
        """General status updates for terminal."""
        self._emit("STATUS", text)


class SingleAircraftTurnaround:
    """State machine for one aircraft from arrival through departure."""

    def __init__(
        self,
        ground_ops: GroundOperations,
        feed: TerminalFeed,
        wind_model: WindModel,
        visual_mode: bool = True,
        plane=None,
        arrival_route=None,
        departure_route=None,
        is_emergency: bool = False,
        arrival_sim_cls=ArrivalSimulator,
        arrival_sim_kwargs: Optional[dict] = None,
        info_window_enabled: bool = True,
    ):
        self.arrival_route = arrival_route or random.choice(generate_arrival_routes(num_routes=20))
        self.departure_route = departure_route or random.choice(generate_departure_routes())
        if plane is None:
            plane_cls = random.choice(FLEET)
            plane_id = f"{plane_cls.__name__}-{random.randint(100, 999)}"
            plane = plane_cls(plane_id)
        self.plane = plane
        self.is_emergency = is_emergency

        self.ground_ops = ground_ops
        self.wind_model = wind_model
        self.arrival_sim = arrival_sim_cls(
            self.arrival_route,
            self.plane,
            verbose=False,
            wind_model=self.wind_model,
            **(arrival_sim_kwargs or {}),
        )
        self.departure_sim = None

        self.state = "arrival"  # arrival -> gate_assignment -> at_gate -> taxi_runway -> runway_hold -> departure -> complete
        self.position = None
        self.trail = []
        self.sim_time = 0.0
        self.frame_dt = ARRIVAL_DT
        self.feed = feed
        info_enabled = visual_mode and info_window_enabled
        self.info_window = AircraftInfoWindow(self.plane.id, enabled=info_enabled)

        self.landing_busy_marked = False
        self.final_speed_logged = False
        self.runway_hold_logged = False
        self.gate_wait_logged = False
        self.runway_hold_depart_logged = False
        self.arrival_conflict_logged = False
        self.gate_index = None
        self.vacate_end_time: Optional[float] = None
        self.taxi_to_gate_end_time: Optional[float] = None
        self.takeoff_runway_released = False
        self.runway_exit_position: Optional[np.ndarray] = None
        self.arrival_hold_active = False

        self.service_time = None
        self.service_end_time = None
        self.taxi_end_time = None
        self.taxi_progress_reported = False

        self.frame_interval_ms = 100

        self._log_intro()

    # ------------------------------------------------------------------
    # Logging helpers
    # ------------------------------------------------------------------
    def _log_intro(self):
        if not self.feed.banner_printed:
            print("\n" + "=" * 70)
            print("AUTONOMOUS MULTI-AIRCRAFT TURNAROUND SIMULATION")
            print("=" * 70)
            print("Each aircraft has its own telemetry window.")
            print("Terminal shows overall status updates only.")
            print("=" * 70)
            self.feed.banner_printed = True
        
        # Log to aircraft's info window
        self.info_window.add_line(f"{'='*60}")
        self.info_window.add_line(f"AIRCRAFT: {self.plane.plane_type} ({self.plane.id})")
        self.info_window.add_line(f"{'='*60}")
        self.info_window.add_line(f"Arrival: {self.arrival_route['name']} ({self.arrival_route['description']})")
        self.info_window.add_line(f"  Waypoints: {len(self.arrival_route['waypoints'])}")
        self.info_window.add_line(f"Departure: {self.departure_route['name']} (heading {self.departure_route['heading']}°)")
        self.info_window.add_line(f"  Waypoints: {len(self.departure_route['waypoints'])}")
        self.info_window.add_line(f"{'='*60}")
        self.info_window.add_line("")
        
        # Terminal status
        status_msg = f"{self.plane.id} spawned on {self.arrival_route['name']}"
        if self.is_emergency:
            status_msg = f"EMERGENCY {status_msg}"
        self.feed.status_update(status_msg)

    def _log(self, message: str):
        print(f"[t={self.sim_time:5.1f}s] {message}")

    def set_arrival_hold(self, active: bool, estimated_hold_time: float = 60.0):
        """Toggle holding pattern if the arrival simulator supports it."""
        if hasattr(self.arrival_sim, "set_hold"):
            self.arrival_sim.set_hold(active, estimated_hold_time)
        self.arrival_hold_active = bool(active)
    
    def _save_trajectory_to_json(self):
        """Save the aircraft's trajectory data to a JSON file."""
        trajectory_data = {
            "plane_id": self.plane.id,
            "plane_type": self.plane.plane_type,
            "arrival_route": self.arrival_route["name"],
            "is_emergency": self.is_emergency,
            "coordinates": [[float(pos[0]), float(pos[1]), float(pos[2])] for pos in self.trail]
        }
        
        filename = f"trajectory_{self.plane.id}.json"
        try:
            with open(filename, 'w') as f:
                json.dump(trajectory_data, f, indent=2)
            self._log(f"Trajectory saved to {filename}")
        except Exception as e:
            self._log(f"Failed to save trajectory: {e}")

    # ------------------------------------------------------------------
    # State machine
    # ------------------------------------------------------------------
    def step(self, global_time: float):
        """
        Advance the simulation by one animation frame.
        Returns the current 3D position of the aircraft (or None if hidden).
        """
        # Align local clock to shared orchestrator time for coordination
        self.sim_time = global_time
        # Dynamic timestep: arrivals use their native DT, other phases use departure DT
        self.frame_dt = ARRIVAL_DT if self.state in ("arrival", "gate_assignment") else DEPARTURE_DT
        if self.state != "arrival":
            self.arrival_hold_active = False

        # Free runway when its busy window expires
        if getattr(self.ground_ops, "runway_busy", False) and self.sim_time >= self.ground_ops.runway_free_time:
            self.ground_ops.release_runway()
            self.feed.runway_status(False, "timer elapsed")
            self._log("Runway now FREE (timer elapsed)")

        if self.state == "arrival":
            return self._advance_arrival()
        if self.state == "vacating_runway":
            return self._vacate_runway()
        if self.state == "gate_assignment":
            return self._assign_gate()
        if self.state == "taxi_to_gate":
            return self._taxi_from_runway_to_gate()
        if self.state == "at_gate":
            return self._service_at_gate()
        if self.state == "taxi_runway":
            return self._taxi_to_runway()
        if self.state == "runway_hold":
            if getattr(self.ground_ops, "emergency_active", False) and not self.is_emergency:
                if not self.runway_hold_depart_logged:
                    self.info_window.add_line("Runway HOLD - emergency traffic")
                    self.feed.status_update(f"{self.plane.id} holding for emergency traffic")
                    self._log("Holding short due to emergency traffic")
                    self.runway_hold_depart_logged = True
                return self.position

            return self._runway_queue()
        if self.state == "departure":
            return self._advance_departure()
        return self.position

    def _advance_arrival(self):
        """Move along the arrival route while respecting runway availability."""
        try:
            pos = next(self.arrival_sim)
            self.position = pos.copy()
            self.trail.append(pos.copy())

            # Continuously publish ETA to the threshold so departures can defer
            eta_to_threshold = self.arrival_sim.estimate_time_to_threshold()
            self.ground_ops.update_arrival_eta(self.plane, eta_to_threshold, self.sim_time)

            if not self.arrival_sim.landed:
                # Log detailed telemetry to aircraft's window
                total = len(self.arrival_sim.route["waypoints"]) - 1
                seg = min(self.arrival_sim.segment + 1, total)
                target_idx = min(self.arrival_sim.segment + 1, total)
                target = self.arrival_sim.route["waypoints"][target_idx]
                
                # Check if holding
                is_holding = hasattr(self.arrival_sim, 'hold_active') and self.arrival_sim.hold_active
                
                if is_holding:
                    self.info_window.add_line(
                        f"ARR | HOLDING | "
                        f"Alt {pos[2]:6.1f}m | "
                        f"HDG {math.degrees(self.arrival_sim.psi):5.1f}° | "
                        f"V {self.arrival_sim.V:5.1f} m/s"
                    )
                else:
                    self.info_window.add_line(
                        f"ARR | "
                        f"Pos ({pos[0]:7.1f}, {pos[1]:7.1f}, {pos[2]:6.1f}) | "
                        f"HDG {math.degrees(self.arrival_sim.psi):5.1f}° | "
                        f"V {self.arrival_sim.V:5.1f} m/s"
                    )
            else:
                # Aircraft has landed - showing rollout telemetry
                if not self.final_speed_logged:
                    self.info_window.add_line(f">>> TOUCHDOWN <<<")
                    self.info_window.add_line(f"Touchdown speed: {self.arrival_sim.V:.1f} m/s")
                    self.info_window.add_line(f"Beginning landing rollout...")
                    self.final_speed_logged = True
                
                # Log rollout progress
                distance_covered = pos[0] - RUNWAY_THRESHOLD[0]
                rollout_speed = getattr(self.arrival_sim, "current_rollout_speed", 0.0)
                self.info_window.add_line(
                    f"ROLLOUT | V={rollout_speed:5.1f} m/s | "
                    f"Distance: {distance_covered:6.1f}m | "
                    f"Pos ({pos[0]:7.1f}, {pos[1]:7.1f}, {pos[2]:6.1f})"
                )

            # Check if aircraft is on final approach (last 2 segments before runway)
            on_final_approach = self.arrival_sim.segment >= len(self.arrival_sim.wps) - 2

            landing_window = None
            if eta_to_threshold is not None:
                try:
                    landing_window = max(
                        ARRIVAL_RUNWAY_CLEARANCE_TIME,
                        eta_to_threshold + ARRIVAL_RUNWAY_CLEARANCE_TIME,
                    )
                except Exception:
                    landing_window = None
            
            # Request landing clearance when on final approach
            if on_final_approach and not self.landing_busy_marked:
                clearance_granted = self.ground_ops.request_landing_clearance(
                    self.plane,
                    self.sim_time,
                    on_final_approach=True,
                    landing_window=landing_window,
                )
                
                if clearance_granted:
                    self.landing_busy_marked = True
                    self.info_window.add_line(">>> ATC: CLEARED TO LAND <<<")
                    self.feed.announce_clearance(f"{self.plane.id} cleared to land")
                    self.feed.runway_status(True, f"{self.plane.id} landing")
                    # Update arrival_sim clearance flag for display
                    self.arrival_sim.clearance_given = True
                else:
                    # Holding pattern - shouldn't happen with proper sequencing
                    if not self.runway_hold_logged:
                        self.info_window.add_line("Runway occupied → HOLDING on final approach")
                        self.feed.announce_clearance(f"{self.plane.id} holding - runway occupied")
                        self.runway_hold_logged = True
            
            return self.position
        except StopIteration:
            # Landing complete - notify ground ops
            if self.position is not None:
                self.runway_exit_position = self.position.copy()
            else:
                self.runway_exit_position = RUNWAY_THRESHOLD.copy()
            self.ground_ops.clear_arrival_eta(self.plane)
            self.ground_ops.complete_landing(self.plane)
            
            # Save landing trajectory to JSON file
            self._save_trajectory_to_json()
            
            # Begin short vacate window before taxi to gate
            vacate_buffer = random.uniform(3.0, 4.0)
            self.vacate_end_time = self.sim_time + vacate_buffer
            # Shorten runway busy window to vacate period so it frees promptly
            self.ground_ops.mark_runway_busy(self.plane, vacate_buffer, self.sim_time)
            self.state = "vacating_runway"
            self.info_window.add_line(f"Landing rollout COMPLETE - vacating runway ({vacate_buffer:.1f}s)")
            self.feed.status_update(f"{self.plane.id} vacating runway (clears in {vacate_buffer:.1f}s)")
            self._log(f"Landing rollout complete; vacating runway ({vacate_buffer:.1f}s)")
            return self.position

    def _assign_gate(self):
        """Snap the aircraft to a gate once runway is free."""
        if self.ground_ops.runway_busy:
            return self.position

        reference_pos = self.runway_exit_position if self.runway_exit_position is not None else RUNWAY_THRESHOLD
        gate = self.ground_ops.allocate_gate(self.plane, reference_position=reference_pos)
        if gate is None:
            if not self.gate_wait_logged:
                self._log("No gate available → holding on apron")
                self.gate_wait_logged = True
            return self.position

        self.gate_index = gate
        self.plane.assigned_gate = gate
        self.position = self.ground_ops.gate_positions[gate]
        self.trail.append(self.position.copy())
        # Begin taxi timer from runway to gate before service starts
        taxi_duration = self._sample_taxi_to_gate_duration(reference_pos, gate)
        self.taxi_to_gate_end_time = self.sim_time + taxi_duration
        self.state = "taxi_to_gate"
        self.info_window.add_line(f">>> GATE {gate + 1} ASSIGNED <<<")
        self.info_window.add_line(f"Taxiing from runway to gate: {taxi_duration:.1f}s (ETA t={self.taxi_to_gate_end_time:.1f}s)")
        self.feed.status_update(f"{self.plane.id} taxiing to Gate {gate + 1} (ETA {taxi_duration:.1f}s, arrival t={self.taxi_to_gate_end_time:.1f}s)")
        self._log(
            f"Gate {gate + 1} assigned. Taxi to gate {taxi_duration:.1f}s (ETA t={self.taxi_to_gate_end_time:.1f})"
        )
        return self.position

    def _vacate_runway(self):
        """Short buffer after rollout before runway is freed."""
        if self.vacate_end_time is None:
            # Fallback: move straight to gate assignment
            self.state = "gate_assignment"
            return self.position

        if self.sim_time >= self.vacate_end_time:
            # Runway frees via timer check at top of step; ensure it's free now
            if self.ground_ops.runway_busy:
                self.ground_ops.release_runway()
                # Don't announce runway free here - it's already been announced by timer
            self.state = "gate_assignment"
            return self.position

        # Still vacating; keep runway busy timer intact
        return self.position

    def _sample_taxi_to_gate_duration(self, start_position: np.ndarray, gate_index: int) -> float:
        """Generate a taxi duration using a distance-aware random distribution."""
        gate_pos = self.ground_ops.gate_positions[gate_index]
        distance = float(np.linalg.norm(gate_pos - start_position))
        mean_time = max(30.0, 25.0 + distance / 12.0)  # (~5 m/s baseline with buffer)
        sigma = max(5.0, 0.2 * mean_time)
        return max(20.0, random.normalvariate(mean_time, sigma))

    def _taxi_from_runway_to_gate(self):
        """Taxi animation/timer from runway to assigned gate before service."""
        if self.taxi_to_gate_end_time is None or self.gate_index is None:
            self.state = "gate_assignment"
            return self.position

        if self.sim_time >= self.taxi_to_gate_end_time:
            self.position = self.ground_ops.gate_positions[self.gate_index]
            self.trail.append(self.position.copy())
            # Emergency aircraft need longer service time
            time_range = EMERGENCY_SERVICE_TIME_RANGE if self.is_emergency else SERVICE_TIME_RANGE
            self.service_time = random.randint(*time_range)
            self.service_end_time = self.sim_time + self.service_time
            self.state = "at_gate"
            self.runway_exit_position = None
            self.info_window.add_line(f">>> PARKED AT GATE {self.gate_index + 1} <<<")
            self.info_window.add_line(f"Service time: {self.service_time}s (until t={self.service_end_time:.1f}s)")
            self.feed.gate_event(f"{self.plane.id} parked at Gate {self.gate_index + 1}")
            self._log(
                f"Parked at Gate {self.gate_index + 1}. Service time {self.service_time}s (until t={self.service_end_time:.1f})"
            )
            return self.position

        # Still taxiing; hold position reference at runway or simple interp
        self.position = self.ground_ops.gate_positions[self.gate_index]
        return self.position

    def _service_at_gate(self):
        """Stay parked until service is done, then start taxi timer."""
        if self.sim_time >= self.service_end_time:
            taxi_time = FORCED_TAXI_TIME
            self.taxi_end_time = self.sim_time + taxi_time
            self.ground_ops.release_gate(self.gate_index)
            self.info_window.add_line(f"Service COMPLETE - beginning taxi to runway")
            self.info_window.add_line(f"Taxi time: {taxi_time:.1f}s (threshold at t={self.taxi_end_time:.1f}s)")
            self.feed.status_update(f"{self.plane.id} taxiing to runway from Gate {self.gate_index + 1} (ETA {taxi_time:.1f}s, arrival t={self.taxi_end_time:.1f}s)")
            self._log(
                f"Service complete. Taxiing to runway (ETA {taxi_time:.1f}s, spawn at threshold t={self.taxi_end_time:.1f})"
            )
            self.state = "taxi_runway"
            self.taxi_progress_reported = False
            return self.position
        
        # Stay visible at gate until taxi begins
        self.position = self.ground_ops.gate_positions[self.gate_index]
        return self.position

    def _taxi_to_runway(self):
        """Runway taxi timer; spawn at threshold when done."""
        if self.sim_time >= self.taxi_end_time:
            self.position = RUNWAY_THRESHOLD.copy()
            self.trail.append(self.position.copy())
            self.state = "runway_hold"
            self.info_window.add_line(">>> ARRIVED AT RUNWAY THRESHOLD <<<")
            self.info_window.add_line("Awaiting takeoff clearance...")
            self.feed.status_update(f"{self.plane.id} holding short of runway")
            self._log("Arrived at runway threshold, awaiting takeoff clearance")
            self.taxi_progress_reported = False
        else:
            self.position = self.ground_ops.gate_positions[self.gate_index]
            if not self.taxi_progress_reported:
                remaining = max(0.0, self.taxi_end_time - self.sim_time)
                self.info_window.add_line(f"Taxiing to runway... {remaining:.1f}s remaining")
                self._log(f"Taxiing… {remaining:.1f}s remaining")
                self.taxi_progress_reported = True
        return self.position

    def _runway_queue(self):
        """Wait for a free runway, then start the departure simulator."""
        if not self.ground_ops.is_runway_available(self.sim_time):
            if not self.runway_hold_depart_logged:
                self.info_window.add_line("Runway OCCUPIED - holding short")
                self.feed.status_update(f"{self.plane.id} holding - runway busy")
                self._log("Runway occupied → holding short")
                self.runway_hold_depart_logged = True
            return self.position
        self.runway_hold_depart_logged = False

        # Check for inbound arrivals before starting a takeoff roll
        takeoff_run_time = self.ground_ops.estimate_takeoff_runway_time(self.plane)
        if self.ground_ops.has_arrival_conflict(self.sim_time, takeoff_run_time):
            if not self.arrival_conflict_logged:
                next_eta = self.ground_ops.get_next_arrival_eta(self.sim_time)
                wait = max(0.0, (next_eta or self.sim_time) - self.sim_time)
                self.info_window.add_line(
                    f"Arrival priority — waiting {wait:.1f}s before takeoff clearance"
                )
                self.feed.status_update(f"{self.plane.id} holding for arrival separation")
                self._log(f"Arrival inbound soon; holding short (ETA {wait:.1f}s)")
                self.arrival_conflict_logged = True
            return self.position
        self.arrival_conflict_logged = False

        # Runway free -> start departure sim and block runway for takeoff roll
        self.departure_sim = DepartureSimulator(self.departure_route, self.plane, verbose=False, wind_model=self.wind_model)
        actual_run_time = max(self.departure_sim.ground_roll_data["t"][-1], 5.0) + 5.0
        block_time = max(actual_run_time, takeoff_run_time)
        self.ground_ops.mark_runway_busy(self.plane, block_time, self.sim_time)
        self.state = "departure"
        self.takeoff_runway_released = False
        self.info_window.add_line(">>> CLEARED FOR TAKEOFF <<<")
        self.info_window.add_line(f"Runway blocked for {block_time:.1f}s")
        self.feed.announce_clearance(f"{self.plane.id} cleared for takeoff")
        self.feed.runway_status(True, f"{self.plane.id} departing")
        self._log(
            f"Cleared for takeoff. Blocking runway for {block_time:.1f}s (release t={self.ground_ops.runway_free_time:.1f})"
        )
        return self.position

    def _advance_departure(self):
        """Run the departure simulator until it reaches the boundary."""
        if self.departure_sim is None:
            return self.position

        try:
            pos = next(self.departure_sim)
            # Log departure telemetry to aircraft window
            if self.departure_sim.phase == "GROUND_ROLL" and self.departure_sim.playback_index > 0:
                idx = self.departure_sim.playback_index - 1
                t = self.departure_sim.ground_roll_data["t"][idx]
                x = self.departure_sim.ground_roll_data["x"][idx]
                v = self.departure_sim.ground_roll_data["V"][idx]
                self.info_window.add_line(f"DEP | ROLL | t={t:5.2f}s | x={x:6.1f}m | V={v:5.1f} m/s")
            else:
                current_time = self.departure_sim.time_hist[-1] if self.departure_sim.time_hist else 0.0
                heading = math.degrees(self.departure_sim.dyn.psi)
                gamma = math.degrees(self.departure_sim.dyn.gamma)
                self.info_window.add_line(
                    f"DEP | {self.departure_sim.phase:9s} | t={current_time:5.1f}s | "
                    f"Pos ({pos[0]:6.1f}, {pos[1]:6.1f}, {pos[2]:5.1f}) | "
                    f"V={self.departure_sim.dyn.V:5.1f} m/s | ψ={heading:5.1f}° | γ={gamma:5.2f}°"
                )
            
            self.position = pos.copy()
            self.trail.append(pos.copy())

            # Release runway only once we have transitioned to climb AND reached safe altitude
            # Safety buffer: require at least 120m altitude before releasing runway for next operation
            if (
                not self.takeoff_runway_released
                and self.departure_sim.phase == "CLIMB"
                and pos[2] >= 120.0  # Altitude safety buffer
                and self.ground_ops.runway_busy
            ):
                self.ground_ops.release_runway()
                self.info_window.add_line(f">>> AIRBORNE - RUNWAY RELEASED (alt={pos[2]:.1f}m) <<<")
                self.feed.runway_status(False, f"{self.plane.id} airborne")
                self.feed.status_update(f"{self.plane.id} taking off")
                self._log(f"Runway freed — aircraft is airborne at {pos[2]:.1f}m")
                self.takeoff_runway_released = True
            return self.position
        except StopIteration:
            self.state = "complete"
            self.info_window.add_line(">>> DEPARTURE COMPLETE <<<")
            self.info_window.add_line("Aircraft left the airspace")
            self.feed.status_update(f"{self.plane.id} departed airspace")
            self._log("Departure complete; aircraft left the airspace")
            # Keep last known position visible briefly
            return self.position

    # ------------------------------------------------------------------
    # Presentation helpers
    # ------------------------------------------------------------------
    @property
    def state_label(self) -> str:
        labels = {
            "arrival": "Arrival",
            "vacating_runway": "Vacating runway",
            "gate_assignment": "Taxi to gate",
            "taxi_to_gate": "Taxi to gate",
            "at_gate": "At gate (service)",
            "taxi_runway": "Taxi to runway",
            "runway_hold": "Holding short",
            "departure": "Departure",
            "complete": "Complete",
        }
        label = labels.get(self.state, self.state)
        if self.state == "arrival" and self.arrival_hold_active:
            label = "Arrival (HOLD)"
        if self.is_emergency:
            label = f"EMERGENCY | {label}"
        return label


class EmergencyArrivalSimulator(ArrivalSimulator):
    """
    Arrival simulator variant that can enter a shallow holding pattern by
    oscillating between two waypoints at a constant altitude. Used during
    emergency scenarios to keep non-priority aircraft safely away from the runway.
    """

    def __init__(self, route, plane, verbose=True, wind_model: Optional[WindModel] = None, heavy_brake: bool = False):
        super().__init__(route, plane, verbose=verbose, wind_model=wind_model)
        self.hold_active = False
        self.hold_state = "FORWARD"  # FORWARD = towards next WP, BACKWARD = to previous
        self.heavy_brake = heavy_brake
        self.hold_initialized = False
        self.hold_altitude = None
        self.hold_turn_dir = 1
        self.hold_radius = 750.0  # meters - will be dynamically adjusted
        self.estimated_hold_time = 60.0  # default estimate
        self.hold_center: Optional[np.ndarray] = None
        self.hold_angle = 0.0
        self.hold_angular_rate = 0.0
        self.hold_target_speed = CRUISE_SPEED
        self.hold_elapsed = 0.0
        self.hold_loop_duration = 0.0

    def set_hold(self, active: bool, estimated_hold_time: float = 60.0):
        if self.hold_active == active:
            return
        self.hold_active = active
        if active:
            # Calculate radius so one circle takes roughly the estimated hold time
            # Circle circumference = 2π×R, time for one circle = circumference / speed
            # R = (speed × time) / (2π)
            self.hold_target_speed = CRUISE_SPEED
            self.estimated_hold_time = max(15.0, estimated_hold_time)
            self.hold_radius = max(300.0, (self.hold_target_speed * self.estimated_hold_time) / (2 * math.pi))
            self.hold_turn_dir = 1 if random.random() < 0.5 else -1
            offset = np.array([
                -math.sin(self.psi),
                math.cos(self.psi),
                0.0,
            ])
            self.hold_center = self.pos + (self.hold_turn_dir * self.hold_radius) * offset
            rel = self.pos - self.hold_center
            self.hold_angle = math.atan2(rel[1], rel[0])
            self.hold_angular_rate = self.hold_target_speed / self.hold_radius
            self.hold_elapsed = 0.0
            self.hold_loop_duration = (2 * math.pi * self.hold_radius) / self.hold_target_speed
            self.hold_initialized = True
            self.hold_altitude = self.pos[2]
        if not active:
            self.hold_state = "FORWARD"
            self.hold_initialized = False
            self.hold_altitude = None
            self.hold_center = None
            self.hold_angle = 0.0
            self.hold_angular_rate = 0.0
            self.hold_elapsed = 0.0

    def _start_rollout(self):
        """Standard landing rollout for emergency scenario."""
        rollout_dyn = FlightDynamics(self.plane)
        rollout_dyn.reset_state()
        # Use standard reverse thrust
        T_rev = 50000.0
        rollout_dyn.ground_roll_landing(dt=ARRIVAL_DT, V_touchdown=self.V, T_reverse=T_rev)
        self.rollout_points = []
        self.rollout_speeds = []
        for entry in rollout_dyn.telemetry():
            x_offset = min(entry["x"], RUNWAY_LENGTH - 20)
            self.rollout_points.append(np.array([RUNWAY_THRESHOLD[0] + x_offset, 0.0, 0.0]))
            self.rollout_speeds.append(entry.get("V", 0.0))
            if x_offset >= RUNWAY_LENGTH - 20:
                break
        if not self.rollout_points:
            self.rollout_points.append(RUNWAY_THRESHOLD.copy())
            self.rollout_speeds = [0.0]
        # Initialize rollout speed tracking
        self.current_rollout_speed = self.rollout_speeds[0] if self.rollout_speeds else 0.0

    def __next__(self):
        if self.finished:
            raise StopIteration

        if self.landed:
            if self.rollout_index < len(self.rollout_points):
                pos = self.rollout_points[self.rollout_index]
                if self.rollout_index < len(self.rollout_speeds):
                    self.current_rollout_speed = self.rollout_speeds[self.rollout_index]
                self.rollout_index += 1
                return pos
            self.finished = True
            raise StopIteration

        if self.segment >= len(self.wps) - 1:
            self.finished = True
            raise StopIteration

        # Wind update
        if self.external_wind:
            self.wind_vector = self.wind.current()
        else:
            self.wind_vector = self.wind.update(ARRIVAL_DT)
        wind_vec = self.wind_vector

        # HOLDING LOOP
        if self.hold_active:
            if not self.hold_initialized or self.hold_center is None:
                self.set_hold(True, self.estimated_hold_time)

            self.hold_elapsed += ARRIVAL_DT
            theta_dot = self.hold_turn_dir * self.hold_angular_rate
            self.hold_angle = (self.hold_angle + theta_dot * ARRIVAL_DT) % (2 * math.pi)
            self.pos[0] = self.hold_center[0] + self.hold_radius * math.cos(self.hold_angle)
            self.pos[1] = self.hold_center[1] + self.hold_radius * math.sin(self.hold_angle)
            self.pos[2] = self.hold_altitude

            vx = -self.hold_radius * math.sin(self.hold_angle) * theta_dot
            vy = self.hold_radius * math.cos(self.hold_angle) * theta_dot
            self.psi = math.atan2(vy, vx)
            self.gamma = 0.0
            self.V = self.hold_target_speed

            self.current_time += ARRIVAL_DT
            self.time_hist.append(self.current_time)
            self.speed_hist.append(self.V)
            return self.pos.copy()
        else:
            self.hold_initialized = False
            self.hold_altitude = None
            self.hold_center = None

        # NORMAL ARRIVAL (uses original speed management)
        target = np.array(self.wps[self.segment + 1], dtype=float)

        vec = target - self.pos
        seg_vec = target - self.seg_start
        horiz = math.hypot(vec[0], vec[1])
        dist = math.sqrt(horiz**2 + vec[2]**2)

        if np.dot(vec, seg_vec) <= 0:
            self.segment += 1
            self.seg_start = self.pos.copy()
            if self.segment >= len(self.wps) - 1:
                self.landed = True
                self._start_rollout()
                return self.pos
            target = np.array(self.wps[self.segment + 1], dtype=float)
            vec = target - self.pos
            horiz = math.hypot(vec[0], vec[1])
            dist = math.sqrt(horiz**2 + vec[2]**2)

        if dist < 5.0:
            self.pos = target.copy()
            if self.segment == len(self.wps) - 2:
                self.landed = True
                self._start_rollout()
            else:
                self.segment += 1
                self.seg_start = target.copy()
            return self.pos

        if self.segment >= len(self.wps) - 3:
            target_speed = FINAL_SPEED
        elif self.segment >= len(self.wps) - 4:
            target_speed = APPROACH_SPEED
        else:
            target_speed = CRUISE_SPEED

        target_psi = math.atan2(vec[1], vec[0])
        target_gamma = math.atan2(vec[2], horiz if horiz > 1e-3 else 1e-3)

        psi_err = math.atan2(math.sin(target_psi - self.psi), math.cos(target_psi - self.psi))
        phi_cmd = clamp(HEADING_GAIN * psi_err, -MAX_BANK_RAD, MAX_BANK_RAD)
        self.psi += self.dyn.compute_heading_rate(self.V, phi_cmd) * ARRIVAL_DT

        self.gamma += clamp(GAMMA_GAIN * (target_gamma - self.gamma), -0.05, 0.05)

        speed_err = target_speed - self.V
        speedbrake_multiplier = 1.0
        if speed_err < -10.0:
            throttle_setting = 0.0
            speedbrake_multiplier = 4.0
        elif speed_err < -3.0:
            throttle_setting = 0.0
            speedbrake_multiplier = 3.5
        elif speed_err < 0:
            throttle_setting = clamp(0.02 + 0.1 * speed_err / target_speed, 0.0, 0.08)
            speedbrake_multiplier = 2.5
        else:
            throttle_setting = clamp(0.3 + V_THROTTLE_GAIN * speed_err / target_speed, 0.08, 0.85)

        thrust = self.plane.compute_thrust(self.V) * throttle_setting
        base_drag = self.plane.compute_drag(self.V, self.gamma)
        drag = base_drag * speedbrake_multiplier
        dVdt = (thrust - drag - self.plane.mass * 9.81 * math.sin(self.gamma)) / self.plane.mass
        self.V = max(60.0, self.V + dVdt * ARRIVAL_DT)

        self.current_time += ARRIVAL_DT
        self.time_hist.append(self.current_time)
        self.speed_hist.append(self.V)

        air_dx = self.V * math.cos(self.gamma) * math.cos(self.psi) * ARRIVAL_DT
        air_dy = self.V * math.cos(self.gamma) * math.sin(self.psi) * ARRIVAL_DT
        air_dz = self.V * math.sin(self.gamma) * ARRIVAL_DT
        self.pos[0] += air_dx + wind_vec[0] * ARRIVAL_DT
        self.pos[1] += air_dy + wind_vec[1] * ARRIVAL_DT
        self.pos[2] += air_dz + wind_vec[2] * ARRIVAL_DT

        if not self.clearance_given and self.segment == len(self.wps) - 2:
            self.clearance_given = True

        return self.pos.copy()


class MultiAircraftSimulator:
    """Orchestrates multiple turnaround flows with random arrivals."""

    def __init__(
        self,
        visual_mode: bool = True,
        emergency_mode: bool = False,
        emergency_spawn_chance: float = 1.0 / 10000.0,
    ):
        self.ground_ops = GroundOperations(runway_length=RUNWAY_LENGTH, num_gates=19)
        self.visual_mode = visual_mode
        self.emergency_mode = emergency_mode
        # Treat chance as probability per aircraft spawn, clamp into [0, 1]
        self.emergency_spawn_chance = max(0.0, min(1.0, emergency_spawn_chance))
        self.console_window = ATCConsoleWindow(enabled=visual_mode)
        self.feed = TerminalFeed(self.console_window)
        self.wind_model = WindModel()
        self.wind_vector = np.zeros(3)
        self.actors: list[SingleAircraftTurnaround] = []
        self.global_time = 0.0
        self.next_spawn_time = 0.0
        self.dt = GLOBAL_DT
        self.frame_interval_ms = 100
        self.emergency_actor: Optional[SingleAircraftTurnaround] = None

        if self.emergency_mode:
            self._spawn_emergency_flight()
            self.next_spawn_time = float("inf")
            self.emergency_active = True
        else:
            self.emergency_active = False
            self._spawn_aircraft(initial=True)

    def _plane_to_actor(self, plane) -> Optional["SingleAircraftTurnaround"]:
        if plane is None:
            return None
        for actor in self.actors:
            if actor.plane is plane:
                return actor
        return None

    def _sample_spawn_interval(self) -> float:
        interval = random.expovariate(1.0 / INTER_ARRIVAL_MEAN)
        return max(MIN_SPAWN_INTERVAL, interval)

    def _spawn_aircraft(self, initial: bool = False):
        if self.emergency_mode or self.emergency_active:
            return
        if len(self.actors) >= MAX_ACTIVE_AIRCRAFT:
            self.next_spawn_time = self.global_time + self._sample_spawn_interval()
            return

        actor = SingleAircraftTurnaround(
            self.ground_ops,
            self.feed,
            wind_model=self.wind_model,
            visual_mode=self.visual_mode,
            arrival_sim_cls=EmergencyArrivalSimulator,
            arrival_sim_kwargs={"heavy_brake": False},
        )
        self.actors.append(actor)
        self.next_spawn_time = self.global_time + self._sample_spawn_interval()
        if not initial:
            self.feed.status_update(
                f"New arrival: {actor.plane.id} on {actor.arrival_route['name']}"
            )

    def _spawn_emergency_flight(self):
        emergency_cls = random.choice(FLEET)
        emergency_plane = emergency_cls(f"{emergency_cls.__name__}-{random.randint(100, 999)}")
        emerg_route = random.choice(generate_arrival_routes(num_routes=20))

        emergency_actor = SingleAircraftTurnaround(
            self.ground_ops,
            self.feed,
            wind_model=self.wind_model,
            visual_mode=self.visual_mode,
            plane=emergency_plane,
            arrival_route=emerg_route,
            is_emergency=True,
            arrival_sim_cls=EmergencyArrivalSimulator,
            arrival_sim_kwargs={"heavy_brake": True},
            info_window_enabled=True,
        )

        self.actors.append(emergency_actor)
        self.emergency_actor = emergency_actor
        self.feed.status_update(
            f"Emergency flight {emergency_plane.id} inbound on {emerg_route['name']}."
        )
        self.emergency_active = True

    def _update_emergency_hold_states(self):
        emergency_required = self.emergency_mode or self.emergency_active
        if self.emergency_actor not in self.actors:
            self.emergency_actor = None

        emergency_inbound = False
        emergency_eta = None
        estimated_emergency_time = 60.0
        if self.emergency_actor is not None:
            emergency_inbound = self.emergency_actor.state in ("arrival", "vacating_runway")
            if hasattr(self.emergency_actor, "arrival_sim"):
                try:
                    emergency_eta = max(0.0, self.emergency_actor.arrival_sim.estimate_time_to_threshold())
                except Exception:
                    emergency_eta = None
            if emergency_eta is not None:
                estimated_emergency_time = max(30.0, emergency_eta + EMERGENCY_CLEARANCE_BUFFER)

        if emergency_required and not emergency_inbound:
            self.emergency_active = False
            self.emergency_actor = None

        runway_departure_actor = None
        runway_departure_hold_time = 0.0
        if self.ground_ops.runway_busy:
            owner = self._plane_to_actor(self.ground_ops.current_runway_plane)
            if owner is not None and owner.state == "departure":
                runway_departure_actor = owner
                runway_departure_hold_time = max(
                    5.0,
                    (self.ground_ops.runway_free_time - self.global_time) + LANDING_CLEAR_BUFFER,
                )

        for actor in self.actors:
            if actor.state != "arrival":
                continue
            if actor is self.emergency_actor:
                actor.set_arrival_hold(False)
                continue

            hold_durations = []

            if emergency_inbound and self.emergency_actor is not None:
                allow_continue = False
                normal_eta = None
                if hasattr(actor, "arrival_sim"):
                    try:
                        normal_eta = max(0.0, actor.arrival_sim.estimate_time_to_threshold())
                    except Exception:
                        normal_eta = None

                if normal_eta is not None and emergency_eta is not None:
                    clearance_time = normal_eta + ARRIVAL_RUNWAY_CLEARANCE_TIME
                    if clearance_time + EMERGENCY_MIN_LEAD_TIME <= emergency_eta:
                        allow_continue = True
                if not allow_continue:
                    hold_durations.append(estimated_emergency_time)

            if runway_departure_actor is not None:
                hold_durations.append(runway_departure_hold_time)

            if hold_durations:
                actor.set_arrival_hold(True, max(hold_durations))
            else:
                actor.set_arrival_hold(False)

        if self.emergency_actor is None and not self.emergency_mode:
            self.emergency_active = False

        # Make sure emergency itself is not holding
        if self.emergency_actor and hasattr(self.emergency_actor, "set_arrival_hold"):
            self.emergency_actor.set_arrival_hold(False)

    def step(self):
        """Advance global time and step all active aircraft."""
        self.global_time += self.dt
        self.wind_vector = self.wind_model.update(self.dt)
        # Expose emergency flag for other components
        self.ground_ops.emergency_active = self.emergency_active
        if not self.emergency_mode:
            if self.global_time >= self.next_spawn_time and not self.emergency_active:
                # Evaluate emergency probability once per scheduled spawn event
                spawn_emergency = random.random() < self.emergency_spawn_chance
                if spawn_emergency:
                    self.feed.status_update("⚠ Emergency detected in airspace")
                    self._spawn_emergency_flight()
                    # Sample a fresh interval so we don't immediately spawn again
                    self.next_spawn_time = self.global_time + self._sample_spawn_interval()
                else:
                    self._spawn_aircraft()

        self._update_emergency_hold_states()

        updates = []
        for actor in list(self.actors):
            pos = actor.step(self.global_time)
            updates.append((actor, pos))
            if actor.state == "complete":
                actor.info_window.close()
                self.actors.remove(actor)
        return updates


def run_visual_sim(max_frames: int = 6000, show: bool = True, emergency_mode: bool = False, emergency_chance: float = 1.0 / 10000.0):
    sim = MultiAircraftSimulator(
        visual_mode=True,
        emergency_mode=emergency_mode,
        emergency_spawn_chance=emergency_chance,
    )

    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection="3d")

    # Plot static geometry
    ax.plot(
        [RUNWAY_THRESHOLD[0], RUNWAY_END[0]],
        [RUNWAY_THRESHOLD[1], RUNWAY_END[1]],
        [RUNWAY_THRESHOLD[2], RUNWAY_END[2]],
        color="gray",
        linewidth=8,
        label="Runway",
        alpha=0.7,
    )

    gate_positions = sim.ground_ops.gate_positions
    ax.scatter(
        gate_positions[:, 0],
        gate_positions[:, 1],
        gate_positions[:, 2],
        c="lightblue",
        s=80,
        marker="s",
        alpha=0.6,
        label="Gates",
        zorder=2,
    )
    for i, pos in enumerate(gate_positions):
        ax.text(pos[0], pos[1], pos[2] + 15, f"G{i+1}", fontsize=7, ha="center", color="navy", alpha=0.7)

    ax.set_xlim([-AIRSPACE_RADIUS, AIRSPACE_RADIUS])
    ax.set_ylim([-AIRSPACE_RADIUS, AIRSPACE_RADIUS])
    ax.set_zlim([0, 600])
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Altitude (m)")
    ax.set_title("Autonomous Turnaround Simulation")
    ax.legend(loc="upper left")
    ax.grid(True, alpha=0.25)
    ax.view_init(elev=28, azim=-60)
    wind_text = fig.text(0.02, 0.95, "", fontsize=11, fontweight="bold", color="darkgreen")

    actor_artists = {}
    actor_paths: dict[str, dict] = {}
    cmap = plt.colormaps.get_cmap("tab10")

    def init():
        return []

    def update(frame):
        updates = sim.step()
        # Remove artists for completed actors
        current_ids = {actor.plane.id for actor in sim.actors}
        for actor_id in list(actor_artists.keys()):
            if actor_id not in current_ids:
                art = actor_artists.pop(actor_id)
                for h in art.values():
                    if hasattr(h, "remove"):
                        try:
                            h.remove()
                        except Exception:
                            pass
                actor_paths.pop(actor_id, None)

        artists = []
        for actor, pos in updates:
            actor_id = actor.plane.id
            if actor_id not in actor_artists:
                color = cmap(len(actor_artists) % 10)
                point, = ax.plot([], [], [], "o", color=color, markersize=8, label=f"{actor_id}")
                trail, = ax.plot([], [], [], color=color, linewidth=2, alpha=0.8)
                label = ax.text(0, 0, 0, "", fontsize=9, color=color, fontweight="bold")
                arr_wps = actor.arrival_route["waypoints"]
                dep_wps = actor.departure_route["waypoints"]
                arr_line, = ax.plot(arr_wps[:, 0], arr_wps[:, 1], arr_wps[:, 2], linestyle="--", color=color, alpha=0.25)
                dep_line, = ax.plot(dep_wps[:, 0], dep_wps[:, 1], dep_wps[:, 2], linestyle="--", color=color, alpha=0.25)
                actor_artists[actor_id] = {
                    "point": point,
                    "trail": trail,
                    "label": label,
                    "arr": arr_line,
                    "dep": dep_line,
                }
                actor_paths[actor_id] = {"points": [], "drawing": False}

            art = actor_artists[actor_id]
            if pos is not None:
                should_draw_path = actor.state in ("arrival", "departure", "runway_hold")
                path_info = actor_paths.get(actor_id, {"points": [], "drawing": False})
                if should_draw_path:
                    if not path_info["drawing"] and len(path_info["points"]) > 0:
                        path_info["points"].append(np.array([np.nan, np.nan, np.nan]))
                    path_info["drawing"] = True
                    path_info["points"].append(pos.copy())
                else:
                    path_info["drawing"] = False
                actor_paths[actor_id] = path_info

                art["point"].set_data([pos[0]], [pos[1]])
                art["point"].set_3d_properties([pos[2]])
                art["label"].set_position((pos[0], pos[1]))
                art["label"].set_3d_properties(pos[2] + 25)
                art["label"].set_text(f"{actor_id}\n{actor.state_label}")
                path = np.array(path_info["points"]) if path_info["points"] else np.empty((0, 3))
                if path.size > 0:
                    art["trail"].set_data(path[:, 0], path[:, 1])
                    art["trail"].set_3d_properties(path[:, 2])
                else:
                    art["trail"].set_data([], [])
                    art["trail"].set_3d_properties([])
            artists.extend(art.values())

        ax.set_title(
            f"Autonomous Turnaround | Active aircraft: {len(sim.actors)} | t={sim.global_time:.1f}s\n"
            f"Gates occupied: {len(sim.ground_ops.get_all_parked_aircraft())}/19 | "
            f"Runway: {'BUSY' if sim.ground_ops.runway_busy else 'FREE'}",
            fontsize=13,
            fontweight="bold",
        )
        wind_speed = math.hypot(sim.wind_vector[0], sim.wind_vector[1])
        wind_knots = wind_speed * 1.94384
        wind_dir = (math.degrees(math.atan2(sim.wind_vector[1], sim.wind_vector[0])) + 360.0) % 360.0
        wind_text.set_text(
            f"Wind: {wind_speed:.1f} m/s ({wind_knots:.1f} kt) @ {wind_dir:.0f}° | "
            f"Vertical {sim.wind_vector[2]:+.2f} m/s"
        )

        # No return needed when blit=False
        return []

    ani = animation.FuncAnimation(
        fig,
        update,
        frames=max_frames,
        init_func=init,
        interval=sim.frame_interval_ms,
        blit=True,  # Must be False for 3D plots to update properly
        repeat=False,
    )

    if show:
        plt.ion()  # Enable interactive mode
        plt.show(block=True)  # Block to keep windows open
    return fig, ani


def run_headless(max_frames: int = 6000, emergency_mode: bool = False, emergency_chance: float = 1.0 / 2.0):
    """
    Headless run for quick validation (no plotting).
    """
    sim = MultiAircraftSimulator(
        visual_mode=False,
        emergency_mode=emergency_mode,
        emergency_spawn_chance=emergency_chance,
    )
    for _ in range(max_frames):
        sim.step()
    print("\nHeadless run finished.")
    print(f"Active aircraft remaining: {len(sim.actors)}")
    print(f"Total sim time: {sim.global_time:.1f}s")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Autonomous single-aircraft turnaround demo")
    parser.add_argument("--headless", action="store_true", help="Run without plotting (fast sanity check)")
    parser.add_argument(
        "--emergency",
        action="store_true",
        help="Run emergency landing scenario with priority handling",
    )
    parser.add_argument(
        "--frames",
        type=int,
        default=6000,
        help="Maximum animation frames (6000 frames = 10 minutes of simulation time)",
    )
    parser.add_argument(
        "--emergency-chance",
        type=float,
        default=1.0 / 15.0,
        help="Per-aircraft probability that the next spawn will be an emergency",
    )
    args = parser.parse_args()

    if args.headless:
        run_headless(max_frames=args.frames, emergency_mode=args.emergency, emergency_chance=args.emergency_chance)
    else:
        run_visual_sim(max_frames=args.frames, show=True, emergency_mode=args.emergency, emergency_chance=args.emergency_chance)
