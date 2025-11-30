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
import math
import random
import numpy as np
import matplotlib
matplotlib.use('TkAgg')  # Force interactive backend
import matplotlib.pyplot as plt
from matplotlib import animation

from arrival_live_sim import ArrivalSimulator, DT as ARRIVAL_DT
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

# Service time bounds (seconds)
SERVICE_TIME_RANGE = (20, 35)

# Fleet to sample from
FLEET = [EmbraerE170, AirbusA220, Dash8_400, ATR72_600]

# Temporary forced taxi duration (debugging convenience)
FORCED_TAXI_TIME = 10.0
FINAL_APPROACH_TARGET = 65.0
INTER_ARRIVAL_MEAN = 105.0  # seconds between new arrivals (mean)
MIN_SPAWN_INTERVAL = 45.0   # seconds, safety lower bound
MAX_ACTIVE_AIRCRAFT = 4
GLOBAL_DT = DEPARTURE_DT  # base time step for orchestration


class AircraftInfoWindow:
    """Separate matplotlib window for each aircraft's detailed telemetry."""

    def __init__(self, plane_id: str, enabled: bool = True):
        self.plane_id = plane_id
        self.enabled = enabled
        self.lines = []
        self.max_lines = 50
        
        if self.enabled:
            self.fig = plt.figure(figsize=(10, 7))
            self.fig.canvas.manager.set_window_title(f"Aircraft {plane_id} - Telemetry")
            self.ax = self.fig.add_subplot(111)
            self.ax.axis('off')
            self.text = self.ax.text(0.05, 0.95, "", transform=self.ax.transAxes,
                                     fontfamily='monospace', fontsize=9,
                                     verticalalignment='top', wrap=True)
            plt.ion()
            self.fig.show()
        else:
            self.fig = None
            self.ax = None
            self.text = None

    def add_line(self, line: str):
        """Add a new line to the info window."""
        self.lines.append(line)
        if len(self.lines) > self.max_lines:
            self.lines.pop(0)
        
        if self.enabled:
            self.text.set_text('\n'.join(self.lines))
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()

    def close(self):
        """Close the window."""
        if self.enabled and self.fig is not None:
            plt.close(self.fig)


class TerminalFeed:
    """Manages terminal output for status messages only."""

    def __init__(self):
        self.banner_printed = False

    def announce_clearance(self, text: str):
        print(f"ATC | {text}", flush=True)

    def runway_status(self, busy: bool, reason: str):
        state = "BUSY" if busy else "FREE"
        print(f"ATC | Runway {state} → {reason}", flush=True)

    def gate_event(self, text: str):
        print(f"GATE | {text}", flush=True)

    def status_update(self, text: str):
        """General status updates for terminal."""
        print(f"STATUS | {text}", flush=True)


class SingleAircraftTurnaround:
    """State machine for one aircraft from arrival through departure."""

    def __init__(self, ground_ops: GroundOperations, feed: TerminalFeed, visual_mode: bool = True):
        self.arrival_route = random.choice(generate_arrival_routes(num_routes=20))
        self.departure_route = random.choice(generate_departure_routes())
        plane_cls = random.choice(FLEET)
        plane_id = f"{plane_cls.__name__}-{random.randint(100, 999)}"
        self.plane = plane_cls(plane_id)

        self.ground_ops = ground_ops
        self.arrival_sim = ArrivalSimulator(self.arrival_route, self.plane, verbose=False)
        self.departure_sim = None

        self.state = "arrival"  # arrival -> gate_assignment -> at_gate -> taxi_runway -> runway_hold -> departure -> complete
        self.position = None
        self.trail = []
        self.sim_time = 0.0
        self.frame_dt = ARRIVAL_DT
        self.feed = feed
        self.info_window = AircraftInfoWindow(plane_id, enabled=visual_mode)

        self.landing_busy_marked = False
        self.final_speed_logged = False
        self.runway_hold_logged = False
        self.gate_wait_logged = False
        self.runway_hold_depart_logged = False
        self.gate_index = None
        self.takeoff_runway_released = False

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
        self.feed.status_update(f"{self.plane.id} spawned on {self.arrival_route['name']}")

    def _log(self, message: str):
        print(f"[t={self.sim_time:5.1f}s] {message}")

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

        # Free runway when its busy window expires
        if getattr(self.ground_ops, "runway_busy", False) and self.sim_time >= self.ground_ops.runway_free_time:
            self.ground_ops.release_runway()
            self.feed.runway_status(False, "timer elapsed")
            self._log("Runway now FREE (timer elapsed)")

        if self.state == "arrival":
            return self._advance_arrival()
        if self.state == "gate_assignment":
            return self._assign_gate()
        if self.state == "at_gate":
            return self._service_at_gate()
        if self.state == "taxi_runway":
            return self._taxi_to_runway()
        if self.state == "runway_hold":
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

            if not self.arrival_sim.landed:
                # Log detailed telemetry to aircraft's window
                total = len(self.arrival_sim.route["waypoints"]) - 1
                seg = min(self.arrival_sim.segment + 1, total)
                target_idx = min(self.arrival_sim.segment + 1, total)
                target = self.arrival_sim.route["waypoints"][target_idx]
                self.info_window.add_line(
                    f"ARR | Seg {seg:02d}/{total:02d} | "
                    f"Pos ({pos[0]:7.1f}, {pos[1]:7.1f}, {pos[2]:6.1f}) | "
                    f"HDG {math.degrees(self.arrival_sim.psi):5.1f}° | "
                    f"Γ {math.degrees(self.arrival_sim.gamma):5.2f}° | "
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
                rollout_progress = self.arrival_sim.rollout_index
                rollout_total = len(self.arrival_sim.rollout_points)
                distance_covered = pos[0] - RUNWAY_THRESHOLD[0]
                self.info_window.add_line(
                    f"ROLLOUT | Progress {rollout_progress:03d}/{rollout_total:03d} | "
                    f"Distance: {distance_covered:6.1f}m | "
                    f"Pos ({pos[0]:7.1f}, {pos[1]:7.1f}, {pos[2]:6.1f})"
                )

            # Check if aircraft is on final approach (last 2 segments before runway)
            on_final_approach = self.arrival_sim.segment >= len(self.arrival_sim.wps) - 2
            
            # Request landing clearance when on final approach
            if on_final_approach and not self.landing_busy_marked:
                clearance_granted = self.ground_ops.request_landing_clearance(
                    self.plane,
                    self.sim_time,
                    on_final_approach=True,
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
            self.ground_ops.complete_landing(self.plane)
            self.state = "gate_assignment"
            self.info_window.add_line("Landing rollout COMPLETE - taxiing clear of runway")
            self.feed.status_update(f"{self.plane.id} landed, vacating runway")
            self._log("Landing rollout complete; taxiing clear of runway")
            return self.position

    def _assign_gate(self):
        """Snap the aircraft to a gate once runway is free."""
        if self.ground_ops.runway_busy:
            return self.position

        gate = self.ground_ops.allocate_gate(self.plane)
        if gate is None:
            if not self.gate_wait_logged:
                self._log("No gate available → holding on apron")
                self.gate_wait_logged = True
            return self.position

        self.gate_index = gate
        self.plane.assigned_gate = gate
        self.position = self.ground_ops.gate_positions[gate]
        self.trail.append(self.position.copy())
        self.service_time = random.randint(*SERVICE_TIME_RANGE)
        self.service_end_time = self.sim_time + self.service_time
        self.state = "at_gate"
        self.info_window.add_line(f">>> PARKED AT GATE {gate + 1} <<<")
        self.info_window.add_line(f"Service time: {self.service_time}s (until t={self.service_end_time:.1f}s)")
        self.feed.gate_event(f"{self.plane.id} parked at Gate {gate + 1}")
        self._log(
            f"Parked at Gate {gate + 1}. Service time {self.service_time}s (until t={self.service_end_time:.1f})"
        )
        return self.position

    def _service_at_gate(self):
        """Stay parked until service is done, then start taxi timer."""
        if self.sim_time >= self.service_end_time:
            taxi_time = FORCED_TAXI_TIME
            self.taxi_end_time = self.sim_time + taxi_time
            self.ground_ops.release_gate(self.gate_index)
            self.info_window.add_line(f"Service COMPLETE - beginning taxi to runway")
            self.info_window.add_line(f"Taxi time: {taxi_time:.1f}s (threshold at t={self.taxi_end_time:.1f}s)")
            self.feed.status_update(f"{self.plane.id} taxiing to runway from Gate {self.gate_index + 1}")
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

        # Runway free -> start departure sim and block runway for takeoff roll
        self.departure_sim = DepartureSimulator(self.departure_route, self.plane, verbose=False)
        takeoff_run_time = max(self.departure_sim.ground_roll_data["t"][-1], 5.0) + 5.0
        self.ground_ops.mark_runway_busy(self.plane, takeoff_run_time, self.sim_time)
        self.state = "departure"
        self.takeoff_runway_released = False
        self.info_window.add_line(">>> CLEARED FOR TAKEOFF <<<")
        self.info_window.add_line(f"Runway blocked for {takeoff_run_time:.1f}s")
        self.feed.announce_clearance(f"{self.plane.id} cleared for takeoff")
        self.feed.runway_status(True, f"{self.plane.id} departing")
        self._log(
            f"Cleared for takeoff. Blocking runway for {takeoff_run_time:.1f}s (release t={self.ground_ops.runway_free_time:.1f})"
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

            # Release runway only once we have transitioned to climb
            if (
                not self.takeoff_runway_released
                and self.departure_sim.phase == "CLIMB"
                and self.ground_ops.runway_busy
            ):
                self.ground_ops.release_runway()
                self.info_window.add_line(">>> AIRBORNE - RUNWAY RELEASED <<<")
                self.feed.runway_status(False, f"{self.plane.id} airborne")
                self.feed.status_update(f"{self.plane.id} taking off")
                self._log("Runway freed — aircraft is airborne")
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
            "gate_assignment": "Taxi to gate",
            "at_gate": "At gate (service)",
            "taxi_runway": "Taxi to runway",
            "runway_hold": "Holding short",
            "departure": "Departure",
            "complete": "Complete",
        }
        return labels.get(self.state, self.state)


class MultiAircraftSimulator:
    """Orchestrates multiple turnaround flows with random arrivals."""

    def __init__(self, visual_mode: bool = True):
        self.ground_ops = GroundOperations(runway_length=RUNWAY_LENGTH, num_gates=19)
        self.feed = TerminalFeed()
        self.actors: list[SingleAircraftTurnaround] = []
        self.global_time = 0.0
        self.next_spawn_time = 0.0
        self.dt = GLOBAL_DT
        self.frame_interval_ms = 100
        self.visual_mode = visual_mode
        self._spawn_aircraft(initial=True)

    def _sample_spawn_interval(self) -> float:
        interval = random.expovariate(1.0 / INTER_ARRIVAL_MEAN)
        return max(MIN_SPAWN_INTERVAL, interval)

    def _spawn_aircraft(self, initial: bool = False):
        if len(self.actors) >= MAX_ACTIVE_AIRCRAFT:
            self.next_spawn_time = self.global_time + self._sample_spawn_interval()
            return

        actor = SingleAircraftTurnaround(self.ground_ops, self.feed, visual_mode=self.visual_mode)
        self.actors.append(actor)
        # Schedule the next spawn window
        self.next_spawn_time = self.global_time + self._sample_spawn_interval()
        if initial:
            pass  # Initial spawn logged in _log_intro
        else:
            self.feed.status_update(
                f"New arrival: {actor.plane.id} on {actor.arrival_route['name']}"
            )

    def step(self):
        """Advance global time and step all active aircraft."""
        self.global_time += self.dt
        if self.global_time >= self.next_spawn_time:
            self._spawn_aircraft()

        updates = []
        for actor in list(self.actors):
            pos = actor.step(self.global_time)
            updates.append((actor, pos))
            if actor.state == "complete":
                # Close the aircraft's info window
                actor.info_window.close()
                self.actors.remove(actor)
        return updates


def run_visual_sim(max_frames: int = 2500, show: bool = True):
    sim = MultiAircraftSimulator(visual_mode=True)

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

    actor_artists = {}
    actor_paths = {}
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
                actor_paths[actor_id] = []

            art = actor_artists[actor_id]
            if pos is not None:
                actor_paths[actor_id].append(pos.copy())
                art["point"].set_data([pos[0]], [pos[1]])
                art["point"].set_3d_properties([pos[2]])
                art["label"].set_position((pos[0], pos[1]))
                art["label"].set_3d_properties(pos[2] + 25)
                art["label"].set_text(f"{actor_id}\n{actor.state_label}")
                path = np.array(actor_paths[actor_id])
                if len(path) > 0:
                    art["trail"].set_data(path[:, 0], path[:, 1])
                    art["trail"].set_3d_properties(path[:, 2])
            artists.extend(art.values())

        ax.set_title(
            f"Autonomous Turnaround | Active aircraft: {len(sim.actors)} | t={sim.global_time:.1f}s\n"
            f"Gates occupied: {len(sim.ground_ops.get_all_parked_aircraft())}/19 | "
            f"Runway: {'BUSY' if sim.ground_ops.runway_busy else 'FREE'}",
            fontsize=13,
            fontweight="bold",
        )

        # No return needed when blit=False
        return []

    ani = animation.FuncAnimation(
        fig,
        update,
        frames=max_frames,
        init_func=init,
        interval=sim.frame_interval_ms,
        blit=False,  # Must be False for 3D plots to update properly
        repeat=False,
    )

    if show:
        plt.ion()  # Enable interactive mode
        plt.show(block=True)  # Block to keep windows open
    return fig, ani


def run_headless(max_frames: int = 2500):
    """
    Headless run for quick validation (no plotting).
    """
    sim = MultiAircraftSimulator(visual_mode=False)
    for _ in range(max_frames):
        sim.step()
    print("\nHeadless run finished.")
    print(f"Active aircraft remaining: {len(sim.actors)}")
    print(f"Total sim time: {sim.global_time:.1f}s")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Autonomous single-aircraft turnaround demo")
    parser.add_argument("--headless", action="store_true", help="Run without plotting (fast sanity check)")
    parser.add_argument(
        "--frames",
        type=int,
        default=2500,
        help="Maximum animation frames (≈2500 needed for a full arrival → departure cycle)",
    )
    args = parser.parse_args()

    if args.headless:
        run_headless(max_frames=args.frames)
    else:
        run_visual_sim(max_frames=args.frames, show=True)
