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


class TerminalFeed:
    """Mirrors the verbose arrival/departure console output for live monitoring."""

    def announce_arrival(self, sim: ArrivalSimulator, pos: np.ndarray):
        total = len(sim.route["waypoints"]) - 1
        seg = min(sim.segment + 1, total)
        target_idx = min(sim.segment + 1, total)
        target = sim.route["waypoints"][target_idx]
        print(
            f"ARR  | Seg {seg:02d}/{total:02d} | Pos ({pos[0]:8.1f}, {pos[1]:8.1f}, {pos[2]:6.1f}) | "
            f"HDG {math.degrees(sim.psi):6.1f}° | Γ {math.degrees(sim.gamma):6.2f}° | "
            f"V {sim.V:6.1f} m/s | Target ({target[0]:.1f}, {target[1]:.1f}, {target[2]:.1f})"
        )

    def announce_clearance(self, text: str):
        print(f"ATC | {text}")

    def runway_status(self, busy: bool, reason: str):
        state = "BUSY" if busy else "FREE"
        print(f"ATC | Runway {state} → {reason}")

    def gate_event(self, text: str):
        print(f"GATE | {text}")

    def announce_departure(self, sim: DepartureSimulator, pos: np.ndarray):
        if sim.phase == "GROUND_ROLL" and sim.playback_index > 0:
            idx = sim.playback_index - 1
            t = sim.ground_roll_data["t"][idx]
            x = sim.ground_roll_data["x"][idx]
            v = sim.ground_roll_data["V"][idx]
            print(f"DEP  | ROLL | t={t:6.2f}s | x={x:7.1f} m | V={v:6.1f} m/s")
            return

        current_time = sim.time_hist[-1] if sim.time_hist else 0.0
        heading = math.degrees(sim.dyn.psi)
        gamma = math.degrees(sim.dyn.gamma)
        print(
            f"DEP  | {sim.phase:9s} | t={current_time:6.2f}s | "
            f"Pos ({pos[0]:7.1f}, {pos[1]:7.1f}, {pos[2]:6.1f}) | "
            f"V={sim.dyn.V:6.1f} m/s | ψ={heading:6.1f}° | γ={gamma:6.2f}°"
        )


class SingleAircraftTurnaround:
    """State machine for one aircraft from arrival through departure."""

    def __init__(self):
        self.arrival_route = random.choice(generate_arrival_routes(num_routes=20))
        self.departure_route = random.choice(generate_departure_routes())
        plane_cls = random.choice(FLEET)
        plane_id = f"{plane_cls.__name__}-{random.randint(100, 999)}"
        self.plane = plane_cls(plane_id)

        self.ground_ops = GroundOperations(runway_length=RUNWAY_LENGTH, num_gates=19)
        self.arrival_sim = ArrivalSimulator(self.arrival_route, self.plane)
        self.departure_sim = None

        self.state = "arrival"  # arrival -> gate_assignment -> at_gate -> taxi_runway -> runway_hold -> departure -> complete
        self.position = None
        self.trail = []
        self.sim_time = 0.0
        self.frame_dt = ARRIVAL_DT
        self.feed = TerminalFeed()

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
        print("\n" + "=" * 70)
        print("AUTONOMOUS SINGLE-AIRCRAFT TURNAROUND")
        print("=" * 70)
        print(f"Arrival route: {self.arrival_route['name']} ({self.arrival_route['description']})")
        print(f"  • Waypoints: {len(self.arrival_route['waypoints'])}")
        print(f"Departure route: {self.departure_route['name']} (heading {self.departure_route['heading']}°)")
        print(f"  • Waypoints: {len(self.departure_route['waypoints'])}")
        print(f"Aircraft: {self.plane.plane_type} ({self.plane.id})")
        print("Terminal feed mirrors arrival/departure sims for situational awareness.")
        print("=" * 70)

    def _log(self, message: str):
        print(f"[t={self.sim_time:5.1f}s] {message}")

    # ------------------------------------------------------------------
    # State machine
    # ------------------------------------------------------------------
    def step(self):
        """
        Advance the simulation by one animation frame.
        Returns the current 3D position of the aircraft (or None if hidden).
        """
        # Dynamic timestep: arrivals use their native DT, other phases use departure DT
        self.frame_dt = ARRIVAL_DT if self.state in ("arrival", "gate_assignment") else DEPARTURE_DT
        self.sim_time += self.frame_dt

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
                self.feed.announce_arrival(self.arrival_sim, pos)
            elif not self.final_speed_logged:
                print(f"Final approach speed: {self.arrival_sim.V:.1f} m/s (target: {FINAL_APPROACH_TARGET:.1f} m/s)")
                self.final_speed_logged = True

            # Check if aircraft is on final approach (last 2 segments before runway)
            on_final_approach = self.arrival_sim.segment >= len(self.arrival_sim.wps) - 2
            
            # Request landing clearance when on final approach
            if on_final_approach and not self.landing_busy_marked:
                clearance_granted = self.ground_ops.request_landing_clearance(
                    self.plane, 
                    self.sim_time, 
                    on_final_approach=True
                )
                
                if clearance_granted:
                    self.landing_busy_marked = True
                    self._log("ATC: Cleared to land. Runway now busy.")
                    self.feed.announce_clearance("Cleared to land.")
                    self.feed.runway_status(True, "Final approach + touchdown")
                    # Update arrival_sim clearance flag for display
                    self.arrival_sim.clearance_given = True
                else:
                    # Holding pattern - shouldn't happen with proper sequencing
                    if not self.runway_hold_logged:
                        self._log("Runway occupied → holding on final approach")
                        self.feed.announce_clearance("Runway occupied → hold on final.")
                        self.runway_hold_logged = True
            
            return self.position
        except StopIteration:
            # Landing complete - notify ground ops
            self.ground_ops.complete_landing(self.plane)
            self.state = "gate_assignment"
            self.feed.announce_clearance("Runway vacated, taxi clear.")
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
            self.feed.gate_event(f"{self.plane.id} service complete → taxi ({taxi_time:.1f}s)")
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
            self._log("Arrived at runway threshold, awaiting takeoff clearance")
            self.feed.announce_clearance("Holding short, awaiting takeoff clearance.")
            self.taxi_progress_reported = False
        else:
            self.position = self.ground_ops.gate_positions[self.gate_index]
            if not self.taxi_progress_reported:
                remaining = max(0.0, self.taxi_end_time - self.sim_time)
                self._log(f"Taxiing… {remaining:.1f}s remaining")
                self.taxi_progress_reported = True
        return self.position

    def _runway_queue(self):
        """Wait for a free runway, then start the departure simulator."""
        if not self.ground_ops.is_runway_available(self.sim_time):
            if not self.runway_hold_depart_logged:
                self._log("Runway occupied → holding short")
                self.feed.announce_clearance("Holding short — runway occupied.")
                self.runway_hold_depart_logged = True
            return self.position

        # Runway free -> start departure sim and block runway for takeoff roll
        self.departure_sim = DepartureSimulator(self.departure_route, self.plane)
        takeoff_run_time = max(self.departure_sim.ground_roll_data["t"][-1], 5.0) + 5.0
        self.ground_ops.mark_runway_busy(self.plane, takeoff_run_time, self.sim_time)
        self.state = "departure"
        self.takeoff_runway_released = False
        self.feed.announce_clearance("Cleared for takeoff.")
        self.feed.runway_status(True, f"Takeoff run for {takeoff_run_time:.1f}s")
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
            self.feed.announce_departure(self.departure_sim, pos)
            self.position = pos.copy()
            self.trail.append(pos.copy())

            # Release runway only once we have transitioned to climb
            if (
                not self.takeoff_runway_released
                and self.departure_sim.phase == "CLIMB"
                and self.ground_ops.runway_busy
            ):
                self.ground_ops.release_runway()
                self.feed.runway_status(False, "aircraft airborne")
                self._log("Runway freed — aircraft is airborne")
                self.takeoff_runway_released = True
            return self.position
        except StopIteration:
            self.state = "complete"
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


def run_visual_sim(max_frames: int = 2500, show: bool = True):
    sim = SingleAircraftTurnaround()

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

    # Arrival & departure planned routes
    arr_wps = sim.arrival_route["waypoints"]
    dep_wps = sim.departure_route["waypoints"]
    ax.plot(arr_wps[:, 0], arr_wps[:, 1], arr_wps[:, 2], "k--", alpha=0.35, label="Arrival route")
    ax.plot(dep_wps[:, 0], dep_wps[:, 1], dep_wps[:, 2], color="orange", linestyle="--", alpha=0.35, label="Departure route")

    # Aircraft marker and trail
    point, = ax.plot([], [], [], "ro", markersize=8, label="Aircraft")
    trail, = ax.plot([], [], [], color="blue", linewidth=2, alpha=0.8, label="Flown path")
    label = ax.text(0, 0, 0, "", fontsize=9, color="red", fontweight="bold")

    ax.set_xlim([-AIRSPACE_RADIUS, AIRSPACE_RADIUS])
    ax.set_ylim([-AIRSPACE_RADIUS, AIRSPACE_RADIUS])
    ax.set_zlim([0, max(arr_wps[:, 2].max(), dep_wps[:, 2].max(), 600)])
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Altitude (m)")
    ax.set_title("Autonomous Turnaround Simulation")
    ax.legend(loc="upper left")
    ax.grid(True, alpha=0.25)
    ax.view_init(elev=28, azim=-60)

    positions = []

    def init():
        point.set_data([], [])
        point.set_3d_properties([])
        trail.set_data([], [])
        trail.set_3d_properties([])
        label.set_text("")
        return point, trail, label

    def update(frame):
        pos = sim.step()
        if pos is not None:
            positions.append(pos)
            point.set_data([pos[0]], [pos[1]])
            point.set_3d_properties([pos[2]])
            label.set_position((pos[0], pos[1]))
            label.set_3d_properties(pos[2] + 25)
            label.set_text(f"{sim.plane.id}\n{sim.state_label}")
            path = np.array(positions)
            trail.set_data(path[:, 0], path[:, 1])
            trail.set_3d_properties(path[:, 2])
        else:
            point.set_data([], [])
            point.set_3d_properties([])
            label.set_text("")

        ax.set_title(
            f"Autonomous Turnaround | State: {sim.state_label} | t={sim.sim_time:.1f}s\n"
            f"Gates occupied: {len(sim.ground_ops.get_all_parked_aircraft())}/19 | "
            f"Runway: {'BUSY' if sim.ground_ops.runway_busy else 'FREE'}",
            fontsize=13,
            fontweight="bold",
        )

        if sim.state == "complete":
            ani.event_source.stop()
        return point, trail, label

    ani = animation.FuncAnimation(
        fig,
        update,
        frames=max_frames,
        init_func=init,
        interval=sim.frame_interval_ms,
        blit=True,
        repeat=False,
    )

    if show:
        plt.show()
    return fig, ani


def run_headless(max_frames: int = 2500):
    """
    Headless run for quick validation (no plotting).
    """
    sim = SingleAircraftTurnaround()
    for _ in range(max_frames):
        sim.step()
        if sim.state == "complete":
            break
    print("\nHeadless run finished.")
    print(f"Final state: {sim.state_label}")
    print(f"Total sim time: {sim.sim_time:.1f}s")


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
