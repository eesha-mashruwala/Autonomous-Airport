"""
Emergency Landing Anomaly Simulation (Final Version - Fixed)

Scenario:
1. Two aircraft spawn on RANDOM distinct arrival routes.
2. Plane A is EMERGENCY: Priorities landing, no holding.
3. Plane B is NORMAL: Detects Emergency plane.
   - If Emergency plane is in airspace or on runway -> Normal Plane HOLDS (loops).
   - Once Emergency plane vacates runway -> Normal Plane RESUMES approach.
4. Both planes continue through full turnaround (Gate -> Departure).

Performance:
- optimized with trail capping and speed-up multiplier.
- Overshoot protection: Stronger braking and lower landing speeds.
- Safe Holding: Plane holds at current altitude rather than climbing back up.
"""

import math
import random
import numpy as np
import matplotlib
matplotlib.use('TkAgg')  # Force interactive backend
import matplotlib.pyplot as plt
from matplotlib import animation

# --- IMPORT SIMULATION MODULES ---
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
    clamp
)
from arrival_routes import (
    generate_arrival_routes,
    RUNWAY_THRESHOLD,
    RUNWAY_END,
    AIRSPACE_RADIUS,
)
from departure_live_sim import ANIMATION_DT as DEPARTURE_DT
from departure_routes import generate_departure_routes
from ground_operations import GroundOperations, RUNWAY_LENGTH
from aircraft import EmbraerE170, AirbusA220, Dash8_400, ATR72_600
from autonomous_turnaround_sim import (
    TerminalFeed, 
    AircraftInfoWindow, 
    SingleAircraftTurnaround,
    FLEET,
    SERVICE_TIME_RANGE,
    FORCED_TAXI_TIME
)
from flight_dynamics import FlightDynamics # Explicit import needed for rollout

# --- CONFIGURATION ---
TRAIL_LENGTH = 750       # Keep paths short to prevent lag
SIM_SPEED_UP = 1         # Physics steps per frame (Higher = Faster)

class EmergencyArrivalSimulator(ArrivalSimulator):
    """
    Custom Arrival Simulator that allows an aircraft to enter a holding pattern
    by oscillating between its current and previous waypoints.
    """
    def __init__(self, route, plane, verbose=True):
        super().__init__(route, plane, verbose)
        self.hold_active = False
        self.hold_state = "FORWARD"  # 'FORWARD' to next WP, 'BACKWARD' to prev WP
        
    def set_hold(self, active: bool):
        """Enable or disable holding pattern."""
        if self.hold_active == active:
            return
            
        self.hold_active = active
        if active:
            print(f"SIM | {self.plane.id} >> ENTERING HOLDING PATTERN")
        else:
            print(f"SIM | {self.plane.id} >> RESUMING APPROACH")
            self.hold_state = "FORWARD"

    def _start_rollout(self):
        """Calculates ground roll path with visual clamping."""
        if self.verbose:
            print(f"{self.plane.id} Touchdown. Beginning rollout at {self.V:.1f} m/s")
        
        rollout_dyn = FlightDynamics(self.plane)
        rollout_dyn.reset_state()
        
        # INCREASED BRAKING FORCE: T_reverse raised to 80,000N (from 50k)
        rollout_dyn.ground_roll_landing(dt=ARRIVAL_DT, V_touchdown=self.V, T_reverse=80000.0)
        
        self.rollout_points = []
        for entry in rollout_dyn.telemetry():
            x_offset = entry["x"]
            
            # VISUAL CLAMP: Do not allow drawing past the runway end
            if x_offset > RUNWAY_LENGTH - 20: 
                x_offset = RUNWAY_LENGTH - 20 # Stop just before end
                self.rollout_points.append(np.array([RUNWAY_THRESHOLD[0] + x_offset, 0.0, 0.0]))
                break # Stop generating points
                
            self.rollout_points.append(np.array([RUNWAY_THRESHOLD[0] + x_offset, 0.0, 0.0]))
            
        if not self.rollout_points:
            self.rollout_points.append(RUNWAY_THRESHOLD.copy())

    def __next__(self):
        """
        Advance physics by one step. Handles holding logic vs normal approach.
        """
        if self.finished:
            raise StopIteration

        # 1. LANDING ROLLOUT (No holding allowed here)
        if self.landed:
            if self.rollout_index < len(self.rollout_points):
                pos = self.rollout_points[self.rollout_index]
                self.rollout_index += 1
                return pos
            self.finished = True
            raise StopIteration

        # 2. DETERMINE TARGET WAYPOINT
        if self.hold_active and self.segment > 0:
            if self.hold_state == "BACKWARD":
                # --- SAFE HOLDING LOGIC ---
                # Fly towards the X/Y of the previous waypoint...
                prev_wp = np.array(self.wps[self.segment], dtype=float)
                # ...BUT maintain the Z (altitude) of the current waypoint.
                curr_wp = np.array(self.wps[self.segment + 1], dtype=float)
                
                target = prev_wp.copy()
                target[2] = curr_wp[2] # FORCE FLAT ALTITUDE
            else:
                target = np.array(self.wps[self.segment + 1], dtype=float)
        else:
            target = np.array(self.wps[self.segment + 1], dtype=float)

        # 3. DETERMINE TARGET SPEED
        if self.hold_active:
            target_speed = CRUISE_SPEED
        elif self.segment >= len(self.wps) - 3:
            target_speed = FINAL_SPEED
        elif self.segment >= len(self.wps) - 4:
            target_speed = APPROACH_SPEED
        else:
            target_speed = CRUISE_SPEED

        # 4. WAYPOINT DETECTION LOGIC
        vec = target - self.pos
        dist = math.sqrt(vec[0]**2 + vec[1]**2 + vec[2]**2)
        
        # Check overshoot
        seg_vec = target - self.seg_start
        overshot = np.dot(vec, seg_vec) <= 0
        arrived = (dist < 30.0) or overshot

        if arrived:
            if self.hold_active and self.segment > 0:
                if self.hold_state == "FORWARD":
                    self.hold_state = "BACKWARD"
                else:
                    self.hold_state = "FORWARD"
                self.seg_start = target.copy()
            else:
                self.pos = target.copy()
                if self.segment == len(self.wps) - 2:
                    self.landed = True
                    self._start_rollout()
                else:
                    self.segment += 1
                    self.seg_start = target.copy()
            return self.pos

        # 5. PHYSICS UPDATE
        horiz = math.hypot(vec[0], vec[1])
        target_psi = math.atan2(vec[1], vec[0])
        target_gamma = math.atan2(vec[2], horiz if horiz > 1e-3 else 1e-3)

        psi_err = math.atan2(math.sin(target_psi - self.psi), math.cos(target_psi - self.psi))
        phi_cmd = clamp(HEADING_GAIN * psi_err, -MAX_BANK_RAD, MAX_BANK_RAD)
        
        self.psi += self.dyn.compute_heading_rate(self.V, phi_cmd) * ARRIVAL_DT
        self.gamma += clamp(GAMMA_GAIN * (target_gamma - self.gamma), -0.05, 0.05)

        speed_err = target_speed - self.V
        throttle_setting = clamp(0.3 + V_THROTTLE_GAIN * speed_err / target_speed, 0.0, 1.0)
        thrust = self.plane.compute_thrust(self.V) * throttle_setting
        drag = self.plane.compute_drag(self.V, self.gamma)
        
        dVdt = (thrust - drag - self.plane.mass * 9.81 * math.sin(self.gamma)) / self.plane.mass
        
        # ADJUSTMENT: Allow slower minimum speed on final approach to ensure stopping
        min_speed = 45.0 if self.segment >= len(self.wps) - 2 else 60.0
        self.V = max(min_speed, self.V + dVdt * ARRIVAL_DT)

        self.pos[0] += self.V * math.cos(self.gamma) * math.cos(self.psi) * ARRIVAL_DT
        self.pos[1] += self.V * math.cos(self.gamma) * math.sin(self.psi) * ARRIVAL_DT
        self.pos[2] += self.V * math.sin(self.gamma) * ARRIVAL_DT
        
        if not self.clearance_given and self.segment == len(self.wps) - 2 and not self.hold_active:
            self.clearance_given = True

        return self.pos.copy()


class EmergencyTurnaround(SingleAircraftTurnaround):
    """
    Wrapper for a single aircraft. Inherits standard turnaround logic
    but injects the EmergencyArrivalSimulator.
    """
    def __init__(self, ground_ops, feed, route, is_emergency=False):
        self.is_emergency = is_emergency
        
        self.arrival_route = route
        self.departure_route = random.choice(generate_departure_routes())
        
        plane_cls = AirbusA220 if is_emergency else EmbraerE170
        plane_id = f"{'EMERGENCY' if is_emergency else 'NORMAL'}-{random.randint(100, 999)}"
        self.plane = plane_cls(plane_id)

        self.ground_ops = ground_ops
        self.arrival_sim = EmergencyArrivalSimulator(self.arrival_route, self.plane, verbose=False)
        self.departure_sim = None

        self.state = "arrival"
        self.position = None
        self.trail = []
        self.sim_time = 0.0
        self.frame_dt = ARRIVAL_DT
        self.feed = feed
        self.info_window = AircraftInfoWindow(plane_id, enabled=False)

        # Flags
        self.landing_busy_marked = False
        self.final_speed_logged = False
        self.runway_hold_logged = False
        self.gate_wait_logged = False
        self.runway_hold_depart_logged = False
        self.takeoff_runway_released = False
        self.gate_index = None
        self.service_time = 0
        self.service_end_time = 0
        self.taxi_end_time = 0
        self.taxi_progress_reported = False
        
        self._log_intro()
        
    @property
    def label_text(self):
        status = self.state_label
        if self.state == "arrival" and getattr(self.arrival_sim, 'hold_active', False):
            status = "HOLDING (Waiting for Emergency)"
        prefix = ">>> EMERGENCY <<<" if self.is_emergency else "Normal Flight"
        return f"{prefix}\n{self.plane.id}\n{status}"


class EmergencyScenarioManager:
    def __init__(self):
        self.ground_ops = GroundOperations(runway_length=RUNWAY_LENGTH, num_gates=19)
        self.feed = TerminalFeed()
        self.actors = []
        self.global_time = 0.0
        self.dt = DEPARTURE_DT
        
        print("\n" + "="*60)
        print(" INITIALIZING EMERGENCY SCENARIO")
        print("="*60)
        
        all_routes = generate_arrival_routes(num_routes=20)
        selected_routes = random.sample(all_routes, 2)
        emerg_route = selected_routes[0]
        norm_route = selected_routes[1]
        
        print(f"Emergency Route: {emerg_route['name']}")
        print(f"Normal Route:    {norm_route['name']}")
        
        self.emergency_plane = EmergencyTurnaround(self.ground_ops, self.feed, emerg_route, is_emergency=True)
        self.normal_plane = EmergencyTurnaround(self.ground_ops, self.feed, norm_route, is_emergency=False)
        
        self.actors = [self.emergency_plane, self.normal_plane]

    def step(self):
        self.global_time += self.dt
        
        emerg_is_active = (self.emergency_plane.state == "arrival")
        norm_is_active = (self.normal_plane.state == "arrival")
        
        if norm_is_active:
            if emerg_is_active:
                if self.normal_plane.arrival_sim.segment > 1:
                    self.normal_plane.arrival_sim.set_hold(True)
            else:
                self.normal_plane.arrival_sim.set_hold(False)
        
        updates = []
        for actor in list(self.actors):
            pos = actor.step(self.global_time)
            updates.append((actor, pos))
            if actor.state == "complete":
                actor.info_window.close()
                self.actors.remove(actor)
        return updates


def run_emergency_sim():
    sim = EmergencyScenarioManager()

    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection="3d")

    ax.plot([RUNWAY_THRESHOLD[0], RUNWAY_END[0]], [RUNWAY_THRESHOLD[1], RUNWAY_END[1]], [RUNWAY_THRESHOLD[2], RUNWAY_END[2]],
        color="gray", linewidth=8, label="Runway", alpha=0.7)
    
    gate_positions = sim.ground_ops.gate_positions
    ax.scatter(gate_positions[:, 0], gate_positions[:, 1], gate_positions[:, 2], c="lightblue", s=80, marker="s", alpha=0.6)

    for actor in sim.actors:
        wps = actor.arrival_route['waypoints']
        color = 'red' if actor.is_emergency else 'blue'
        ax.plot(wps[:,0], wps[:,1], wps[:,2], color=color, linestyle=':', alpha=0.3)

    ax.set_xlim([-AIRSPACE_RADIUS, AIRSPACE_RADIUS])
    ax.set_ylim([-AIRSPACE_RADIUS, AIRSPACE_RADIUS])
    ax.set_zlim([0, 600])
    ax.set_title("EMERGENCY LANDING ANOMALY SIMULATION")
    ax.legend(loc='upper left')
    ax.view_init(elev=30, azim=-60)

    actor_artists = {}
    actor_paths = {}

    def update(frame):
        final_updates = []
        for _ in range(SIM_SPEED_UP):
            final_updates = sim.step()

        current_ids = {actor.plane.id for actor in sim.actors}
        for actor_id in list(actor_artists.keys()):
            if actor_id not in current_ids:
                for h in actor_artists[actor_id].values(): 
                    try: h.remove()
                    except: pass
                del actor_artists[actor_id]

        for actor, pos in final_updates:
            actor_id = actor.plane.id
            if actor_id not in actor_artists:
                color = "red" if actor.is_emergency else "blue"
                point, = ax.plot([], [], [], "o", color=color, markersize=10, label=actor_id)
                trail, = ax.plot([], [], [], color=color, linewidth=2, alpha=0.5)
                label = ax.text(0, 0, 0, "", fontsize=9, fontweight="bold", color=color)
                actor_artists[actor_id] = {"point": point, "trail": trail, "label": label}
                actor_paths[actor_id] = []

            art = actor_artists[actor_id]
            if pos is not None:
                actor_paths[actor_id].append(pos.copy())
                if len(actor_paths[actor_id]) > TRAIL_LENGTH:
                     actor_paths[actor_id] = actor_paths[actor_id][-TRAIL_LENGTH:]

                art["point"].set_data([pos[0]], [pos[1]])
                art["point"].set_3d_properties([pos[2]])
                art["label"].set_text(actor.label_text)
                art["label"].set_position((pos[0], pos[1]))
                art["label"].set_3d_properties(pos[2] + 40)
                
                path = np.array(actor_paths[actor_id])
                if len(path) > 0:
                    art["trail"].set_data(path[:, 0], path[:, 1])
                    art["trail"].set_3d_properties(path[:, 2])

        if sim.emergency_plane in sim.actors:
            emerg_state = sim.emergency_plane.state
            if emerg_state == "arrival": emerg_txt = "LANDING (Priority)"
            elif emerg_state == "gate_assignment": emerg_txt = "LANDED (Taxiing)"
            elif emerg_state == "at_gate": emerg_txt = "AT GATE"
            else: emerg_txt = "DEPARTING"
        else:
            emerg_txt = "GONE"
            
        if sim.normal_plane in sim.actors:
            is_holding = getattr(sim.normal_plane.arrival_sim, 'hold_active', False)
            norm_txt = "HOLDING" if is_holding else "APPROACHING"
        else:
            norm_txt = "GONE"
        
        ax.set_title(
            f"EMERGENCY SCENARIO (+{SIM_SPEED_UP}x Speed) | T={sim.global_time:.0f}s\n"
            f"Emergency: {emerg_txt} | Normal: {norm_txt}",
            fontweight='bold'
        )
        return []

    ani = animation.FuncAnimation(fig, update, interval=20, blit=False, cache_frame_data=False)
    plt.show()
    return ani

if __name__ == "__main__":
    anim = run_emergency_sim()