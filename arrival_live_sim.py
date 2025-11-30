"""
Live arrival simulation controlled by the ODE helpers in flight_dynamics.

- Randomly selects one of the 20 predefined arrival routes (same 5.5 km scale).
- Spawns an EmbraerE170 from aircraft.py at the entry waypoint.
- Each animation frame advances the aircraft using compute_heading_rate and
  compute_airspeed_derivative so the red dot you see is the real-time solution.
- Only a single live 3D plot is shown; close the window to end the run.
"""

import math
import random
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation

from arrival_routes import (
    generate_arrival_routes,
    RUNWAY_THRESHOLD,
    RUNWAY_END,
    AIRSPACE_RADIUS,
)
from aircraft import EmbraerE170, AirbusA220, Dash8_400, ATR72_600
from flight_dynamics import FlightDynamics

# ---------------------- SIMULATION PARAMETERS ------------------------------
DT = 0.2                 # s timestep for live animation
CRUISE_SPEED = 85.0      # m/s before final approach
APPROACH_SPEED = 75.0    # m/s on approach
FINAL_SPEED = 65.0       # m/s for landing (touchdown speed)
HEADING_GAIN = 2.4       # proportional heading -> bank
GAMMA_GAIN = 0.8         # proportional vertical controller
MAX_BANK_RAD = math.radians(55)
V_THROTTLE_GAIN = 1.2    # throttle gain to hold target speed (increased for better control)


def clamp(val, lo, hi):
    return max(lo, min(hi, val))


class ArrivalSimulator:
    """Iterator providing live aircraft positions as it flies the route."""

    def __init__(self, route, plane, verbose=True):
        self.route = route
        self.plane = plane
        self.dyn = FlightDynamics(plane)
        self.dyn.reset_state()
        self.wps = route["waypoints"]
        self.pos = np.array(self.wps[0], dtype=float)
        self.seg_start = self.pos.copy()
        self.psi = math.atan2(self.wps[1][1] - self.wps[0][1],
                              self.wps[1][0] - self.wps[0][0])
        self.gamma = 0.0
        self.V = CRUISE_SPEED
        self.segment = 0
        self.clearance_given = False
        self.landed = False
        self.rollout_points = []
        self.rollout_index = 0
        self.finished = False
        self.verbose = verbose
        
        # Track speed and time for plotting
        self.time_hist = []
        self.speed_hist = []
        self.current_time = 0.0
        
        # Track speed and time for plotting
        self.time_hist = []
        self.speed_hist = []
        self.current_time = 0.0

    def __iter__(self):
        return self

    def __next__(self):
        if self.finished:
            raise StopIteration

        if self.landed:
            if self.rollout_index < len(self.rollout_points):
                pos = self.rollout_points[self.rollout_index]
                self.rollout_index += 1
                return pos
            self.finished = True
            raise StopIteration

        if self.segment >= len(self.wps) - 1:
            self.finished = True
            raise StopIteration

        while True:
            target = np.array(self.wps[self.segment + 1], dtype=float)
            
            # Progressive speed reduction for landing
            # Last 3 segments: target 65 m/s (final approach - start slowing earlier)
            # 4th from last: target 75 m/s (approach speed)
            # Earlier segments: cruise at 85 m/s
            if self.segment >= len(self.wps) - 3:
                target_speed = FINAL_SPEED  # 65 m/s for final approach
            elif self.segment >= len(self.wps) - 4:
                target_speed = APPROACH_SPEED  # 75 m/s for approach
            else:
                target_speed = CRUISE_SPEED  # 85 m/s cruise

            vec = target - self.pos
            seg_vec = target - self.seg_start
            horiz = math.hypot(vec[0], vec[1])
            dist = math.sqrt(horiz**2 + vec[2]**2)

            if np.dot(vec, seg_vec) <= 0:
                if self.verbose:
                    print(f"Overshot waypoint {self.segment + 1}, retargeting next segment.")
                self.segment += 1
                self.seg_start = self.pos.copy()
                if self.segment >= len(self.wps) - 1:
                    self.landed = True
                    self._start_rollout()
                    return self.pos
                continue

            if dist < 5.0:
                self.pos = target.copy()
                if self.verbose:
                    print(f"Reached waypoint {self.segment + 1}/{len(self.wps)-1}")
                if self.segment == len(self.wps) - 2:
                    self.landed = True
                    self._start_rollout()
                else:
                    self.segment += 1
                    self.seg_start = target.copy()
                return self.pos
            break

        target_psi = math.atan2(vec[1], vec[0])
        target_gamma = math.atan2(vec[2], horiz if horiz > 1e-3 else 1e-3)

        psi_err = math.atan2(math.sin(target_psi - self.psi), math.cos(target_psi - self.psi))
        phi_cmd = clamp(HEADING_GAIN * psi_err, -MAX_BANK_RAD, MAX_BANK_RAD)
        self.psi += self.dyn.compute_heading_rate(self.V, phi_cmd) * DT

        self.gamma += clamp(GAMMA_GAIN * (target_gamma - self.gamma), -0.05, 0.05)

        # Dynamic speed control using throttle and drag
        # Use aggressive proportional control to reach target speed
        speed_err = target_speed - self.V
        
        # Very aggressive throttle control for proper deceleration
        # The aircraft needs to slow down significantly for landing
        speedbrake_multiplier = 1.0  # Normal drag
        
        if speed_err < -10.0:  # Way too fast - idle with speedbrakes effect
            throttle_setting = 0.0  # Full idle
            speedbrake_multiplier = 4.0  # Deploy speedbrakes for extra drag
        elif speed_err < -3.0:  # Too fast - go to idle
            throttle_setting = 0.0  # Idle thrust for drag deceleration
            speedbrake_multiplier = 3.5  # Moderate speedbrakes
        elif speed_err < 0:  # Slightly too fast - minimal thrust
            # Very low throttle when still above target
            throttle_setting = clamp(0.02 + 0.1 * speed_err / target_speed, 0.0, 0.08)
            speedbrake_multiplier = 2.5  # Light speedbrakes
        else:  # Too slow - add thrust proportionally
            throttle_setting = clamp(0.3 + V_THROTTLE_GAIN * speed_err / target_speed, 0.08, 0.85)
        
        thrust = self.plane.compute_thrust(self.V) * throttle_setting
        
        # Compute drag with speedbrake multiplier
        base_drag = self.plane.compute_drag(self.V, self.gamma)
        drag = base_drag * speedbrake_multiplier
        
        # Manually compute dV/dt with modified drag
        dVdt = (thrust - drag - self.plane.mass * 9.81 * math.sin(self.gamma)) / self.plane.mass
        self.V = max(60.0, self.V + dVdt * DT)
        
        # Record time and speed
        self.current_time += DT
        self.time_hist.append(self.current_time)
        self.speed_hist.append(self.V)

        self.pos[0] += self.V * math.cos(self.gamma) * math.cos(self.psi) * DT
        self.pos[1] += self.V * math.cos(self.gamma) * math.sin(self.psi) * DT
        self.pos[2] += self.V * math.sin(self.gamma) * DT

        # Live tracker output
        if self.verbose:
            print(
                f"Seg {self.segment+1}/{len(self.wps)-1} | Pos {self.pos} | "
                f"HDG {math.degrees(self.psi):.1f}° | Gamma {math.degrees(self.gamma):.2f}° "
                f"| V {self.V:.1f} m/s | Target WP {target}"
            )

        if not self.clearance_given and self.segment == len(self.wps) - 2:
            self.clearance_given = True
            if self.verbose:
                print("ATC: Cleared to land.")

        return self.pos.copy()

    def _start_rollout(self):
        if self.verbose:
            print("Touchdown. Beginning rollout...")
        rollout_dyn = FlightDynamics(self.plane)
        rollout_dyn.reset_state()
        # Use actual approach speed at touchdown, not the hardcoded FINAL_SPEED
        rollout_dyn.ground_roll_landing(dt=DT, V_touchdown=self.V, T_reverse=50000.0)
        self.rollout_points = []
        for entry in rollout_dyn.telemetry():
            x_offset = entry["x"]
            self.rollout_points.append(np.array([RUNWAY_THRESHOLD[0] + x_offset, 0.0, 0.0]))
            if self.verbose:
                print(f"Rollout: x={x_offset:.1f} m, V={entry['V']:.1f} m/s")
        if not self.rollout_points:
            self.rollout_points.append(RUNWAY_THRESHOLD.copy())
        if self.verbose:
            print("Rollout complete. Holding position for inspection.")


def animate_live(route, simulator):
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')

    wps = route["waypoints"]
    ax.plot(wps[:, 0], wps[:, 1], wps[:, 2], 'k--', alpha=0.4, label='Planned Route')
    ax.plot([RUNWAY_THRESHOLD[0], RUNWAY_END[0]],
            [RUNWAY_THRESHOLD[1], RUNWAY_END[1]],
            [RUNWAY_THRESHOLD[2], RUNWAY_END[2]],
            color='black', linewidth=6, label='Runway')

    point, = ax.plot([], [], [], 'ro', markersize=6, label='Aircraft')
    label = ax.text(0, 0, 0, "", fontsize=8, color='red')
    trail, = ax.plot([], [], [], color='blue', linewidth=2, label='Flown Path')

    ax.set_xlim([-AIRSPACE_RADIUS, AIRSPACE_RADIUS])
    ax.set_ylim([-AIRSPACE_RADIUS, AIRSPACE_RADIUS])
    ax.set_zlim([0, max(wps[:, 2].max(), 600)])
    ax.set_title(f"Live Arrival - {route['name']}")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.legend(loc='upper left')
    ax.grid(True, alpha=0.3)

    positions = []

    def init():
        point.set_data([], [])
        point.set_3d_properties([])
        label.set_position((0, 0))
        label.set_3d_properties(0)
        trail.set_data([], [])
        trail.set_3d_properties([])
        return point, trail, label

    def update(pos):
        positions.append(pos)
        point.set_data([pos[0]], [pos[1]])
        point.set_3d_properties([pos[2]])
        label.set_position((pos[0], pos[1]))
        label.set_3d_properties(pos[2] + 20)
        label.set_text(simulator.plane.id)
        path = np.array(positions)
        trail.set_data(path[:, 0], path[:, 1])
        trail.set_3d_properties(path[:, 2])
        return point, trail, label

    ani = animation.FuncAnimation(
        fig, update, frames=simulator, init_func=init,
        interval=DT * 1000, blit=True, repeat=False
    )
    return fig, ani


if __name__ == "__main__":
    routes = generate_arrival_routes(num_routes=20)
    route = random.choice(routes)
    fleet = [EmbraerE170, AirbusA220, Dash8_400, ATR72_600]
    plane_cls = random.choice(fleet)
    plane_id = f"{plane_cls.__name__}-{random.randint(100, 999)}"
    plane = plane_cls(plane_id)
    verbose = True  # Set to False to suppress prints
    if verbose:
        print(f"Selected route: {route['name']} ({route['description']})")
        print(f"Selected aircraft: {plane.plane_type} ({plane.id})")
    simulator = ArrivalSimulator(route, plane, verbose=verbose)

    fig, ani = animate_live(route, simulator)
    if verbose:
        print("Simulation running. Close the window to finish.")
    plt.show()
    
    # Plot speed vs time
    fig_speed = plt.figure(figsize=(10, 6))
    ax_speed = fig_speed.add_subplot(111)
    
    ax_speed.plot(simulator.time_hist, simulator.speed_hist, 'b-', linewidth=2, label='Actual Speed')
    ax_speed.axhline(FINAL_SPEED, color='r', linestyle='--', linewidth=2, 
                     label=f'Target Landing Speed ({FINAL_SPEED} m/s)', alpha=0.7)
    ax_speed.axhline(APPROACH_SPEED, color='orange', linestyle='--', linewidth=1.5, 
                     label=f'Approach Speed ({APPROACH_SPEED} m/s)', alpha=0.6)
    ax_speed.axhline(CRUISE_SPEED, color='green', linestyle='--', linewidth=1.5, 
                     label=f'Cruise Speed ({CRUISE_SPEED} m/s)', alpha=0.6)
    
    ax_speed.set_xlabel('Time (s)', fontsize=12)
    ax_speed.set_ylabel('Speed (m/s)', fontsize=12)
    ax_speed.set_title(f'Arrival Speed Profile - {plane.plane_type} ({plane.id})', 
                       fontsize=14, fontweight='bold')
    ax_speed.legend(loc='best', fontsize=10)
    ax_speed.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()
    
    if verbose:
        print(f"\nFinal approach speed: {simulator.speed_hist[-1]:.1f} m/s (target: {FINAL_SPEED} m/s)")
