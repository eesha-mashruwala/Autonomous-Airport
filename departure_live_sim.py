"""
Live departure simulation using departure_routes and flight_dynamics ODE helpers.

- Picks a random departure route (5.5 km radius) and a random aircraft type.
- Spawns at the runway threshold, follows the ground roll segment using realistic
  flight dynamics ODEs, then climbs and turns per the route waypoints.
- Shows live 3D animation with the runway, planned route, flown path, and an ID tag
  following the aircraft. The aircraft is removed when it reaches the 5.5 km boundary.
"""

import math
import random
import json
from datetime import datetime
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation

from departure_routes import generate_departure_routes, AIRSPACE_RADIUS, RUNWAY_THRESHOLD, RUNWAY_END
from aircraft import EmbraerE170, AirbusA220, Dash8_400, ATR72_600
from flight_dynamics import FlightDynamics

# Animation timestep
ANIMATION_DT = 0.1

# Flight parameters
VR = 70.0  # Rotation speed (m/s)
V_CLIMB_TARGET = 85.0  # Target climb speed (m/s)
GAMMA_CLIMB = math.radians(5)  # Climb angle (5 degrees)
HEADING_GAIN = 2.5
MAX_BANK_RAD = math.radians(30)


def clamp(val, lo, hi):
    return max(lo, min(hi, val))


class DepartureSimulator:
    """
    Iterator yielding live positions as the aircraft flies the departure route.
    Uses realistic flight dynamics ODEs for takeoff and climb.
    """

    def __init__(self, route, plane):
        self.route = route
        self.plane = plane
        self.wps = route["waypoints"]
        self.finished = False
        
        # Flight phases
        self.phase = "GROUND_ROLL"  # GROUND_ROLL -> CLIMB -> CRUISE
        
        # Initialize flight dynamics with proper ODE simulation
        self.dyn = FlightDynamics(plane)
        
        # Run ground roll simulation using flight_dynamics
        print(f"\n{'='*60}")
        print(f"Starting ground roll for {plane.plane_type} ({plane.id})")
        print(f"Target rotation speed: {VR} m/s")
        print(f"{'='*60}")
        
        takeoff_result = self.dyn.ground_roll_takeoff(dt=0.05, Vr=VR, max_time=120.0)
        self.ground_roll_data = self.dyn.get_history()
        
        print(f"✓ Takeoff complete: V={takeoff_result['V_rot']:.1f} m/s, "
              f"distance={takeoff_result['distance']:.1f} m, time={takeoff_result['time']:.1f} s")
        
        # Set up initial heading based on route
        self.psi = math.atan2(self.wps[1][1] - self.wps[0][1],
                              self.wps[1][0] - self.wps[0][0])
        
        # Initialize position tracking
        self.pos = np.array([0.0, 0.0, 0.0])
        self.V = 0.0
        self.gamma = 0.0
        self.playback_index = 0
        
        # Storage for animation
        self.positions = []
        self.time_hist = []
        self.speed_hist = []
        self.altitude_hist = []
        
        # Trajectory data for JSON export
        self.trajectory_data = {
            "aircraft_id": plane.id,
            "aircraft_type": plane.plane_type,
            "route_name": route["name"],
            "simulation_timestamp": datetime.now().isoformat(),
            "trajectory": []  # Will store {"t": time, "x": x, "y": y, "z": z, "V": speed}
        }
        
        # Waypoint tracking
        self.segment = 0
        self.seg_start = self.pos.copy()
        
        # Climb phase state
        self.climb_started = False
        self.climb_time = 0.0

    def __iter__(self):
        return self

    def __next__(self):
        if self.finished:
            raise StopIteration

        # =================== PHASE 1: GROUND ROLL ===================
        if self.phase == "GROUND_ROLL":
            if self.playback_index < len(self.ground_roll_data['t']):
                # Playback pre-computed ground roll
                self.pos[0] = self.ground_roll_data['x'][self.playback_index]
                self.pos[1] = 0.0
                self.pos[2] = 0.0
                self.V = self.ground_roll_data['V'][self.playback_index]
                current_time = self.ground_roll_data['t'][self.playback_index]
                
                self.positions.append(self.pos.copy())
                self.time_hist.append(current_time)
                self.speed_hist.append(self.V)
                self.altitude_hist.append(0.0)
                
                self.playback_index += 1
                
                # Check if we reached rotation speed - IMMEDIATELY transition to climb
                if self.V >= VR * 0.99 and not self.climb_started:
                    print(f"\n✓ Rotation! V={self.V:.1f} m/s at x={self.pos[0]:.0f}m, transitioning to CLIMB")
                    self.phase = "CLIMB"
                    self.climb_started = True
                    
                    # Initialize climb dynamics first
                    self.dyn = FlightDynamics(self.plane)
                    self.dyn.V = self.V
                    self.dyn.x = self.pos[0]
                    self.dyn.y = self.pos[1]
                    self.dyn.z = self.pos[2]
                    self.dyn.psi = self.psi
                    # Start with initial climb angle for rotation
                    self.dyn.gamma = math.radians(8)  # Initial rotation climb angle
                    
                    # Determine which waypoint to target for smoothest transition
                    # Look ahead to find the best waypoint to target
                    best_segment = 0
                    min_turn_angle = float('inf')
                    
                    for i in range(len(self.wps) - 1):
                        target = self.wps[i + 1]
                        vec = np.array(target) - self.pos
                        horiz_dist = math.hypot(vec[0], vec[1])
                        
                        # Skip waypoints behind us
                        if vec[0] < 0:
                            continue
                        
                        target_heading = math.atan2(vec[1], vec[0])
                        turn_angle = abs(math.atan2(math.sin(target_heading - self.psi),
                                                     math.cos(target_heading - self.psi)))
                        
                        # Prefer waypoints ahead with smaller turn angles
                        if turn_angle < min_turn_angle and horiz_dist > 100:
                            min_turn_angle = turn_angle
                            best_segment = i
                    
                    self.segment = best_segment
                    self.seg_start = self.pos.copy()
                    print(f"  → Targeting waypoint {self.segment + 1} for smoothest climb (turn angle: {math.degrees(min_turn_angle):.1f}°)")
                    
                    # Don't return here - continue immediately to climb phase
                else:
                    # Still in ground roll, return current position
                    return self.pos.copy()
            else:
                # Fallback if ground roll completes without rotation
                self.phase = "CLIMB"
                self.climb_started = True
                self.segment = 0
                self.seg_start = self.pos.copy()
                self.dyn.gamma = math.radians(8)
                print(f"\n✓ Ground roll complete, transitioning to CLIMB from segment {self.segment}")

        # =================== PHASE 2: CLIMB ===================
        if self.phase == "CLIMB":
            # Boundary check - exit when reaching airspace limit
            if np.hypot(self.pos[0], self.pos[1]) >= AIRSPACE_RADIUS:
                print(f"✓ Reached airspace boundary at {np.hypot(self.pos[0], self.pos[1]):.0f}m")
                self.finished = True
                raise StopIteration
            
            # Check if completed all waypoints
            if self.segment >= len(self.wps) - 1:
                # Continue flying current heading to boundary with gradual level off
                if self.dyn.gamma > 0.01:
                    self.dyn.gamma -= 0.01  # Gradually level off
                
                self.pos[0] += self.dyn.V * math.cos(self.dyn.gamma) * math.cos(self.dyn.psi) * ANIMATION_DT
                self.pos[1] += self.dyn.V * math.cos(self.dyn.gamma) * math.sin(self.dyn.psi) * ANIMATION_DT
                self.pos[2] += self.dyn.V * math.sin(self.dyn.gamma) * ANIMATION_DT
                
                # Record state
                current_time = self.time_hist[-1] + ANIMATION_DT if self.time_hist else 0.0
                self.positions.append(self.pos.copy())
                self.time_hist.append(current_time)
                self.speed_hist.append(self.dyn.V)
                self.altitude_hist.append(self.pos[2])
                
                # Record trajectory point for JSON export
                self.trajectory_data["trajectory"].append({
                    "t": round(current_time, 2),
                    "x": round(self.pos[0], 2),
                    "y": round(self.pos[1], 2),
                    "z": round(self.pos[2], 2),
                    "V": round(self.dyn.V, 2),
                    "gamma_deg": round(math.degrees(self.dyn.gamma), 2),
                    "psi_deg": round(math.degrees(self.dyn.psi), 2)
                })
                
                return self.pos.copy()

            # Get target waypoint (use a waypoint tracking loop similar to arrival sim)
            while True:
                target = np.array(self.wps[self.segment + 1], dtype=float)
                vec = target - self.pos
                seg_vec = target - self.seg_start
                horiz_dist = math.hypot(vec[0], vec[1])
                dist_3d = np.linalg.norm(vec)

                # Check if we overshot the waypoint (dot product test)
                if np.dot(vec, seg_vec) <= 0 and dist_3d > 50.0:
                    print(f"⚠ Overshot waypoint {self.segment+1}, advancing to next segment")
                    self.segment += 1
                    self.seg_start = self.pos.copy()
                    if self.segment >= len(self.wps) - 1:
                        break
                    continue

                # Check if reached waypoint (proximity test)
                if dist_3d < 50.0:
                    print(f"✓ Reached waypoint {self.segment+1}/{len(self.wps)-1}: "
                          f"pos=({target[0]:.0f}, {target[1]:.0f}, {target[2]:.0f})")
                    self.segment += 1
                    self.seg_start = target.copy()
                    if self.segment >= len(self.wps) - 1:
                        break
                    continue
                
                # Still tracking this waypoint
                break

            # If still have waypoints, compute target heading and climb angle
            if self.segment < len(self.wps) - 1:
                target = np.array(self.wps[self.segment + 1], dtype=float)
                vec = target - self.pos
                horiz_dist = math.hypot(vec[0], vec[1])
                
                # Proportional heading control (like arrival sim)
                target_psi = math.atan2(vec[1], vec[0])
                psi_err = math.atan2(math.sin(target_psi - self.dyn.psi), 
                                     math.cos(target_psi - self.dyn.psi))
                phi_cmd = clamp(HEADING_GAIN * psi_err, -MAX_BANK_RAD, MAX_BANK_RAD)
                self.dyn.psi += self.dyn.compute_heading_rate(self.dyn.V, phi_cmd) * ANIMATION_DT

                # Dynamic climb angle control
                # Calculate target gamma from waypoint geometry
                target_gamma = math.atan2(vec[2], horiz_dist if horiz_dist > 1e-3 else 1e-3)
                
                # If target waypoint is at or below current altitude, maintain minimum climb angle
                # This ensures continuous climb even when targeting ground-level waypoints
                MIN_CLIMB_ANGLE = math.radians(8)  # Maintain at least 8° climb
                if vec[2] <= 0:  # Target is at or below us
                    target_gamma = MIN_CLIMB_ANGLE
                else:
                    # Ensure we don't descend, maintain minimum positive climb
                    target_gamma = max(MIN_CLIMB_ANGLE * 0.5, target_gamma)
                
                # Clamp target gamma to reasonable climb limits
                target_gamma = clamp(target_gamma, math.radians(3), math.radians(15))
                
                # Use proportional control with gain
                GAMMA_GAIN = 0.6
                gamma_err = target_gamma - self.dyn.gamma
                self.dyn.gamma += clamp(GAMMA_GAIN * gamma_err, -0.03, 0.03)

            # Speed control with adaptive throttle for continuous acceleration
            # Keep high throttle to maintain acceleration throughout climb
            if self.dyn.V < V_CLIMB_TARGET:
                throttle = 0.95  # High thrust for acceleration to target
            elif self.dyn.V < V_CLIMB_TARGET + 20.0:
                throttle = 0.85  # Continue accelerating beyond target
            else:
                throttle = 0.75  # Maintain speed at higher velocities
            
            thrust = self.plane.compute_thrust(self.dyn.V) * throttle
            dVdt = self.dyn.compute_airspeed_derivative(self.dyn.V, self.dyn.gamma, thrust)
            self.dyn.V = max(50.0, self.dyn.V + dVdt * ANIMATION_DT)

            # Update position using flight dynamics (continuous integration, no snapping)
            self.pos[0] += self.dyn.V * math.cos(self.dyn.gamma) * math.cos(self.dyn.psi) * ANIMATION_DT
            self.pos[1] += self.dyn.V * math.cos(self.dyn.gamma) * math.sin(self.dyn.psi) * ANIMATION_DT
            self.pos[2] += self.dyn.V * math.sin(self.dyn.gamma) * ANIMATION_DT

            # Record state
            current_time = self.time_hist[-1] + ANIMATION_DT if self.time_hist else 0.0
            self.positions.append(self.pos.copy())
            self.time_hist.append(current_time)
            self.speed_hist.append(self.dyn.V)
            self.altitude_hist.append(self.pos[2])
            
            # Record trajectory point for JSON export
            self.trajectory_data["trajectory"].append({
                "t": round(current_time, 2),
                "x": round(self.pos[0], 2),
                "y": round(self.pos[1], 2),
                "z": round(self.pos[2], 2),
                "V": round(self.dyn.V, 2),
                "gamma_deg": round(math.degrees(self.dyn.gamma), 2),
                "psi_deg": round(math.degrees(self.dyn.psi), 2)
            })
            
            self.climb_time += ANIMATION_DT
            
            # Periodic debug output (every 2 seconds)
            if int(current_time) % 2 == 0 and abs(current_time - int(current_time)) < ANIMATION_DT:
                if self.segment < len(self.wps) - 1:
                    target = self.wps[self.segment + 1]
                    dist_to_wp = np.linalg.norm(target - self.pos)
                    print(f"t={current_time:.0f}s | WP{self.segment+1} | "
                          f"Pos=({self.pos[0]:.0f},{self.pos[1]:.0f},{self.pos[2]:.0f}) | "
                          f"V={self.dyn.V:.1f} m/s | γ={math.degrees(self.dyn.gamma):.1f}° | "
                          f"ψ={math.degrees(self.dyn.psi):.0f}° | dist={dist_to_wp:.0f}m")

            return self.pos.copy()

        # Should never reach here
        self.finished = True
        raise StopIteration


def animate_live(route, simulator):
    """
    Create live 3D animation of departure showing ground roll, rotation, and climb.
    """
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    wps = route["waypoints"]
    ax.plot(wps[:, 0], wps[:, 1], wps[:, 2], 'k--', alpha=0.4, linewidth=2, label='Planned Route')
    ax.plot([RUNWAY_THRESHOLD[0], RUNWAY_END[0]],
            [RUNWAY_THRESHOLD[1], RUNWAY_END[1]],
            [RUNWAY_THRESHOLD[2], RUNWAY_END[2]],
            color='gray', linewidth=8, label='Runway', alpha=0.7)

    # Aircraft marker and trail
    point, = ax.plot([], [], [], 'ro', markersize=8, label='Aircraft')
    label = ax.text(0, 0, 0, "", fontsize=9, color='red', fontweight='bold')
    trail, = ax.plot([], [], [], color='blue', linewidth=2.5, alpha=0.8, label='Flown Path')
    
    # Track if aircraft has been removed
    aircraft_visible = [True]  # Use list to allow modification in nested function

    # Set view limits
    ax.set_xlim([-AIRSPACE_RADIUS, AIRSPACE_RADIUS])
    ax.set_ylim([-AIRSPACE_RADIUS, AIRSPACE_RADIUS])
    ax.set_zlim([0, max(wps[:, 2].max(), 1000)])
    ax.set_title(f"Live Departure - {route['name']} | {simulator.plane.plane_type}", 
                 fontsize=14, fontweight='bold')
    ax.set_xlabel("X (m)", fontsize=11)
    ax.set_ylabel("Y (m)", fontsize=11)
    ax.set_zlabel("Altitude (m)", fontsize=11)
    ax.legend(loc='upper left', fontsize=9)
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
        
        # Check if aircraft has reached boundary
        dist_from_origin = math.hypot(pos[0], pos[1])
        if dist_from_origin >= AIRSPACE_RADIUS and aircraft_visible[0]:
            # Remove aircraft marker and label
            point.set_data([], [])
            point.set_3d_properties([])
            label.set_text("")
            aircraft_visible[0] = False
        elif aircraft_visible[0]:
            # Update aircraft position
            point.set_data([pos[0]], [pos[1]])
            point.set_3d_properties([pos[2]])
            
            # Label shows aircraft ID and current phase
            phase_text = simulator.phase
            speed_text = f"V={simulator.V:.0f} m/s" if hasattr(simulator, 'V') else ""
            label.set_position((pos[0], pos[1]))
            label.set_3d_properties(pos[2] + 30)
            label.set_text(f"{simulator.plane.id}\n{phase_text}\n{speed_text}")
        
        # Always update trail (keeps the path visible)
        path = np.array(positions)
        trail.set_data(path[:, 0], path[:, 1])
        trail.set_3d_properties(path[:, 2])
        return point, trail, label

    ani = animation.FuncAnimation(
        fig, update, frames=simulator, init_func=init,
        interval=ANIMATION_DT * 1000, blit=True, repeat=False
    )
    return fig, ani


if __name__ == "__main__":
    print("\n" + "="*60)
    print("LIVE DEPARTURE SIMULATION")
    print("="*60)
    
    # Select random route and aircraft
    routes = generate_departure_routes()
    route = random.choice(routes)
    fleet = [EmbraerE170, AirbusA220, Dash8_400, ATR72_600]
    plane_cls = random.choice(fleet)
    plane_id = f"{plane_cls.__name__}-{random.randint(100, 999)}"
    plane = plane_cls(plane_id)
    
    print(f"Route: {route['name']} (heading {route['heading']}°)")
    print(f"Aircraft: {plane.plane_type} ({plane.id})")
    print(f"Mass: {plane.mass:.0f} kg | Thrust: {plane._T0:.0f} N | T/W: {plane._T0/(plane.mass*9.81):.3f}")
    print(f"\nRoute waypoints ({len(route['waypoints'])} total):")
    for i, wp in enumerate(route['waypoints']):
        print(f"  WP{i}: ({wp[0]:.0f}, {wp[1]:.0f}, {wp[2]:.0f})")
    
    # Create simulator
    simulator = DepartureSimulator(route, plane)

    # Run animation
    print("\n" + "="*60)
    print("Starting live animation...")
    print("="*60)
    fig, ani = animate_live(route, simulator)
    plt.show()
    
    # Save trajectory data to JSON
    json_filename = f"departure_trajectory_{plane.id}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    with open(json_filename, 'w') as f:
        json.dump(simulator.trajectory_data, f, indent=2)
    print(f"\n✓ Trajectory data saved to: {json_filename}")

    # Plot performance after simulation completes
    print("\n" + "="*60)
    print("Generating performance plots...")
    print("="*60)
    
    # Speed vs Time
    fig, axes = plt.subplots(2, 1, figsize=(10, 8))
    
    ax1 = axes[0]
    ax1.plot(simulator.time_hist, simulator.speed_hist, 'b-', linewidth=2, label='Actual Speed')
    ax1.axhline(VR, color='red', linestyle='--', linewidth=1.5, label=f'Vr = {VR} m/s', alpha=0.7)
    ax1.axhline(V_CLIMB_TARGET, color='green', linestyle='--', linewidth=1.5, 
                label=f'Target = {V_CLIMB_TARGET} m/s', alpha=0.7)
    ax1.set_xlabel("Time (s)", fontsize=11)
    ax1.set_ylabel("Speed (m/s)", fontsize=11)
    ax1.set_title(f"Departure Speed Profile - {plane.plane_type} ({plane.id})", 
                  fontsize=12, fontweight='bold')
    ax1.legend(fontsize=10)
    ax1.grid(True, alpha=0.3)
    
    # Altitude vs Time
    ax2 = axes[1]
    ax2.plot(simulator.time_hist, simulator.altitude_hist, 'g-', linewidth=2, label='Altitude')
    ax2.set_xlabel("Time (s)", fontsize=11)
    ax2.set_ylabel("Altitude (m)", fontsize=11)
    ax2.set_title("Climb Profile", fontsize=12, fontweight='bold')
    ax2.legend(fontsize=10)
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()
    
    print("\n" + "="*60)
    print("SIMULATION COMPLETE")
    print(f"Final Speed: {simulator.speed_hist[-1]:.1f} m/s")
    print(f"Final Altitude: {simulator.altitude_hist[-1]:.1f} m")
    print(f"Total Time: {simulator.time_hist[-1]:.1f} s")
    print("="*60)
