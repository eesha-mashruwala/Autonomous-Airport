"""
Departure Routes Module

Defines simple departure profiles for London City Airport sharing the same
5.5 km airspace scale used for arrivals. Each route:
- rolls along 75% of the runway before rotating,
- climbs straight ahead at 10° for 1.2 km on the runway heading,
- then splits into five headings (straight, ±15°, ±30°) while keeping
  the same 10° climb until it reaches the 5.5 km boundary.
"""

import math
import numpy as np
import matplotlib.pyplot as plt


# ------ AIRSPACE & RUNWAY CONSTANTS ----------------------------------------------------

AIRSPACE_RADIUS = 5500.0       # meters (matches arrival routes)
BOUNDARY_DISTANCE = AIRSPACE_RADIUS * 0.95

RUNWAY_LENGTH = 1508.0
RUNWAY_THRESHOLD = np.array([0.0, 0.0, 0.0])
RUNWAY_END = np.array([RUNWAY_LENGTH, 0.0, 0.0])

TAKEOFF_ROLL_DISTANCE = RUNWAY_LENGTH * 0.75  # meters before rotation
UNIVERSAL_CLIMB_DISTANCE = 1200.0  # meters travelled while climbing straight ahead
CLIMB_ANGLE_DEG = 10.0         # steady 10° climb
CLIMB_SLOPE = math.tan(math.radians(CLIMB_ANGLE_DEG))

STRAIGHT_CLIMB_DISTANCE = max(0.0, min(TAKEOFF_ROLL_DISTANCE + UNIVERSAL_CLIMB_DISTANCE, BOUNDARY_DISTANCE) - TAKEOFF_ROLL_DISTANCE)
TURN_DISTANCE = max(0.0, BOUNDARY_DISTANCE - min(TAKEOFF_ROLL_DISTANCE + UNIVERSAL_CLIMB_DISTANCE, BOUNDARY_DISTANCE))
MAX_CLIMB_DISTANCE = STRAIGHT_CLIMB_DISTANCE + TURN_DISTANCE

DEPARTURE_HEADINGS = [0.0, 15.0, 30.0, -15.0, -30.0]  # degrees relative to +X


# ------ ROUTE GENERATION --------------------------------------------------------------

def generate_departure_routes():
    """
    Create five departure profiles.
    Returns a list of route dictionaries.
    """
    routes = []
    
    for idx, heading in enumerate(DEPARTURE_HEADINGS):
        route_name = f"DEP{idx+1:02d}"
        waypoints = _generate_route_waypoints(heading)
        route = {
            "name": route_name,
            "heading": heading,
            "waypoints": waypoints
        }
        routes.append(route)
    
    return routes


def _generate_route_waypoints(heading_deg):
    """
    Build waypoints for a single departure route.
    """
    positions = []
    
    # Start at runway threshold
    positions.append(RUNWAY_THRESHOLD.tolist())
    
    # Ground roll along runway (no climb yet)
    positions.append([TAKEOFF_ROLL_DISTANCE, 0.0, 0.0])
    
    # Universal 1.2 km climb straight ahead (still on runway heading)
    straight_end = TAKEOFF_ROLL_DISTANCE + UNIVERSAL_CLIMB_DISTANCE
    straight_end = min(straight_end, BOUNDARY_DISTANCE)
    positions.append([straight_end, 0.0, 0.0])
    
    if straight_end >= BOUNDARY_DISTANCE:
        # already at boundary; no additional turn segment
        return np.array(_apply_climb_profile(positions))
    
    # Second segment: continue climbing while turning to heading_deg until boundary
    heading_rad = math.radians(heading_deg)
    remaining_distance = BOUNDARY_DISTANCE - straight_end
    target_x = straight_end + remaining_distance * math.cos(heading_rad)
    target_y = remaining_distance * math.sin(heading_rad)
    
    positions.append([target_x, target_y, 0.0])
    
    return np.array(_apply_climb_profile(positions))


def _apply_climb_profile(positions):
    waypoints = []
    cumulative_distance = 0.0
    prev_x, prev_y = positions[0][0], positions[0][1]
    
    for idx, (x, y, _) in enumerate(positions):
        if idx == 0:
            altitude = 0.0
        else:
            leg_distance = math.hypot(x - prev_x, y - prev_y)
            cumulative_distance += leg_distance
            climb_distance = max(0.0, cumulative_distance - TAKEOFF_ROLL_DISTANCE)
            altitude = CLIMB_SLOPE * climb_distance
        waypoints.append([x, y, altitude])
        prev_x, prev_y = x, y
    
    return waypoints


# ------ VISUALIZATION --------------------------------------------------------------

def plot_departure_routes_3d(routes):
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    colors = plt.cm.tab10(np.linspace(0, 1, len(routes)))
    
    for route, color in zip(routes, colors):
        wps = route["waypoints"]
        ax.plot(wps[:, 0], wps[:, 1], wps[:, 2],
                marker='o', linewidth=2, label=route["name"], color=color)
        ax.scatter(wps[0, 0], wps[0, 1], wps[0, 2],
                   color=color, marker='s', s=100, edgecolors='k')
    
    # Runway depiction
    ax.plot([RUNWAY_THRESHOLD[0], RUNWAY_END[0]],
            [RUNWAY_THRESHOLD[1], RUNWAY_END[1]],
            [RUNWAY_THRESHOLD[2], RUNWAY_END[2]],
            color='black', linewidth=6, label='Runway')
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('Departure Routes (3D)')
    ax.set_xlim([-AIRSPACE_RADIUS, AIRSPACE_RADIUS])
    ax.set_ylim([-AIRSPACE_RADIUS, AIRSPACE_RADIUS])
    max_altitude = CLIMB_SLOPE * MAX_CLIMB_DISTANCE
    ax.set_zlim([0, max_altitude])
    ax.set_box_aspect((2*AIRSPACE_RADIUS, 2*AIRSPACE_RADIUS, max_altitude))
    ax.legend(loc='upper left', bbox_to_anchor=(1.02, 1))
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    return fig, ax


def plot_departure_routes_top_view(routes):
    fig, ax = plt.subplots(figsize=(10, 10))
    colors = plt.cm.tab10(np.linspace(0, 1, len(routes)))
    
    for route, color in zip(routes, colors):
        wps = route["waypoints"]
        ax.plot(wps[:, 0], wps[:, 1], marker='o', linewidth=2,
                color=color, label=route["name"])
    
    # Departure wedge (±45°)
    wedge = plt.matplotlib.patches.Wedge(
        (0, 0), AIRSPACE_RADIUS, -45, 45,
        facecolor='red', alpha=0.1, edgecolor='red', linestyle='--',
        label='Departure Sector'
    )
    ax.add_patch(wedge)
    
    # Boundary circle
    circle = plt.Circle((0, 0), AIRSPACE_RADIUS, fill=False,
                        linestyle='--', linewidth=1.5, color='gray')
    ax.add_patch(circle)
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Departure Routes (Top View)')
    ax.set_xlim([-AIRSPACE_RADIUS, AIRSPACE_RADIUS])
    ax.set_ylim([-AIRSPACE_RADIUS, AIRSPACE_RADIUS])
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper left', bbox_to_anchor=(1.02, 1))
    plt.tight_layout()
    return fig, ax


def print_departure_summary(routes):
    print("\n" + "="*70)
    print("DEPARTURE ROUTES SUMMARY")
    print("="*70)
    print(f"Airspace Radius: {AIRSPACE_RADIUS/1000:.2f} km")
    print(f"Ground Roll Before Rotation: {TAKEOFF_ROLL_DISTANCE:.0f} m")
    print(f"Straight Climb Before Turn: {STRAIGHT_CLIMB_DISTANCE:.0f} m")
    print(f"Turn/Extension Distance: {TURN_DISTANCE:.0f} m")
    print(f"Climb Angle: {CLIMB_ANGLE_DEG:.1f}°")
    print("-"*70)
    print(f"{'Route':<8} {'Heading':<10} {'Exit Alt (m)':<12}")
    print("-"*70)
    
    for route in routes:
        exit_alt = route["waypoints"][-1, 2]
        print(f"{route['name']:<8} {route['heading']:>6.1f}°     {exit_alt:>8.1f}")
    
    print("-"*70)
    print()


# ------ MAIN TEST HARNESS --------------------------------------------------------------

if __name__ == "__main__":
    routes = generate_departure_routes()
    print_departure_summary(routes)
    fig3d, ax3d = plot_departure_routes_3d(routes)
    fig2d, ax2d = plot_departure_routes_top_view(routes)
    plt.show()
