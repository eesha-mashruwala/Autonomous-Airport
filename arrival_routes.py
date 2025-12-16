"""
Arrival Routes Module

Defines arrival routes (STAR - Standard Terminal Arrival Route) for London City Airport.
Each route consists of multiple waypoints that guide aircraft from the airspace boundary
to the final approach fix.

Airspace specifications:
- Radius: 5,500 m
- Maximum altitude: 500 m (entry altitude)
- Final approach fix (FAF): ~463 m from runway threshold, ~45 m altitude
- Approach angle: ~5.5 degrees
- Runway: 1,508m long, aligned with x-axis (0,0,0) at threshold
- Exclusion zone: ±45° from takeoff direction (opposite to landing)
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
import random


# ------ AIRSPACE CONSTANTS --------------------------------------------------------------

# Geometry scales (retain shape, bring everything closer)
AIRSPACE_RADIUS = 5500.0    # meters (5.5 km boundary)
BASE_AIRSPACE_RADIUS = 9500.0
GEOMETRY_SCALE = AIRSPACE_RADIUS / BASE_AIRSPACE_RADIUS

STANDARD_APPROACH_ANGLE_DEG = 5.5
MAX_ALTITUDE = 500.0
RUNWAY_LENGTH = 1508.0      # meters
RUNWAY_THRESHOLD = np.array([0.0, 0.0, 0.0])  # touchdown at runway start
RUNWAY_END = np.array([RUNWAY_LENGTH, 0.0, 0.0])  # far end of runway

# Final Approach Fix (FAF) - common to all routes
# Scaled distance keeps proportions while flattening slope consistently
FAF_DISTANCE = 800.0  # meters before threshold (back to original)
FAF_ALTITUDE = math.tan(math.radians(STANDARD_APPROACH_ANGLE_DEG)) * FAF_DISTANCE
FAF_POSITION = np.array([-FAF_DISTANCE, 0.0, FAF_ALTITUDE])

# Landing direction: positive x (planes land moving in +x direction)
# Takeoff direction: positive x (planes also takeoff moving in +x direction)
# Exclusion zone: ±45° around +x axis (where takeoff traffic climbs out)

# ------ ROUTE GENERATION --------------------------------------------------------------

def generate_arrival_routes(num_routes=20):
    """
    Generate arrival routes from airspace boundary to final approach fix.
    
    Routes avoid the departure zone (±45° from -x axis) and include:
    - Entry point at airspace boundary
    - Multiple waypoints creating a curved path
    - Final approach fix (common to all routes)
    - Runway threshold
    
    Returns
    -------
    list of dict
        Each route contains: name, waypoints (Nx3 array), description
    """
    
    routes = []
    
    # Define angular sectors for arrivals (avoiding departure zone)
    # Departures use 0° ± 45° (eastward, along +x axis)
    # Exclude -45° to +45° in arrival sector
    
    # Safe arrival sectors: roughly 45° to 315° (north, west, south)
    # Split into northern, western, and southern approaches
    
    for i in range(num_routes):
        route_name = f"STAR{i+1:02d}"
        
        # Cluster routes in ±90° fan around the extended centerline
        angle = np.radians(90 + (i * 180 / max(num_routes - 1, 1)))
        
        # Generate waypoints for this route
        approach_angle = STANDARD_APPROACH_ANGLE_DEG
        waypoints = generate_route_waypoints(angle, route_name, approach_angle)
        
        route = {
            "name": route_name,
            "waypoints": waypoints,
            "entry_angle": math.degrees(angle),
            "description": f"Arrival from {math.degrees(angle):.1f}°",
            "approach_angle": approach_angle
        }
        
        routes.append(route)
    
    return routes


def generate_route_waypoints(entry_angle, route_name, approach_angle_deg):
    """
    Generate waypoints for a single arrival route.
    
    Creates a curved path from airspace boundary to FAF with 4-6 waypoints.
    
    Parameters
    ----------
    entry_angle : float
        Angle in radians for entry point on airspace boundary
    route_name : str
        Name of the route
    approach_angle_deg : float
        Desired glide angle in degrees for this route
        
    Returns
    -------
    np.ndarray
        Nx3 array of waypoints [x, y, z]
    """
    
    waypoints = []
    
    # helper to smoothly steer every route toward the extended centreline
    def steer_toward_center(start_angle, factor):
        """
        Interpolate the current bearing toward the inbound (-x) direction.
        The wrap logic keeps the shortest turning direction for each route.
        """
        target_angle = math.pi  # -x direction
        # smallest signed angular difference
        delta = ((target_angle - start_angle + math.pi) % (2 * math.pi)) - math.pi
        return start_angle + delta * factor
    
    positions = []
    
    # 1. ENTRY POINT at airspace boundary
    entry_x = AIRSPACE_RADIUS * np.cos(entry_angle)
    entry_y = AIRSPACE_RADIUS * np.sin(entry_angle)
    positions.append([entry_x, entry_y, 0.0])
    
    # 2. INTERMEDIATE WAYPOINTS - move along shallow arcs towards -x
    special_routes = {"STAR01", "STAR02", "STAR03", "STAR04",
                      "STAR05", "STAR15",
                      "STAR17", "STAR18", "STAR19", "STAR20"}
    if route_name in special_routes:
        # Loop-like extended curve without zig-zag
        curve_profile = [
            (0.10, 1.00),
            (0.30, 0.95),
            (0.50, 0.85),
            (0.65, 0.70),
            (0.80, 0.60),
        ]
    else:
        curve_profile = [
            (0.25, 0.85),
            (0.45, 0.70),
        ]
        if route_name == "STAR16":
            curve_profile = [
                (0.20, 0.90),
                (0.35, 0.80),
                (0.50, 0.72),
                (0.60, 0.65),
            ]
    for factor, radius_scale in curve_profile:
        turn_angle = steer_toward_center(entry_angle, factor)
        radius = AIRSPACE_RADIUS * radius_scale
        waypoint_x = radius * np.cos(turn_angle)
        waypoint_y = radius * np.sin(turn_angle)
        positions.append([waypoint_x, waypoint_y, 0.0])
    
    # Mid-point 3: almost aligned with centreline behind the runway
    if route_name in special_routes:
        sign = 1 if entry_y >= 0 else -1
        loop_alignment_points = [
            (-2100.0, 300.0 * sign),
            (-1850.0, 180.0 * sign),
            (-1600.0, 80.0 * sign),
        ]
        for x_val, y_val in loop_alignment_points:
            positions.append([x_val, y_val, 0.0])
    else:
        mid3_distance_from_runway = 1500.0
        lateral_offset = random.uniform(-60, 60) * GEOMETRY_SCALE
        positions.append([-mid3_distance_from_runway, lateral_offset, 0.0])
    
    # 3. FINAL APPROACH FIX (FAF) - same x/y for every route
    positions.append([FAF_POSITION[0], FAF_POSITION[1], 0.0])
    
    # 4. RUNWAY THRESHOLD - touchdown point
    positions.append([RUNWAY_THRESHOLD[0], RUNWAY_THRESHOLD[1], RUNWAY_THRESHOLD[2]])
    
    # Apply altitude based on glide angle with 500 m entry cap
    slope = math.tan(math.radians(approach_angle_deg))
    for idx, point in enumerate(positions):
        x, y, _ = point
        if idx == len(positions) - 1:
            z = 0.0
        else:
            horizontal_distance = math.hypot(RUNWAY_THRESHOLD[0] - x, RUNWAY_THRESHOLD[1] - y)
            z = min(MAX_ALTITUDE, slope * horizontal_distance)
        waypoints.append([x, y, z])
    
    return np.array(waypoints)


# ------ VISUALIZATION --------------------------------------------------------------

def plot_arrival_routes_3d(routes):
    """
    Plot all arrival routes in 3D showing the airspace and runway.
    
    Parameters
    ----------
    routes : list of dict
        List of route dictionaries with waypoints
    """
    
    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot each route
    colors = plt.cm.tab20(np.linspace(0, 1, len(routes)))
    
    for route, color in zip(routes, colors):
        waypoints = route['waypoints']
        
        # Plot the path
        ax.plot(waypoints[:, 0], waypoints[:, 1], waypoints[:, 2],
                marker='o', markersize=4, linewidth=2, alpha=0.7,
                label=route['name'], color=color)
        
        # Mark entry point
        ax.scatter(waypoints[0, 0], waypoints[0, 1], waypoints[0, 2],
                  color=color, s=100, marker='^', edgecolors='black', linewidth=1)
    
    # Plot Final Approach Fix (FAF)
    ax.scatter(FAF_POSITION[0], FAF_POSITION[1], FAF_POSITION[2],
              color='red', s=200, marker='*', edgecolors='black', linewidth=2,
              label='FAF (Final Approach Fix)')
    
    # Plot Runway (from 0,0,0 to 1508,0,0)
    runway_start = RUNWAY_THRESHOLD  # touchdown at origin
    runway_end = RUNWAY_END
    ax.plot([runway_start[0], runway_end[0]],
            [runway_start[1], runway_end[1]],
            [runway_start[2], runway_end[2]],
            color='black', linewidth=8, label='Runway')
    
    # Mark runway threshold (touchdown point at origin)
    ax.scatter(runway_start[0], runway_start[1], runway_start[2],
              color='green', s=300, marker='s', edgecolors='black', linewidth=2,
              label='Touchdown (0,0,0)')
    
    # Mark runway end
    ax.scatter(runway_end[0], runway_end[1], runway_end[2],
              color='blue', s=200, marker='s', edgecolors='black', linewidth=2,
              label='Runway End')
    
    # Draw airspace boundary (cylinder)
    theta = np.linspace(0, 2*np.pi, 100)
    z_cylinder = np.linspace(0, MAX_ALTITUDE, 50)
    theta_grid, z_grid = np.meshgrid(theta, z_cylinder)
    x_cylinder = AIRSPACE_RADIUS * np.cos(theta_grid)
    y_cylinder = AIRSPACE_RADIUS * np.sin(theta_grid)
    ax.plot_surface(x_cylinder, y_cylinder, z_grid, alpha=0.1, color='gray')
    
    # Draw exclusion zone (departure cone) - ±45° from +x axis
    # This is the area we avoid (departure climb-out)
    exclusion_angles = np.linspace(np.radians(-45), np.radians(45), 50)
    exclusion_radius = np.linspace(0, AIRSPACE_RADIUS, 30)
    for angle in [np.radians(-45), np.radians(45)]:  # boundaries
        x_excl = exclusion_radius * np.cos(angle)
        y_excl = exclusion_radius * np.sin(angle)
        z_excl = np.zeros_like(exclusion_radius)
        ax.plot(x_excl, y_excl, z_excl, 'r--', linewidth=2, alpha=0.5)
    
    # Labels and formatting
    ax.set_xlabel('X (meters) - Runway Direction', fontsize=12, fontweight='bold')
    ax.set_ylabel('Y (meters) - Lateral', fontsize=12, fontweight='bold')
    ax.set_zlabel('Z (meters) - Altitude', fontsize=12, fontweight='bold')
    ax.set_title('London City Airport - Arrival Routes (STAR)\n20 Routes with Exclusion Zone',
                fontsize=16, fontweight='bold')
    
    # Set axis limits (allow z-axis to scale naturally)
    max_range = AIRSPACE_RADIUS
    ax.set_xlim([-max_range, max_range])
    ax.set_ylim([-max_range, max_range])
    ax.set_zlim([0, MAX_ALTITUDE])
    
    # Add legend (smaller, outside plot)
    ax.legend(loc='upper left', bbox_to_anchor=(1.05, 1), fontsize=8, ncol=2)
    
    # Add grid
    ax.grid(True, alpha=0.3)
    
    # Better viewing angle
    ax.view_init(elev=20, azim=45)
    
    plt.tight_layout()
    
    return fig, ax


def plot_arrival_routes_top_view(routes):
    """
    Plot top-down view of arrival routes showing lateral paths.
    
    Parameters
    ----------
    routes : list of dict
        List of route dictionaries with waypoints
    """
    
    fig, ax = plt.subplots(figsize=(10, 10))
    
    # Plot each route
    colors = plt.cm.tab20(np.linspace(0, 1, len(routes)))
    
    for route, color in zip(routes, colors):
        waypoints = route['waypoints']
        
        # Plot the path
        ax.plot(waypoints[:, 0], waypoints[:, 1],
                marker='o', markersize=6, linewidth=2, alpha=0.7,
                label=route['name'], color=color)
        
        # Mark entry point
        ax.scatter(waypoints[0, 0], waypoints[0, 1],
                  color=color, s=150, marker='^', edgecolors='black', linewidth=1.5)
    
    # Plot Final Approach Fix (FAF)
    ax.scatter(FAF_POSITION[0], FAF_POSITION[1],
              color='red', s=400, marker='*', edgecolors='black', linewidth=2,
              label='FAF', zorder=10)
    
    # Plot Runway
    runway_start = RUNWAY_THRESHOLD  # touchdown at origin
    runway_end = RUNWAY_END
    ax.plot([runway_start[0], runway_end[0]],
            [runway_start[1], runway_end[1]],
            color='black', linewidth=12, label='Runway', zorder=5)
    
    # Mark touchdown point at origin
    ax.scatter(runway_start[0], runway_start[1],
              color='green', s=400, marker='s', edgecolors='black', linewidth=2,
              label='Touchdown (0,0,0)', zorder=10)
    
    # Mark runway end
    ax.scatter(runway_end[0], runway_end[1],
              color='blue', s=300, marker='s', edgecolors='black', linewidth=2,
              label='Runway End', zorder=10)
    
    # Draw airspace boundary
    circle = plt.Circle((0, 0), AIRSPACE_RADIUS, fill=False, 
                       edgecolor='gray', linewidth=3, linestyle='--', alpha=0.5)
    ax.add_patch(circle)
    
    # Draw exclusion zone (departure cone) - ±45° from +x axis
    exclusion_wedge = plt.matplotlib.patches.Wedge(
        (0, 0), AIRSPACE_RADIUS, -45, 45,
        facecolor='red', alpha=0.1, edgecolor='red', linewidth=2,
        linestyle='--', label='Departure Exclusion Zone'
    )
    ax.add_patch(exclusion_wedge)
    
    # Labels and formatting
    ax.set_xlabel('X (meters) - Runway Direction', fontsize=14, fontweight='bold')
    ax.set_ylabel('Y (meters) - Lateral', fontsize=14, fontweight='bold')
    ax.set_title('London City Airport - Arrival Routes (Top View)\nRed Zone: Departure Traffic',
                fontsize=16, fontweight='bold')
    
    ax.set_xlim([-AIRSPACE_RADIUS, AIRSPACE_RADIUS])
    ax.set_ylim([-AIRSPACE_RADIUS, AIRSPACE_RADIUS])
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    
    # Add legend
    ax.legend(loc='upper left', bbox_to_anchor=(1.02, 1), fontsize=9, ncol=2)
    
    plt.tight_layout()
    
    return fig, ax


def plot_route_details(routes, route_indices=[0, 5, 10, 15]):
    """
    Plot detailed information for specific routes.
    
    Parameters
    ----------
    routes : list of dict
        List of route dictionaries
    route_indices : list of int
        Indices of routes to plot in detail
    """
    
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    axes = axes.flatten()
    
    for idx, route_idx in enumerate(route_indices):
        if route_idx >= len(routes):
            continue
            
        route = routes[route_idx]
        waypoints = route['waypoints']
        ax = axes[idx]
        
        # Calculate distances along path
        distances = [0]
        for i in range(1, len(waypoints)):
            dist = np.linalg.norm(waypoints[i] - waypoints[i-1])
            distances.append(distances[-1] + dist)
        
        # Altitude profile
        altitudes = waypoints[:, 2]  # Z is already positive altitude
        
        ax.plot(distances, altitudes, marker='o', markersize=8, linewidth=2.5, color='blue')
        
        # Mark special points
        ax.scatter(distances[-2], altitudes[-2], color='red', s=200, marker='*',
                  edgecolors='black', linewidth=2, label='FAF', zorder=10)
        ax.scatter(distances[-1], altitudes[-1], color='green', s=200, marker='s',
                  edgecolors='black', linewidth=2, label='Threshold', zorder=10)
        
        # Draw expected glideslope from FAF using route-specific angle
        approach_angle = route.get('approach_angle', STANDARD_APPROACH_ANGLE_DEG)
        slope = math.tan(math.radians(approach_angle))
        glideslope_distance = np.linspace(distances[-2], distances[-1], 10)
        glideslope_altitude = altitudes[-2] - (glideslope_distance - distances[-2]) * slope
        ax.plot(glideslope_distance, glideslope_altitude, 'r--', linewidth=2,
               alpha=0.7, label=f'{approach_angle:.1f}° Glideslope')
        
        ax.set_xlabel('Distance Along Route (m)', fontsize=11, fontweight='bold')
        ax.set_ylabel('Altitude (m)', fontsize=11, fontweight='bold')
        ax.set_title(f'{route["name"]} - Entry: {route["entry_angle"]:.1f}°',
                    fontsize=12, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=9)
        
        # Add annotations
        for i, (d, alt) in enumerate(zip(distances, altitudes)):
            ax.annotate(f'WP{i+1}\n{alt:.0f}m',
                       xy=(d, alt), xytext=(10, 10),
                       textcoords='offset points', fontsize=8,
                       bbox=dict(boxstyle='round,pad=0.3', facecolor='yellow', alpha=0.5))
    
    plt.tight_layout()
    return fig, axes


def print_route_summary(routes):
    """
    Print summary information for all routes.
    
    Parameters
    ----------
    routes : list of dict
        List of route dictionaries
    """
    
    print("\n" + "="*80)
    print("ARRIVAL ROUTES SUMMARY")
    print("="*80)
    print(f"\nTotal Routes: {len(routes)}")
    print(f"Airspace Radius: {AIRSPACE_RADIUS/1000:.1f} km")
    print(f"Entry Altitude (max at boundary): ~{MAX_ALTITUDE:.0f} m")
    print(f"Final Approach Fix: {FAF_DISTANCE:.0f}m from touchdown, {FAF_ALTITUDE:.0f}m altitude")
    print(f"Approach Angle: {STANDARD_APPROACH_ANGLE_DEG:.1f}° for all routes")
    print(f"Runway Length: {RUNWAY_LENGTH:.0f} m (touchdown at origin 0,0,0)")
    print(f"\nDeparture Exclusion Zone: -45° to +45° (±45° from +X axis)")
    print(f"Landing & Takeoff Direction: Both in +X direction")
    
    print("\n" + "-"*80)
    print(f"{'Route':<10} {'Entry Angle':<15} {'Waypoints':<12} {'Total Distance':<18} {'Entry Alt':<12} {'Glide':<8}")
    print("-"*80)
    
    for route in routes:
        waypoints = route['waypoints']
        
        # Calculate total distance
        total_dist = 0
        for i in range(1, len(waypoints)):
            total_dist += np.linalg.norm(waypoints[i] - waypoints[i-1])
        
        entry_alt = waypoints[0, 2]
        
        print(f"{route['name']:<10} {route['entry_angle']:>8.1f}°     "
              f"{len(waypoints):>4} points    {total_dist:>10.0f} m        "
              f"{entry_alt:>6.0f} m   {route.get('approach_angle', STANDARD_APPROACH_ANGLE_DEG):>5.1f}°")
    
    print("-"*80)
    print("\n")


# ------ MAIN TESTING --------------------------------------------------------------

if __name__ == '__main__':
    """
    Generate and visualize arrival routes.
    """
    
    print("\n" + "="*80)
    print(" GENERATING ARRIVAL ROUTES")
    print("="*80)
    
    # Generate routes
    routes = generate_arrival_routes(num_routes=20)
    
    # Print summary
    print_route_summary(routes)
    
    # Create visualizations
    print("Creating 3D visualization...")
    fig_3d, ax_3d = plot_arrival_routes_3d(routes)
    
    print("Creating top-down view...")
    fig_top, ax_top = plot_arrival_routes_top_view(routes)
    
    print("Creating detailed route profiles...")
    fig_detail, axes_detail = plot_route_details(routes, route_indices=[0, 5, 10, 15])
    
    print("\nAll plots generated!")
    print("Close the plot windows to continue...")
    
    plt.show()
    
    print("\n" + "="*80)
    print("Route generation complete!")
    print("="*80)
