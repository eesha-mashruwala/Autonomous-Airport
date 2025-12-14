import json
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import glob
import math
import re
import matplotlib.patches as mpatches

# Import your existing module to get the planned paths
from arrival_routes import generate_arrival_routes

def get_closest_distance_to_path(point, waypoints):
    """
    Calculates the shortest distance from a 3D point to a path defined by line segments.
    """
    min_dist_sq = float('inf')
    
    p = np.array(point)
    
    # Iterate through each segment of the planned path
    for i in range(len(waypoints) - 1):
        a = waypoints[i]
        b = waypoints[i+1]
        
        # Vector from a to b
        ab = b - a
        # Vector from a to point
        ap = p - a
        
        # Project point onto line segment (clamped between 0 and 1)
        t = np.dot(ap, ab) / np.dot(ab, ab)
        t = max(0, min(1, t))
        
        # Find the closest point on the segment
        closest = a + t * ab
        dist_sq = np.sum((p - closest)**2)
        
        if dist_sq < min_dist_sq:
            min_dist_sq = dist_sq
            
    return np.sqrt(min_dist_sq)

def analyse_trajectories(folder_path="."):
    # 1. Generate Ground Truth Routes
    # Creates the dictionary of STAR01...STAR20
    all_routes = generate_arrival_routes(num_routes=20)
    route_map = {r['name']: r['waypoints'] for r in all_routes}
    
    mse_results = []
    
    # 2. Load all JSON trajectory files
    files = glob.glob(f"{folder_path}/trajectory_*.json")
    
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    for filepath in files:
        with open(filepath, 'r') as f:
            data = json.load(f)
            
        plane_id = data['plane_id']
        route_name = data['arrival_route']
        actual_path = np.array(data['coordinates'])
        
        # Get the planned waypoints for this specific route
        if route_name in route_map:
            planned_waypoints = route_map[route_name]
            
            # Truncate actual path at runway threshold (Z <= 10m) to exclude rollout/taxi
            # The planned route ends at runway threshold, so we shouldn't compare ground movement
            in_flight_mask = actual_path[:, 2] > 10  # Altitude > 10m
            actual_path_flight = actual_path[in_flight_mask]
            
            # If we filtered everything, use original (edge case)
            if len(actual_path_flight) < 10:
                actual_path_flight = actual_path
            
            # 3. Calculate Error (MSE)
            # Sample every 10th point to speed up calculation
            errors = []
            for point in actual_path_flight[::10]: 
                dist = get_closest_distance_to_path(point, planned_waypoints)
                errors.append(dist)
            
            mse = np.mean(np.array(errors)**2)
            rmse = np.sqrt(mse)
            mse_results.append((plane_id, route_name, mse, rmse))
            
            # 4. Plotting
            # Plot Actual
            ax.plot(actual_path[:,0], actual_path[:,1], actual_path[:,2], 
                    alpha=0.5, label=f"{plane_id} (Actual)")
            
            # Plot Planned (only once per unique route to avoid clutter)
            # (Logic to skip duplicate route plotting can be added here)
            ax.plot(planned_waypoints[:,0], planned_waypoints[:,1], planned_waypoints[:,2], 
                    'k--', linewidth=2, alpha=0.3)

    ax.set_title("Actual vs Planned Trajectories", fontweight='bold', fontsize=14)
    ax.set_xlabel("X (Runway)")
    ax.set_ylabel("Y (Lateral)")
    ax.set_zlabel("Z (Altitude)")
    plt.show()

    # 5. Statistical Summary
    print(f"\n{'='*70}")
    print(f"{'TRAJECTORY TRACKING ANALYSIS':<70}")
    print(f"{'='*70}")
    print(f"{'Plane ID':<25} {'Route':<12} {'MSE (m²)':<12} {'RMSE (m)':<12}")
    print("-" * 70)
    for res in mse_results:
        print(f"{res[0]:<25} {res[1]:<12} {res[2]:>10.2f}  {res[3]:>10.2f}")
    
    # Calculate statistics
    if mse_results:
        mse_values = [r[2] for r in mse_results]
        rmse_values = [r[3] for r in mse_results]
        mean_mse = np.mean(mse_values)
        median_mse = np.median(mse_values)
        std_mse = np.std(mse_values)
        min_mse = np.min(mse_values)
        max_mse = np.max(mse_values)
        mean_rmse = np.mean(rmse_values)
        median_rmse = np.median(rmse_values)
        
        print("-" * 70)
        print(f"{'Mean MSE:':<25} {mean_mse:>10.2f} m²  |  Mean RMSE: {mean_rmse:>8.2f} m")
        print(f"{'Median MSE:':<25} {median_mse:>10.2f} m²  |  Median RMSE: {median_rmse:>8.2f} m")
        print(f"{'Std Dev MSE:':<25} {std_mse:>10.2f} m²")
        print(f"{'Min MSE:':<25} {min_mse:>10.2f} m²")
        print(f"{'Max MSE:':<25} {max_mse:>10.2f} m²")
        print(f"{'='*70}\n")
        
        # Plot MSE Analysis
        plot_mse_analysis(mse_results)

def plot_mse_analysis(mse_results):
    """
    Creates comprehensive MSE visualizations to evaluate trajectory tracking accuracy.
    """
    if not mse_results:
        print("No MSE results to plot.")
        return
    
    # Extract data
    plane_ids = [r[0] for r in mse_results]
    routes = [r[1] for r in mse_results]
    mse_values = np.array([r[2] for r in mse_results])
    rmse_values = np.array([r[3] for r in mse_results])
    
    # Sort by MSE for better visualization
    sorted_indices = np.argsort(mse_values)
    plane_ids_sorted = [plane_ids[i] for i in sorted_indices]
    mse_sorted = mse_values[sorted_indices]
    
    # Create figure with multiple subplots
    fig = plt.figure(figsize=(14, 8))
    gs = fig.add_gridspec(3, 2, hspace=0.45, wspace=0.35)
    
    ax1 = fig.add_subplot(gs[0:2, 0])  # Bar chart (left, tall)
    ax2 = fig.add_subplot(gs[0, 1])     # Histogram (top right)
    ax3 = fig.add_subplot(gs[1, 1])     # Box plot (middle right)
    ax4 = fig.add_subplot(gs[2, :])     # Route comparison (bottom, full width)
    
    # Color mapping based on MSE magnitude
    colors = ['#27ae60' if m < np.percentile(mse_values, 33) 
              else '#f39c12' if m < np.percentile(mse_values, 67) 
              else '#e74c3c' for m in mse_sorted]
    
    # Get sorted RMSE values to match
    rmse_sorted = rmse_values[sorted_indices]
    
    # Plot 1: Bar Chart - Individual Aircraft RMSE (sorted)
    y_pos = np.arange(len(plane_ids_sorted))
    bars = ax1.barh(y_pos, rmse_sorted, color=colors, edgecolor='black', linewidth=0.5)
    ax1.set_yticks(y_pos)
    ax1.set_yticklabels(plane_ids_sorted, fontsize=7)
    ax1.set_xlabel('Root Mean Squared Error (m)', fontsize=9, fontweight='bold')
    ax1.set_ylabel('Aircraft', fontsize=9, fontweight='bold')
    ax1.set_title('Trajectory Tracking Error by Aircraft\n(Sorted from highest to lowest RMSE)', 
                  fontsize=9, fontweight='bold')
    ax1.grid(axis='x', alpha=0.3, linestyle='--')
    ax1.axvline(np.mean(rmse_values), color='red', linestyle='--', linewidth=1.5, 
                label=f'Mean: {np.mean(rmse_values):.1f} m')
    ax1.legend(fontsize=8)
    
    # Add value labels on bars
    # Plot 2: Histogram - RMSE Distribution
    ax2.hist(rmse_values, bins=min(15, len(rmse_values)//2 + 1), 
             color='#3498db', edgecolor='black', alpha=0.7)
    ax2.axvline(np.mean(rmse_values), color='red', linestyle='--', linewidth=1.5, 
                label=f'Mean: {np.mean(rmse_values):.1f}m')
    ax2.axvline(np.median(rmse_values), color='orange', linestyle='--', linewidth=1.5, 
                label=f'Median: {np.median(rmse_values):.1f}m')
    ax2.set_xlabel('RMSE (m)', fontsize=8, fontweight='bold')
    ax2.set_ylabel('Frequency', fontsize=8, fontweight='bold')
    ax2.set_title('RMSE Distribution', fontsize=8, fontweight='bold')
    ax2.legend(fontsize=7)
    ax2.grid(axis='y', alpha=0.3)
    
    # Plot 3: Box Plot - Statistical Summary
    bp = ax3.boxplot(rmse_values, vert=True, patch_artist=True, 
                     widths=0.5, showmeans=True, meanline=True,
                     boxprops=dict(facecolor='#3498db', alpha=0.7),
                     medianprops=dict(color='red', linewidth=1.5),
                     meanprops=dict(color='green', linewidth=1.5, linestyle='--'),
                     whiskerprops=dict(linewidth=1),
                     capprops=dict(linewidth=1))
    ax3.set_ylabel('RMSE (m)', fontsize=8, fontweight='bold')
    ax3.set_title('RMSE Statistics (Box Plot)', fontsize=9, fontweight='bold')
    ax3.grid(axis='y', alpha=0.3)
    ax3.set_xticklabels(['All Trajectories'], fontsize=7)
    
    # Add statistics text
    stats_text = f"Q1: {np.percentile(rmse_values, 25):.1f}m\nMedian: {np.median(rmse_values):.1f}m\nQ3: {np.percentile(rmse_values, 75):.1f}m"
    ax3.text(1.15, np.median(rmse_values), stats_text, fontsize=7, 
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    # Plot 4: RMSE by Route (if multiple aircraft per route)
    route_rmse = {}
    for plane, route, mse, rmse in mse_results:
        if route not in route_rmse:
            route_rmse[route] = []
        route_rmse[route].append(rmse)
    
    # Calculate mean RMSE per route
    routes_unique = sorted(route_rmse.keys())
    route_means = [np.mean(route_rmse[r]) for r in routes_unique]
    route_counts = [len(route_rmse[r]) for r in routes_unique]
    
    x_pos = np.arange(len(routes_unique))
    bars = ax4.bar(x_pos, route_means, color='#9b59b6', edgecolor='black', 
                   linewidth=0.8, alpha=0.8)
    ax4.set_xticks(x_pos)
    ax4.set_xticklabels(routes_unique, rotation=45, ha='right', fontsize=7)
    ax4.set_xlabel('Arrival Route', fontsize=9, fontweight='bold')
    ax4.set_ylabel('Mean RMSE (m)', fontsize=9, fontweight='bold')
    ax4.set_title('Average Tracking Error by Route', fontsize=10, fontweight='bold')
    ax4.grid(axis='y', alpha=0.3)
    ax4.axhline(np.mean(rmse_values), color='red', linestyle='--', linewidth=1.5, 
                label=f'Overall Mean: {np.mean(rmse_values):.1f} m')
    ax4.legend(fontsize=7)
    
    # Add count labels on bars
    for i, (bar, val, count) in enumerate(zip(bars, route_means, route_counts)):
        ax4.text(i, val, f'{val:.1f}m\n(n={count})', ha='center', va='bottom', 
                fontsize=7, fontweight='bold')
    
    plt.suptitle('Trajectory Tracking Performance Analysis', 
                 fontsize=12, fontweight='bold', y=0.98)
    plt.show()

def parse_simulation_log(log_path):
    events = []
    
    # Regex patterns - only match timestamped lines since every event has one
    # Arrival: "[t= 60.8s] ATC: Cleared to land"
    rx_arrival = re.compile(r"\[t=\s*([\d\.]+)s\]\s*ATC:\s*Cleared to land", re.IGNORECASE)
    
    # Departure: "[t=125.6s] Cleared for takeoff..."
    rx_departure = re.compile(r"\[t=\s*([\d\.]+)s\]\s*Cleared for takeoff", re.IGNORECASE)
    
    # Runway free: "[t= 59.7s] Runway now FREE (timer elapsed)"
    rx_free = re.compile(r"\[t=\s*([\d\.]+)s\]\s*Runway now FREE", re.IGNORECASE)
    
    # Total time
    rx_total_time = re.compile(r"Total sim time:\s*([\d\.]+)s", re.IGNORECASE)

    current_event = None
    total_sim_time = 0.0

    # Detect encoding (UTF-16 LE or UTF-8)
    with open(log_path, 'rb') as fb:
        first_bytes = fb.read(2)
        encoding = 'utf-16-le' if first_bytes == b'\xff\xfe' else 'utf-8'
    
    with open(log_path, 'r', encoding=encoding, errors='ignore') as f:
        for line in f:
            line = line.strip()
            
            # Check for total time
            m_total = rx_total_time.search(line)
            if m_total:
                total_sim_time = float(m_total.group(1))

            # 1. Check for Arrival Start
            m_arr = rx_arrival.search(line)
            if m_arr:
                if current_event:
                    current_event['end'] = float(m_arr.group(1))
                    events.append(current_event)
                
                current_event = {
                    'type': 'Arrival',
                    'start': float(m_arr.group(1)),
                    'end': None
                }
                continue

            # 2. Check for Departure Start
            m_dep = rx_departure.search(line)
            if m_dep:
                if current_event:
                    current_event['end'] = float(m_dep.group(1))
                    events.append(current_event)
                
                current_event = {
                    'type': 'Departure',
                    'start': float(m_dep.group(1)),
                    'end': None
                }
                continue

            # 3. Check for Runway Free (End of event)
            m_free = rx_free.search(line)
            if m_free and current_event:
                current_event['end'] = float(m_free.group(1))
                events.append(current_event)
                current_event = None

    # If the log ended while an event is open, close it using total_sim_time if available
    if current_event:
        if total_sim_time > 0:
            current_event['end'] = total_sim_time
        events.append(current_event)

    # Diagnostics: summarize what was parsed
    if events:
        first_ts = min(e['start'] for e in events if 'start' in e)
        last_ts = max((e['end'] if e.get('end') is not None else e['start']) for e in events)
        arrival_count = sum(1 for e in events if e['type'] == 'Arrival')
        departure_count = sum(1 for e in events if e['type'] == 'Departure')
        print(f"Parsed events: arrivals={arrival_count}, departures={departure_count}, time window={first_ts:.1f}s->{last_ts:.1f}s")
    else:
        print("Parsed events: none (check regex/log format)")

    return events, total_sim_time

def plot_runway_stats(events, total_time):
    if not events:
        print("No events found in log.")
        return

    # --- Calculation ---
    total_busy_time = sum(e['end'] - e['start'] for e in events if e['end'])
    arrival_count = len([e for e in events if e['type'] == 'Arrival'])
    departure_count = len([e for e in events if e['type'] == 'Departure'])
    
    utilisation_pct = (total_busy_time / total_time) * 100
    
    print(f"Total Simulation Time: {total_time:.2f}s")
    print(f"Runway Busy Time:      {total_busy_time:.2f}s")
    print(f"Utilisation:           {utilisation_pct:.1f}%")
    print(f"Arrivals: {arrival_count} | Departures: {departure_count}")

    # --- Plotting ---
    fig = plt.figure(figsize=(12, 8))
    gs = fig.add_gridspec(3, 2, hspace=0.4, wspace=0.35)
    
    ax1 = fig.add_subplot(gs[0:2, :])  # Gantt chart (top, full width)
    ax2 = fig.add_subplot(gs[2, 0])     # Active operations timeline
    ax3 = fig.add_subplot(gs[2, 1])     # Traffic mix

    # Plot 1: Runway Timeline (Gantt with separate rows per operation)
    colors = {'Arrival': '#3498db', 'Departure': '#e74c3c'}
    
    ax1.set_xlim(0, total_time)
    ax1.set_ylim(0, len(events))
    ax1.set_xlabel("Simulation Time (s)", fontsize=8)
    ax1.set_ylabel("Operation #", fontsize=8)
    ax1.set_title(f"Runway Operations Timeline - Utilisation: {utilisation_pct:.1f}%", fontsize=10, fontweight='bold')
    ax1.grid(axis='x', alpha=0.3, linestyle='--')
    
    # Plot each event on its own row
    for idx, e in enumerate(events):
        if e['end']:
            duration = e['end'] - e['start']
            ax1.barh(idx, duration, left=e['start'], height=0.8,
                    color=colors[e['type']], edgecolor='black', linewidth=0.5)
            
            # Label with type and duration
            center = e['start'] + duration/2
            label = f"{e['type'][0]} ({duration:.1f}s)"
            ax1.text(center, idx, label, ha='center', va='center', 
                    color='white', fontsize=7, fontweight='bold')
    
    # Legend
    patches = [mpatches.Patch(color=c, label=l) for l, c in colors.items()]
    ax1.legend(handles=patches, loc='upper right', fontsize=8)

    # Plot 2: Active Operations Over Time (Line Chart)
    # Create time series of active operations
    time_points = []
    active_counts = []
    
    # Collect all event boundaries
    boundaries = []
    for e in events:
        if e['end']:
            boundaries.append((e['start'], 1))   # Operation starts
            boundaries.append((e['end'], -1))    # Operation ends
    
    boundaries.sort()
    
    current_active = 0
    time_points.append(0)
    active_counts.append(0)
    
    for time, delta in boundaries:
        current_active += delta
        time_points.append(time)
        active_counts.append(current_active)
    
    # Add final point
    if time_points[-1] < total_time:
        time_points.append(total_time)
        active_counts.append(0)
    
    ax2.plot(time_points, active_counts, color='#2ecc71', linewidth=1.5, marker='o', markersize=2)
    ax2.fill_between(time_points, active_counts, alpha=0.3, color='#2ecc71')
    ax2.set_xlim(0, total_time)
    ax2.set_ylim(0, max(active_counts) + 0.5)
    ax2.set_xlabel("Time (s)", fontsize=8)
    ax2.set_ylabel("Active Operations", fontsize=8)
    ax2.set_title("Runway Occupancy Over Time", fontsize=8, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.set_yticks(range(0, max(active_counts) + 1))
    ax2.tick_params(labelsize=7)

    # Plot 3: Traffic Mix
    labels = ['Arrivals', 'Departures']
    counts = [arrival_count, departure_count]
    bars = ax3.bar(labels, counts, color=[colors['Arrival'], colors['Departure']], 
                   edgecolor='black', linewidth=1)
    ax3.set_ylabel("Number of Operations", fontsize=8)
    ax3.set_title("Traffic Mix", fontsize=8, fontweight='bold')
    ax3.bar_label(bars, fontsize=8, fontweight='bold')
    ax3.grid(axis='y', alpha=0.3)
    ax3.tick_params(labelsize=7)

    plt.show()

if __name__ == "__main__":
    analyse_trajectories()
    # Point this to your actual log file
    log_file = "headless.log"
    
    try:
        events, sim_time = parse_simulation_log(log_file)
        # Fallback if sim_time not found in log (e.g., if log truncated)
        if sim_time == 0 and events: 
            sim_time = events[-1]['end'] + 10 
            
        plot_runway_stats(events, sim_time)
    except FileNotFoundError:
        print(f"Error: Could not find {log_file}. Make sure it is in the same folder.")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()