"""
Static matplotlib preview showing two intersecting runways with one aircraft landing
and one departing. Uses the existing ground layout but does not run the full simulator.
"""

import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from ground_operations import RUNWAY_LENGTH, RUNWAY_THRESHOLD, RUNWAY_END, AIRSPACE_RADIUS
from aircraft import EmbraerE170, Dash8_400
from arrival_routes import generate_arrival_routes


def _second_runway_geometry(angle_deg: float = 60.0, length_scale: float = 0.9):
    """Return start/end points for a second runway intersecting the main one."""
    intersection = np.array(RUNWAY_THRESHOLD) + np.array(
        [RUNWAY_LENGTH * 0.625, RUNWAY_THRESHOLD[1], 0.0]
    )
    half_length = (RUNWAY_LENGTH * length_scale) / 2.0
    theta = math.radians(angle_deg)
    direction = np.array([math.cos(theta), math.sin(theta), 0.0])
    start = intersection - half_length * direction
    end = intersection + half_length * direction
    return start, end, intersection


def _gate_ring_positions(
    num_gates: int = 19,
    radius: float | None = None,
    avoid_angles: tuple[float, float, float, float] = (0, 180, 60, 240),
):
    """
    Place gates in a ring centred on the runway intersection, avoiding headings that align
    with runway directions to keep them off the pavement.
    """
    _, _, center = _second_runway_geometry()
    if radius is None:
        radius = RUNWAY_LENGTH * 0.45

    positions = []
    ang = 0
    while len(positions) < num_gates and ang < 360:
        blocked = any(abs((ang - a + 180) % 360 - 180) < 22 for a in avoid_angles)
        if not blocked:
            theta = math.radians(ang)
            x = center[0] + radius * math.cos(theta)
            y = center[1] + radius * math.sin(theta)
            positions.append([x, y, 0.0])
        ang += 12
    return np.array(positions[:num_gates])


def _select_star_route():
    routes = generate_arrival_routes(num_routes=20)
    # pick the longest route for a pronounced STAR path
    best = None
    best_len = -1
    for r in routes:
        wps = r["waypoints"]
        length = float(np.linalg.norm(np.diff(wps, axis=0), axis=1).sum())
        if length > best_len:
            best_len = length
            best = r
    return best


def plot_intersecting_runways(save_path: str | Path = "intersecting_runways_preview.png", show: bool = False):
    gates = _gate_ring_positions()
    star_route = _select_star_route()
    occupied_gate_indices = [1, 4, 9, 15]  # non-adjacent occupied gates
    fig, ax = plt.subplots(figsize=(12, 8))

    # Main runway
    ax.plot(
        [RUNWAY_THRESHOLD[0], RUNWAY_END[0]],
        [RUNWAY_THRESHOLD[1], RUNWAY_END[1]],
        color="grey",
        linewidth=10,
        alpha=0.55,
        label="Runway A",
    )

    # Second runway at 60° intersecting near 5/8 length
    rw2_start, rw2_end, rw2_int = _second_runway_geometry()
    ax.plot(
        [rw2_start[0], rw2_end[0]],
        [rw2_start[1], rw2_end[1]],
        color="slateblue",
        linewidth=9,
        alpha=0.6,
        label="Runway B",
    )
    # intersection marker intentionally omitted for clarity

    # Gates
    ax.scatter(gates[:, 0], gates[:, 1], c="lightblue", s=70, marker="o", edgecolor="navy", alpha=0.9)
    occupied = gates[occupied_gate_indices]
    parked_colors = ["orange", "purple", "darkblue", "hotpink"]
    for j, pos in enumerate(occupied):
        ax.scatter(pos[0], pos[1], c=parked_colors[j % len(parked_colors)], s=90, marker="o",
                   edgecolor="navy", alpha=0.95)
    for i, pos in enumerate(gates):
        ax.text(pos[0], pos[1] + 20, f"G{i+1}", fontsize=8, ha="center", color="navy", alpha=0.7)

    # Landing aircraft on Runway A
    arrival_plane = EmbraerE170("E170-LND")
    wps2d = star_route["waypoints"][:, :2]
    ax.plot(
        wps2d[:, 0],
        wps2d[:, 1],
        linestyle="--",
        color="darkgreen",
        linewidth=2,
        alpha=0.7,
        label="Arrival path",
    )
    # Show arrival sphere 35% along the primary runway
    arrival_pos = RUNWAY_THRESHOLD[:2] + 0.35 * (RUNWAY_END[:2] - RUNWAY_THRESHOLD[:2])
    ax.scatter(arrival_pos[0], arrival_pos[1], c="darkgreen", marker="o", s=120, edgecolor="black", zorder=6)
    ax.text(
        arrival_pos[0],
        arrival_pos[1] + 180,
        f"Landing {arrival_plane.plane_type}\n({arrival_plane.id})",
        fontsize=9,
        ha="center",
        color="darkgreen",
    )

    # Departing aircraft on Runway B
    departure_plane = Dash8_400("DH8D-DEP")
    dep_start = rw2_start
    dep_end = rw2_start - (rw2_end - rw2_start) * 0.6
    dep_vec = dep_end - dep_start
    dep_dot = dep_start + 0.55 * dep_vec  # push dot further along roll
    ax.plot(
        [dep_start[0], dep_end[0]],
        [dep_start[1], dep_end[1]],
        linestyle="--",
        color="firebrick",
        linewidth=2,
        alpha=0.7,
        label="Departure roll",
    )
    ax.scatter(dep_dot[0], dep_dot[1], c="firebrick", marker="o", s=140, edgecolor="black", zorder=6)
    ax.text(
        dep_dot[0] + 80,
        dep_dot[1] + 120,
        f"Take-off {departure_plane.plane_type}\n({departure_plane.id})",
        fontsize=9,
        color="firebrick",
    )

    # Safety envelopes
    ax.add_patch(
        plt.Circle((0, 0), AIRSPACE_RADIUS * 0.8, color="silver", fill=False, linestyle=":", linewidth=1.5, alpha=0.6)
    )
    ax.text(0, AIRSPACE_RADIUS * 0.8 + 80, "Airspace boundary (not to scale vertically)", ha="center", fontsize=8)

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim([-AIRSPACE_RADIUS * 0.6, RUNWAY_LENGTH + AIRSPACE_RADIUS * 0.2])
    ax.set_ylim([-AIRSPACE_RADIUS * 0.5, AIRSPACE_RADIUS * 0.5])
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.grid(True, alpha=0.2)
    ax.legend(loc="upper left", fontsize=9)

    fig.suptitle("Intersecting Runways Preview (arrival on A, departure on B)", fontsize=12, fontweight="bold")
    fig.tight_layout()
    fig.savefig(save_path, dpi=200)
    if show:
        plt.show()
    plt.close(fig)


def plot_intersecting_runways_3d(save_path: str | Path = "intersecting_runways_preview_3d.png", show: bool = False):
    gates = _gate_ring_positions()
    star_route = _select_star_route()
    occupied_gate_indices = [1, 4, 9, 15]
    rw2_start, rw2_end, rw2_int = _second_runway_geometry()

    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection="3d")

    # Runways (slightly elevated for visibility)
    ax.plot(
        [RUNWAY_THRESHOLD[0], RUNWAY_END[0]],
        [RUNWAY_THRESHOLD[1], RUNWAY_END[1]],
        [1.0, 1.0],
        color="grey",
        linewidth=6,
        alpha=0.7,
        label="Runway A",
    )
    ax.plot(
        [rw2_start[0], rw2_end[0]],
        [rw2_start[1], rw2_end[1]],
        [1.0, 1.0],
        color="slateblue",
        linewidth=6,
        alpha=0.7,
        label="Runway B",
    )

    # Gates as spheres
    ax.scatter(gates[:, 0], gates[:, 1], gates[:, 2], c="lightblue", s=90, marker="o", depthshade=True)
    occupied = gates[occupied_gate_indices]
    parked_colors = ["orange", "purple", "darkblue", "hotpink"]
    for j, pos in enumerate(occupied):
        ax.scatter(pos[0], pos[1], pos[2], c=parked_colors[j % len(parked_colors)], s=110, marker="o",
                   depthshade=True)
    for i, pos in enumerate(gates):
        ax.text(pos[0], pos[1], pos[2] + 10, f"G{i+1}", fontsize=7, ha="center", color="navy")

    # Aircraft markers (spheres)
    arrival_plane = EmbraerE170("E170-LND")
    wps3d = star_route["waypoints"]
    arrival_pos = np.array([RUNWAY_THRESHOLD[0] + 0.35 * (RUNWAY_END[0] - RUNWAY_THRESHOLD[0]),
                            RUNWAY_THRESHOLD[1] + 0.35 * (RUNWAY_END[1] - RUNWAY_THRESHOLD[1]),
                            1.5])
    ax.scatter(arrival_pos[0], arrival_pos[1], arrival_pos[2], c="darkgreen", s=140, marker="o", depthshade=True)
    ax.plot(
        wps3d[:, 0],
        wps3d[:, 1],
        wps3d[:, 2],
        linestyle="--",
        color="darkgreen",
        linewidth=2,
        alpha=0.7,
    )
    ax.text(
        arrival_pos[0],
        arrival_pos[1],
        arrival_pos[2] + 15,
        f"Landing {arrival_plane.plane_type}",
        fontsize=8,
        color="darkgreen",
        ha="center",
    )

    departure_plane = Dash8_400("DH8D-DEP")
    dep_start = rw2_start.copy()
    dep_start[2] = 0.5
    dep_dir = (rw2_end - rw2_start)
    dep_end = dep_start - np.append(dep_dir[:2] * 0.6, -120.0)
    dep_dot = dep_start + 0.55 * (dep_end - dep_start)
    ax.scatter(dep_dot[0], dep_dot[1], dep_dot[2], c="firebrick", s=140, marker="o", depthshade=True)
    ax.plot(
        [dep_start[0], dep_end[0]],
        [dep_start[1], dep_end[1]],
        [dep_start[2], dep_end[2]],
        linestyle="--",
        color="firebrick",
        linewidth=2,
        alpha=0.7,
    )
    ax.text(
        dep_start[0],
        dep_start[1],
        dep_start[2] + 15,
        f"Take-off {departure_plane.plane_type}",
        fontsize=8,
        color="firebrick",
        ha="center",
    )

    ax.set_xlim([-AIRSPACE_RADIUS * 0.6, RUNWAY_LENGTH + AIRSPACE_RADIUS * 0.2])
    ax.set_ylim([-AIRSPACE_RADIUS * 0.5, AIRSPACE_RADIUS * 0.5])
    ax.set_zlim([0, 150])
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Altitude (m)")
    ax.view_init(elev=28, azim=-60)
    ax.legend(loc="upper left", fontsize=9)
    fig.tight_layout()
    fig.savefig(save_path, dpi=200)
    if show:
        plt.show()
    plt.close(fig)


if __name__ == "__main__":
    plot_intersecting_runways(show=True)
    plot_intersecting_runways_3d(show=True)
