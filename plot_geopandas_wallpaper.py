"""
Render a wallpaper-style flight trajectory around London City Airport using GeoPandas.

- Reads one of the generated trajectory_*.json files (local ENU coordinates, meters).
- Converts the path to real-world lat/lon anchored at LCY runway 09 threshold.
- Projects to Web Mercator for a tiled basemap and draws a 5.5 km radius boundary.
- Colors the track by altitude up to 500 m and exports a clean, axis-free image.

Requirements: geopandas, shapely, pyproj, contextily, matplotlib, numpy.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Tuple

import contextily as ctx
import geopandas as gpd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection
from matplotlib.colors import Normalize
from pyproj import Transformer
from shapely.geometry import LineString, Point

# Coordinate anchors (sourced from OpenStreetMap runway geometry)
LCY_RUNWAY09_THRESHOLD = (51.5055242, 0.0457334)  # (lat, lon)
RUNWAY_HEADING_DEG = 94.0  # runway 09/27 magnetic heading from OSM tag

# Airspace limits used by the simulator
AIRSPACE_RADIUS_M = 5500.0
MAX_ALTITUDE_M = 500.0

# UTM zone covering London City Airport (E 0.045 -> EPSG:32631)
WGS84 = "EPSG:4326"
LCY_UTM = "EPSG:32631"
WEB_MERCATOR = "EPSG:3857"


def load_trajectory(path: Path) -> Tuple[dict, np.ndarray]:
    """Load the JSON trajectory and return the metadata and (N, 3) coordinates array."""
    with path.open("r") as f:
        data = json.load(f)
    coords = np.asarray(data["coordinates"], dtype=float)
    return data, coords


def rotate_to_runway_frame(xy: np.ndarray) -> np.ndarray:
    """
    Rotate ENU coordinates so the +x axis matches the real runway heading (094°).

    The simulator assumes the runway lies on +x (due east). LCY is ~4° south of east,
    so we rotate clockwise by that offset to align with the actual pavement.
    """
    heading_delta_rad = math.radians(RUNWAY_HEADING_DEG - 90.0)
    theta = -heading_delta_rad  # clockwise rotation
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    rot_matrix = np.array([[cos_t, -sin_t], [sin_t, cos_t]])
    return xy @ rot_matrix.T


def local_to_wgs84(xyz: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Convert local runway-frame meters to lat/lon using an ENU translation anchored
    at the runway 09 threshold.
    """
    transformer_to_utm = Transformer.from_crs(WGS84, LCY_UTM, always_xy=True)
    transformer_to_wgs84 = Transformer.from_crs(LCY_UTM, WGS84, always_xy=True)

    origin_lon, origin_lat = LCY_RUNWAY09_THRESHOLD[1], LCY_RUNWAY09_THRESHOLD[0]
    origin_e, origin_n = transformer_to_utm.transform(origin_lon, origin_lat)

    xy_rot = rotate_to_runway_frame(xyz[:, :2])
    east = origin_e + xy_rot[:, 0]
    north = origin_n + xy_rot[:, 1]
    lon, lat = transformer_to_wgs84.transform(east, north)

    return np.asarray(lat), np.asarray(lon), xyz[:, 2]


def build_geodataframe(lat: np.ndarray, lon: np.ndarray, alt: np.ndarray) -> gpd.GeoDataFrame:
    """Create a GeoDataFrame of point geometries with altitude attribute."""
    points = [Point(xy) for xy in zip(lon, lat)]
    return gpd.GeoDataFrame({"altitude_m": alt}, geometry=points, crs=WGS84)


def draw_wallpaper(gdf_wgs84: gpd.GeoDataFrame, meta: dict, output: Path) -> None:
    """Render a wallpaper-like map with a dark basemap and altitude-colored track."""
    # Project to Web Mercator for contextily tiles
    gdf_mercator = gdf_wgs84.to_crs(WEB_MERCATOR)
    origin_pt = gpd.GeoSeries([Point(LCY_RUNWAY09_THRESHOLD[::-1])], crs=WGS84).to_crs(WEB_MERCATOR)[0]
    boundary_circle = origin_pt.buffer(AIRSPACE_RADIUS_M)

    fig, ax = plt.subplots(figsize=(12, 12), facecolor="#06070d")

    # Basemap first to let overlays sit on top cleanly
    ctx.add_basemap(ax, crs=WEB_MERCATOR, source=ctx.providers.CartoDB.DarkMatter, zoom=14)

    # Soft boundary disk to emphasize the 5.5 km airspace
    gpd.GeoSeries([boundary_circle], crs=WEB_MERCATOR).plot(
        ax=ax,
        facecolor="#0b1224",
        edgecolor="#1f2a44",
        alpha=0.18,
        linewidth=1.2,
    )

    # Build colored line segments from the 2D projected points
    xy = np.vstack([gdf_mercator.geometry.x, gdf_mercator.geometry.y]).T
    alt = gdf_mercator["altitude_m"].to_numpy()
    segments = np.stack([xy[:-1], xy[1:]], axis=1)
    norm = Normalize(vmin=0, vmax=MAX_ALTITUDE_M)
    lc = LineCollection(
        segments,
        cmap="plasma",
        norm=norm,
        linewidths=4.0,
        alpha=0.95,
    )
    lc.set_array((alt[:-1] + alt[1:]) / 2.0)
    ax.add_collection(lc)

    # Highlight start/end
    ax.scatter(xy[0, 0], xy[0, 1], s=90, color="#48fbd9", edgecolor="#0d0d0d", zorder=5)
    ax.scatter(xy[-1, 0], xy[-1, 1], s=90, color="#f8c630", edgecolor="#0d0d0d", zorder=5)

    # Minimal text overlay (keeps wallpaper feel while conveying context)
    title = f"{meta.get('plane_id', 'Flight')} • {meta.get('arrival_route', '')} • ≤{int(MAX_ALTITUDE_M)} m"
    ax.text(
        0.02,
        0.98,
        title.strip(),
        transform=ax.transAxes,
        ha="left",
        va="top",
        fontsize=12,
        color="#e5e7eb",
        fontweight="bold",
        bbox=dict(facecolor="#0b1224dd", edgecolor="none", boxstyle="round,pad=0.55"),
    )

    cbar = plt.colorbar(lc, ax=ax, fraction=0.025, pad=0.01)
    cbar.set_label("Altitude (m)", color="#cbd5e1", fontsize=10)
    cbar.ax.yaxis.set_tick_params(color="#cbd5e1")
    plt.setp(plt.getp(cbar.ax.axes, "yticklabels"), color="#cbd5e1")

    # Frame and axes cleanup for wallpaper-style output
    ax.set_xlim(origin_pt.x - AIRSPACE_RADIUS_M, origin_pt.x + AIRSPACE_RADIUS_M)
    ax.set_ylim(origin_pt.y - AIRSPACE_RADIUS_M, origin_pt.y + AIRSPACE_RADIUS_M)
    ax.set_aspect("equal")
    ax.axis("off")

    output.parent.mkdir(parents=True, exist_ok=True)
    plt.subplots_adjust(0, 0, 1, 1)
    plt.savefig(output, dpi=300, bbox_inches="tight", pad_inches=0)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot LCY trajectory JSON on a dark map (wallpaper-style).")
    parser.add_argument(
        "trajectory_json",
        type=Path,
        nargs='?',  # Make optional
        default=None,
        help="Path to trajectory_*.json file produced by the simulator.",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=None,
        help="Output image path (PNG recommended).",
    )
    args = parser.parse_args()

    # If no trajectory file specified, find and use the first available one
    if args.trajectory_json is None:
        import glob
        trajectory_files = glob.glob("trajectory_*.json")
        if not trajectory_files:
            parser.error("No trajectory file specified and no trajectory_*.json files found in current directory.")
        args.trajectory_json = Path(trajectory_files[0])
        print(f"Using trajectory file: {args.trajectory_json}")
    
    # Auto-generate output filename if not specified
    if args.output is None:
        plane_name = args.trajectory_json.stem.replace("trajectory_", "")
        args.output = Path(f"flight_wallpaper_{plane_name}.png")

    meta, coords = load_trajectory(args.trajectory_json)
    lat, lon, alt = local_to_wgs84(coords)
    gdf = build_geodataframe(lat, lon, alt)
    draw_wallpaper(gdf, meta, args.output)
    print(f"✓ Wallpaper saved to: {args.output}")


if __name__ == "__main__":
    main()
