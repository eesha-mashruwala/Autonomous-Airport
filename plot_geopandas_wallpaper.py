"""
Render a 3D flight trajectory around London City Airport with map basemap.

- Reads one of the generated trajectory_*.json files (local ENU coordinates, meters).
- Creates a 3D plot with x,y as ground plane and z as altitude.
- Projects a London City Airport satellite map onto the ground plane (z=0).
- Colors the track by altitude and shows the full 3D trajectory.

Requirements: geopandas, shapely, pyproj, contextily, matplotlib, numpy, PIL.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Tuple
import io

import contextily as ctx
import geopandas as gpd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import Normalize
from mpl_toolkits.mplot3d import Axes3D
from pyproj import Transformer
from shapely.geometry import Point
from PIL import Image

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


def get_basemap_image(center_lon: float, center_lat: float, radius_m: float, zoom: int = 14) -> Tuple[np.ndarray, tuple]:
    """
    Download a satellite/map image for the area and return the image array and extent.
    """
    # Create a temporary figure to get the basemap
    transformer = Transformer.from_crs(WGS84, WEB_MERCATOR, always_xy=True)
    origin_x, origin_y = transformer.transform(center_lon, center_lat)
    
    # Create bounds for the map
    west = origin_x - radius_m
    east = origin_x + radius_m
    south = origin_y - radius_m
    north = origin_y + radius_m
    
    # Create temporary 2D plot to fetch basemap
    fig_temp, ax_temp = plt.subplots(figsize=(10, 10))
    ax_temp.set_xlim(west, east)
    ax_temp.set_ylim(south, north)
    
    # Fetch basemap
    ctx.add_basemap(ax_temp, crs=WEB_MERCATOR, source=ctx.providers.Esri.WorldImagery, zoom=zoom)
    
    # Extract the image
    fig_temp.canvas.draw()
    buf = io.BytesIO()
    plt.savefig(buf, format='png', bbox_inches='tight', pad_inches=0, dpi=150)
    buf.seek(0)
    img = Image.open(buf)
    img_array = np.array(img)
    plt.close(fig_temp)
    
    # Get actual extent from the axes
    extent = (west, east, south, north)
    
    return img_array, extent


def build_geodataframe(lat: np.ndarray, lon: np.ndarray, alt: np.ndarray) -> gpd.GeoDataFrame:
    """Create a GeoDataFrame of point geometries with altitude attribute."""
    points = [Point(xy) for xy in zip(lon, lat)]
    return gpd.GeoDataFrame({"altitude_m": alt}, geometry=points, crs=WGS84)


def draw_3d_trajectory(gdf_wgs84: gpd.GeoDataFrame, meta: dict, output: Path) -> None:
    """Render a 3D trajectory plot with London City Airport map on the ground plane."""
    # Project to Web Mercator for metric coordinates
    gdf_mercator = gdf_wgs84.to_crs(WEB_MERCATOR)
    origin_pt = gpd.GeoSeries([Point(LCY_RUNWAY09_THRESHOLD[::-1])], crs=WGS84).to_crs(WEB_MERCATOR)[0]
    
    # Get x, y coordinates in Web Mercator and altitude
    x = gdf_mercator.geometry.x.to_numpy()
    y = gdf_mercator.geometry.y.to_numpy()
    z = gdf_mercator["altitude_m"].to_numpy()
    
    # Shift coordinates to be relative to airport origin
    x_rel = x - origin_pt.x
    y_rel = y - origin_pt.y
    
    # Get basemap image
    print("Fetching satellite imagery...")
    basemap_img, extent = get_basemap_image(
        LCY_RUNWAY09_THRESHOLD[1], 
        LCY_RUNWAY09_THRESHOLD[0], 
        AIRSPACE_RADIUS_M,
        zoom=15
    )
    
    # Adjust extent to be relative to origin
    extent_rel = (
        extent[0] - origin_pt.x,
        extent[1] - origin_pt.x,
        extent[2] - origin_pt.y,
        extent[3] - origin_pt.y
    )
    
    # Create 3D figure
    fig = plt.figure(figsize=(16, 12), facecolor='white')
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot the basemap on the ground plane (z=0)
    xx, yy = np.meshgrid(
        np.linspace(extent_rel[0], extent_rel[1], basemap_img.shape[1]),
        np.linspace(extent_rel[2], extent_rel[3], basemap_img.shape[0])
    )
    zz = np.zeros_like(xx)
    
    # Flip the image vertically for correct orientation
    ax.plot_surface(
        xx, yy, zz,
        rstride=1, cstride=1,
        facecolors=basemap_img/255.0,
        shade=False,
        alpha=0.9
    )
    
    # Create colored line segments for the 3D trajectory
    points = np.array([x_rel, y_rel, z]).T.reshape(-1, 1, 3)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    
    # Color by altitude
    norm = Normalize(vmin=0, vmax=min(MAX_ALTITUDE_M, z.max()))
    colors = plt.cm.plasma((z[:-1] + z[1:]) / 2.0 / min(MAX_ALTITUDE_M, z.max()))
    
    # Plot 3D trajectory
    for i in range(len(segments)):
        ax.plot(segments[i, :, 0], segments[i, :, 1], segments[i, :, 2],
                color=colors[i], linewidth=2.5, alpha=0.9)
    
    # Highlight start and end points
    ax.scatter([x_rel[0]], [y_rel[0]], [z[0]], 
               c='cyan', s=100, marker='o', edgecolors='black', linewidths=2, label='Start', zorder=10)
    ax.scatter([x_rel[-1]], [y_rel[-1]], [z[-1]], 
               c='yellow', s=100, marker='o', edgecolors='black', linewidths=2, label='End', zorder=10)
    
    # Draw vertical lines from ground to trajectory endpoints
    ax.plot([x_rel[0], x_rel[0]], [y_rel[0], y_rel[0]], [0, z[0]], 
            'c--', alpha=0.5, linewidth=1)
    ax.plot([x_rel[-1], x_rel[-1]], [y_rel[-1], y_rel[-1]], [0, z[-1]], 
            'y--', alpha=0.5, linewidth=1)
    
    # Add runway marker at origin
    runway_length = 1508  # London City Airport runway length in meters
    ax.plot([0, runway_length], [0, 0], [0, 0], 
            'r-', linewidth=4, alpha=0.8, label='Runway 09/27')
    
    # Labels and title
    ax.set_xlabel('X (meters)', fontsize=11, labelpad=10)
    ax.set_ylabel('Y (meters)', fontsize=11, labelpad=10)
    ax.set_zlabel('Altitude (meters)', fontsize=11, labelpad=10)
    
    title = f"{meta.get('plane_id', 'Flight')} - {meta.get('plane_type', 'Aircraft')}\n"
    title += f"Route: {meta.get('arrival_route', 'N/A')} | Max Alt: {z.max():.0f}m"
    ax.set_title(title, fontsize=13, fontweight='bold', pad=20)
    
    # Set axis limits
    max_range = AIRSPACE_RADIUS_M
    ax.set_xlim(-max_range, max_range)
    ax.set_ylim(-max_range, max_range)
    ax.set_zlim(0, max(z.max() * 1.1, 100))
    
    # Set viewing angle
    ax.view_init(elev=25, azim=45)
    
    # Add legend
    ax.legend(loc='upper left', fontsize=10)
    
    # Add colorbar for altitude
    sm = plt.cm.ScalarMappable(cmap='plasma', norm=norm)
    sm.set_array([])
    cbar = plt.colorbar(sm, ax=ax, fraction=0.02, pad=0.1, shrink=0.6)
    cbar.set_label('Altitude (m)', fontsize=10)
    
    # Grid
    ax.grid(True, alpha=0.3)
    
    # Save
    output.parent.mkdir(parents=True, exist_ok=True)
    plt.tight_layout()
    plt.savefig(output, dpi=200, bbox_inches='tight', facecolor='white')
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot LCY trajectory JSON in 3D with satellite basemap.")
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
        args.output = Path(f"flight_3d_{plane_name}.png")

    meta, coords = load_trajectory(args.trajectory_json)
    lat, lon, alt = local_to_wgs84(coords)
    gdf = build_geodataframe(lat, lon, alt)
    draw_3d_trajectory(gdf, meta, args.output)
    print(f"✓ 3D trajectory plot saved to: {args.output}")


if __name__ == "__main__":
    main()
