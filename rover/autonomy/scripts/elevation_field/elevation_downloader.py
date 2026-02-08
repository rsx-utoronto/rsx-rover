#!/usr/bin/env python3
import csv
import json
import math
from pathlib import Path
import urllib.request
import time

# API Configuration
ELEV_API_LIMIT = 100  # Max points per API request
API_URL = 'https://api.open-meteo.com/v1/elevation'

# Grid Configuration
GRID_SIZE_M = 50.0        # 50m x 50m area
RESOLUTION_M = 0.5        # 0.5m resolution
POINTS_PER_SIDE = int(GRID_SIZE_M / RESOLUTION_M)  # 100 points per side

# Earth radius for coordinate conversion
EARTH_RADIUS_M = 6371000.0

# URC Desert Location (Hanksville, Utah area)
# 38.41° N, 110.79° W
URC_DESERT_CENTER = (38.41, -110.79)

# Toronto Front Campus (for testing)
TORONTO_CENTER = (43.660, -79.396)


def meters_to_degrees(meters_lat: float, meters_lon: float, ref_lat: float) -> tuple:
    """Convert meters to degrees at a given latitude."""
    # Latitude: 1 degree ≈ 111km
    deg_lat = meters_lat / 111000.0
    # Longitude: depends on latitude (shrinks toward poles)
    deg_lon = meters_lon / (111000.0 * math.cos(math.radians(ref_lat)))
    return deg_lat, deg_lon


def get_elevation_from_points(lats: list, lons: list) -> list:
    """Get elevation using latitude and longitude from Open-Meteo API."""
    lat_str = ','.join(f"{lat:.7f}" for lat in lats)
    lon_str = ','.join(f"{lon:.7f}" for lon in lons)
    url = f"{API_URL}?latitude={lat_str}&longitude={lon_str}"
    
    try:
        with urllib.request.urlopen(url, timeout=30) as response:
            res = response.read().decode('utf-8')
            res_json = json.loads(res)
            if "elevation" in res_json:
                return res_json["elevation"]
    except Exception as e:
        print(f"API Error: {e}")
    return [-1.0] * len(lats)


def generate_grid(center_lat: float, center_lon: float) -> list:
    """Generate grid points centered on the given coordinate."""
    points = []
    
    # Calculate degree offsets for 0.5m at this latitude
    half_size = GRID_SIZE_M / 2.0
    deg_per_point_lat, deg_per_point_lon = meters_to_degrees(RESOLUTION_M, RESOLUTION_M, center_lat)
    
    # Starting corner (southwest)
    deg_offset_lat, deg_offset_lon = meters_to_degrees(half_size, half_size, center_lat)
    start_lat = center_lat - deg_offset_lat
    start_lon = center_lon - deg_offset_lon
    
    print(f"Grid Configuration:")
    print(f"  Center: {center_lat:.6f}, {center_lon:.6f}")
    print(f"  Size: {GRID_SIZE_M}m x {GRID_SIZE_M}m")
    print(f"  Resolution: {RESOLUTION_M}m ({POINTS_PER_SIDE}x{POINTS_PER_SIDE} = {POINTS_PER_SIDE**2} points)")
    print(f"  Lat step: {deg_per_point_lat:.8f}° ({RESOLUTION_M}m)")
    print(f"  Lon step: {deg_per_point_lon:.8f}° ({RESOLUTION_M}m)")
    
    for i in range(POINTS_PER_SIDE):
        for j in range(POINTS_PER_SIDE):
            lat = start_lat + i * deg_per_point_lat
            lon = start_lon + j * deg_per_point_lon
            points.append((lat, lon))
    
    return points


def fetch_elevations(points: list) -> list:
    """Fetch elevations for all points, batching API requests."""
    elevations = []
    total_batches = math.ceil(len(points) / ELEV_API_LIMIT)
    
    print(f"\nFetching elevations ({total_batches} API calls needed)...")
    
    for batch_num in range(total_batches):
        start_idx = batch_num * ELEV_API_LIMIT
        end_idx = min(start_idx + ELEV_API_LIMIT, len(points))
        batch = points[start_idx:end_idx]
        
        lats = [p[0] for p in batch]
        lons = [p[1] for p in batch]
        
        batch_elevs = get_elevation_from_points(lats, lons)
        elevations.extend(batch_elevs)
        
        progress = (batch_num + 1) / total_batches * 100
        print(f"  Batch {batch_num + 1}/{total_batches} ({progress:.1f}%) - {len(batch)} points")
        
        # Small delay to avoid rate limiting
        if batch_num < total_batches - 1:
            time.sleep(0.1)
    
    return elevations


def save_to_csv(points: list, elevations: list, output_path: Path):
    """Save elevation data to CSV file."""
    with open(output_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['latitude', 'longitude', 'elevation_m'])
        for (lat, lon), elev in zip(points, elevations):
            writer.writerow([f"{lat:.7f}", f"{lon:.7f}", f"{elev:.2f}"])
    print(f"\nSaved to: {output_path}")


def save_to_json(points: list, elevations: list, output_path: Path):
    """Save elevation data to JSON file."""
    data = {
        f"{lat:.7f},{lon:.7f}": elev 
        for (lat, lon), elev in zip(points, elevations)
    }
    with open(output_path, 'w') as f:
        json.dump(data, f, indent=2)
    print(f"Saved to: {output_path}")


def process_location(center: tuple, name: str):
    """Process a location and save elevation data."""
    print(f"\n{'='*60}")
    print(f"Processing: {name}")
    print(f"{'='*60}")
    
    center_lat, center_lon = center
    
    # Generate grid points
    points = generate_grid(center_lat, center_lon)
    
    # Fetch elevations
    elevations = fetch_elevations(points)
    
    # Calculate stats
    valid_elevs = [e for e in elevations if e != -1.0]
    if valid_elevs:
        print(f"\nElevation Statistics:")
        print(f"  Min: {min(valid_elevs):.2f}m")
        print(f"  Max: {max(valid_elevs):.2f}m")
        print(f"  Avg: {sum(valid_elevs)/len(valid_elevs):.2f}m")
        print(f"  Range: {max(valid_elevs) - min(valid_elevs):.2f}m")
    
    # Save files
    output_dir = Path(__file__).parent.resolve()
    safe_name = name.lower().replace(' ', '_').replace(',', '')
    
    csv_path = output_dir / f"elevation_{safe_name}.csv"
    json_path = output_dir / f"elevation_{safe_name}.json"
    
    save_to_csv(points, elevations, csv_path)
    save_to_json(points, elevations, json_path)
    
    return points, elevations


if __name__ == "__main__":
    # Process URC Desert location
    #process_location(URC_DESERT_CENTER, "urc_desert_hanksville")
    
    # Uncomment to also process Toronto
    process_location(TORONTO_CENTER, "toronto_front_campus")
  