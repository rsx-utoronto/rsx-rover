#!/usr/bin/env python3
"""
Compute terrain gradient (slope vector field) from an elevation CSV.

Input (same directory):
    Elevation_Toronto_Front_Campus.csv

Output (same directory):
    Elevation_Toronto_Front_Campus_gradient.csv
"""

import numpy as np
import pandas as pd
from pathlib import Path

# -------------------- CONFIG --------------------
# INPUT_FILENAME = "Elevation_urc_desert_hanksville.csv"
INPUT_FILENAME = "Elevation_toronto_front_campus.csv"

LON_COL = "longitude"
LAT_COL = "latitude"
Z_COL   = "elevation_m"

EARTH_RADIUS_M = 6_371_000.0
# ------------------------------------------------


def meters_per_degree(lat_deg: float):
    """Approx meters per degree at a given latitude."""
    lat = np.deg2rad(lat_deg)
    m_per_deg_lat = (np.pi / 180.0) * EARTH_RADIUS_M
    m_per_deg_lon = (np.pi / 180.0) * EARTH_RADIUS_M * np.cos(lat)
    return m_per_deg_lon, m_per_deg_lat


def main():
    script_dir = Path(__file__).parent.resolve()
    input_path = script_dir / INPUT_FILENAME

    if not input_path.exists():
        raise FileNotFoundError(f"Input file not found: {input_path}")

    df = pd.read_csv(input_path)

    for c in (LON_COL, LAT_COL, Z_COL):
        if c not in df.columns:
            raise ValueError(f"Missing column: {c}")

    # Sort grid
    lons = np.sort(df[LON_COL].unique())
    lats = np.sort(df[LAT_COL].unique())

    Z = (
        df.pivot(index=LAT_COL, columns=LON_COL, values=Z_COL)
          .loc[lats, lons]
          .values
    )

    # Grid spacing
    dlon = np.median(np.diff(lons))
    dlat = np.median(np.diff(lats))
    lat0 = float(np.median(lats))

    m_per_deg_lon, m_per_deg_lat = meters_per_degree(lat0)
    dx = dlon * m_per_deg_lon
    dy = dlat * m_per_deg_lat

    # Gradients
    gy, gx = np.gradient(Z, dy, dx)

    slope_mag = np.sqrt(gx**2 + gy**2)
    slope_deg = np.rad2deg(np.arctan(slope_mag))

    # Flatten for CSV
    Lon, Lat = np.meshgrid(lons, lats)

    out_df = pd.DataFrame({
        "lat": Lat.ravel(),
        "lon": Lon.ravel(),
        "elevation": Z.ravel(),
        "gx": gx.ravel(),
        "gy": gy.ravel(),
        "slope_mag": slope_mag.ravel(),
        "slope_deg": slope_deg.ravel()
    })


    output_path = script_dir / "vector_field_front_campus.csv"
    out_df.to_csv(output_path, index=False)

    print("Gradient computation complete.")
    print(f"Saved to: {output_path}")
    print(f"Grid size: {Z.shape[0]} x {Z.shape[1]}")
    print(f"dx = {dx:.2f} m, dy = {dy:.2f} m")


if __name__ == "__main__":
    main()