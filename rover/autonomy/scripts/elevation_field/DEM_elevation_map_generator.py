import rasterio
import numpy as np
import matplotlib.pyplot as plt
from pyproj import Transformer
from rasterio.transform import rowcol
from scipy.ndimage import sobel
import heapq

# Rover GPS location (CHANGE if needed)
lat = 43.660
lon = -79.396

# Desired clip size in meters
clip_size_m = 550  # 100m x 100m

with rasterio.open("dem_0p5m.tif") as src:
    dem = src.read(1)
    dem[dem == -9999.0] = np.nan
    
    transform = src.transform
    resolution = src.res[0]  # pixel size (0.5m)
    dem_crs = src.crs
    
    # Convert lat/lon (EPSG:4326) → DEM CRS (meters)
    transformer = Transformer.from_crs("EPSG:4326", dem_crs, always_xy=True)
    x, y = transformer.transform(lon, lat)
    
    # Convert x/y → row/col in raster
    col, row = rowcol(transform, x, y)
    
    # compute half window in pixels
    half_window = int(clip_size_m / resolution / 2)
    
    # Clip DEM safely
    row_start = max(row - half_window, 0)
    row_end   = min(row + half_window, dem.shape[0])
    col_start = max(col - half_window, 0)
    col_end   = min(col + half_window, dem.shape[1])
    
    dem_clip = dem[row_start:row_end, col_start:col_end]

print("Full DEM shape:", dem.shape)
print("Clipped DEM shape:", dem_clip.shape)
print("Elevation range:", np.nanmin(dem_clip), np.nanmax(dem_clip))

# Visualize clipped DEM
plt.figure(figsize=(6,6))
plt.imshow(dem_clip, cmap="terrain")
plt.colorbar(label="Elevation (m)")
plt.scatter(col - col_start, row - row_start, c='red', s=50)
plt.title(f"{clip_size_m}m x {clip_size_m}m LiDAR DEM")
plt.show()

# -----------------------------
# Generate safe A*-ready cost map
# -----------------------------

# Compute slope-based cost
sx = sobel(dem_clip, axis=1, mode='nearest')
sy = sobel(dem_clip, axis=0, mode='nearest')
slope = np.hypot(sx, sy)

# Normalize slope to 0–1
slope_norm = slope / np.nanmax(slope)

# Replace NaNs with high cost
cost_map = np.copy(slope_norm)
cost_map[np.isnan(cost_map)] = 1.0  # max cost

# Optional: exaggerate slope effect
cost_map = cost_map ** 2

# Visualize cost map
plt.figure(figsize=(6,6))
plt.imshow(cost_map, cmap='Reds', origin='upper')
plt.colorbar(label='Traversal Cost (0=easy, 1=hard)')
plt.scatter(col - col_start, row - row_start, c='blue', s=50, label='Rover')
plt.title("A*-Ready Cost Map")
plt.legend()
plt.show()

# -----------------------------
# Define start and goal inside valid area
# -----------------------------
start = (row - row_start, col - col_start)          # center
goal  = (start[0] - 100, start[1] + 50)             # 50 pixels away

# Clamp goal inside map
goal = (min(goal[0], cost_map.shape[0]-1), min(goal[1], cost_map.shape[1]-1))

# -----------------------------
# A* implementation
# -----------------------------
def heuristic(a, b):
    return np.hypot(a[0] - b[0], a[1] - b[1])

def astar(cost_map, start, goal):
    rows, cols = cost_map.shape
    open_set = []
    heapq.heappush(open_set, (0 + heuristic(start, goal), 0, start, [start]))
    visited = set()

    while open_set:
        f, g, current, path = heapq.heappop(open_set)
        if current in visited:
            continue
        visited.add(current)
        if current == goal:
            return path

        r, c = current
        # 8-connected neighbors
        for dr in [-1, 0, 1]:
            for dc in [-1, 0, 1]:
                if dr == 0 and dc == 0:
                    continue
                nr, nc = r + dr, c + dc
                if 0 <= nr < rows and 0 <= nc < cols:
                    step_cost = np.hypot(dr, dc) * (1 + cost_map[nr, nc])
                    heapq.heappush(open_set, (g + step_cost + heuristic((nr, nc), goal),
                                              g + step_cost, (nr, nc), path + [(nr, nc)]))
    return None

# -----------------------------
# Run A* and visualize path
# -----------------------------
path = astar(cost_map, start, goal)

if path is None:
    print("No path found!")
else:
    print(f"Path found with {len(path)} steps")
    plt.figure(figsize=(6,6))
    plt.imshow(cost_map, cmap='Reds', origin='upper')
    plt.colorbar(label='Traversal Cost')
    path_rows, path_cols = zip(*path)
    plt.plot(path_cols, path_rows, c='blue', linewidth=2, label='A* Path')
    plt.scatter(start[1], start[0], c='green', s=50, label='Start')
    plt.scatter(goal[1], goal[0], c='purple', s=50, label='Goal')
    plt.title("A* Path on LiDAR Cost Map")
    plt.legend()
    plt.show()