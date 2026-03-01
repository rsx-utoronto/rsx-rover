import rasterio
import numpy as np
import matplotlib.pyplot as plt
from pyproj import Transformer
from rasterio.transform import rowcol
from scipy.ndimage import sobel, distance_transform_edt
import heapq


def run_dem_astar(glob_msg, target_name):

    # -----------------------------
    # Get rover's current position using odom offset from GPS start anchor
    # -----------------------------
    start_lat, start_lon = glob_msg.locations["start"]

    # Guard against odom not yet received
    if glob_msg.current_position is None:
        odom_x, odom_y = 0.0, 0.0
    else:
        odom_x, odom_y, _ = glob_msg.current_position  # meters relative to start

    # Desired clip size in meters
    clip_size_m = 550

    with rasterio.open("dem_0p5m.tif") as src:
        dem = src.read(1)
        dem[dem == -9999.0] = np.nan

        transform = src.transform
        resolution = src.res[0]  # pixel size (0.5m)
        dem_crs = src.crs

        # Convert start GPS anchor to projected coordinates
        transformer = Transformer.from_crs("EPSG:4326", dem_crs, always_xy=True)
        start_x, start_y = transformer.transform(start_lon, start_lat)

        # Apply live odom offset to get rover's actual current position
        x = start_x + odom_x
        y = start_y + odom_y

        # Convert x/y → row/col in raster
        col, row = rowcol(transform, x, y)

        # Compute half window in pixels
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
    plt.figure(figsize=(6, 6))
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

    # Exaggerate slope effect
    cost_map = cost_map ** 2

    # Visualize cost map
    plt.figure(figsize=(6, 6))
    plt.imshow(cost_map, cmap='Reds', origin='upper')
    plt.colorbar(label='Traversal Cost (0=easy, 1=hard)')
    plt.scatter(col - col_start, row - row_start, c='blue', s=50, label='Rover')
    plt.title("A*-Ready Cost Map")
    plt.legend()
    plt.show()

    # -----------------------------
    # Define start and goal inside valid area
    # -----------------------------
    start = (row - row_start, col - col_start)  # rover's current pixel position

    # Convert goal GPS coordinates to pixel position in clipped DEM
    goal_lat, goal_lon = glob_msg.locations[target_name]
    goal_x, goal_y = transformer.transform(goal_lon, goal_lat)
    goal_col_abs, goal_row_abs = rowcol(transform, goal_x, goal_y)
    goal = (goal_row_abs - row_start, goal_col_abs - col_start)

    # Clamp goal inside map bounds (both sides)
    goal = (
        max(0, min(goal[0], cost_map.shape[0] - 1)),
        max(0, min(goal[1], cost_map.shape[1] - 1))
    )

    # -----------------------------
    # Inflate obstacles to keep path >= 1.0 m away
    # -----------------------------
    buffer_m = 1
    buffer_pixels = int(np.ceil(buffer_m / resolution))

    # Obstacle where dem_clip has no data
    obstacle_mask = np.isnan(dem_clip)

    if np.any(obstacle_mask):
        # Distance (in pixels) from each free cell to nearest obstacle
        dist_pixels = distance_transform_edt(~obstacle_mask)
        dist_m = dist_pixels * resolution
        inflated = dist_m < buffer_m
        cost_map[inflated] = 1.0  # make these cells very costly to traverse
    else:
        # Fallback: treat very steep cells as obstacles
        slope_thresh = 0.9
        steep_mask = slope_norm > slope_thresh
        if np.any(steep_mask):
            dist_pixels = distance_transform_edt(~steep_mask)
            dist_m = dist_pixels * resolution
            inflated = dist_m < buffer_m
            cost_map[inflated] = 1.0

    # Ensure start/goal aren't blocked by inflation
    sr, sc = start
    gr, gc = goal
    cost_map[sr, sc] = min(cost_map[sr, sc], 0.0)
    cost_map[gr, gc] = min(cost_map[gr, gc], 0.0)

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
        plt.figure(figsize=(6, 6))
        plt.imshow(cost_map, cmap='Reds', origin='upper')
        plt.colorbar(label='Traversal Cost')
        path_rows, path_cols = zip(*path)
        plt.plot(path_cols, path_rows, c='blue', linewidth=2, label='A* Path')
        plt.scatter(start[1], start[0], c='green', s=50, label='Start')
        plt.scatter(goal[1], goal[0], c='purple', s=50, label='Goal')
        plt.title("A* Path on LiDAR Cost Map")
        plt.legend()
        plt.show()

    return path