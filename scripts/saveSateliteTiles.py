#!/usr/bin/python3
import folium
import json
import os
import math
from selenium import webdriver

# Set up the map details
latitude = 38.4063  # Center latitude
longitude = -110.7918 # Center longitude
zoom_start = 25

# File paths
output_folder = 'map_output'
html_file = os.path.join(output_folder, 'map.html')
metadata_file = os.path.join(output_folder, 'map_metadata.json')
png_file = os.path.join(output_folder, 'map_screenshot.png')

# Create output folder if it doesn't exist
os.makedirs(output_folder, exist_ok=True)

# Step 1: Create a folium map with a satellite layer
m = folium.Map(location=[latitude, longitude], zoom_start=zoom_start, tiles="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}", attr="Esri Satellite")

# Save the map with satellite tiles
m.save(html_file)

# Step 2: Calculate map bounds (SW and NE corners)
def lat_lng_to_pixel(lat, lng, zoom):
    scale = 1 << zoom  # 2^zoom
    x = (lng + 180) / 360 * scale
    y = (1 - (math.log(math.tan(math.radians(lat)) + 1 / math.cos(math.radians(lat))) / math.pi)) / 2 * scale
    return x, y

def pixel_to_lat_lng(x, y, zoom):
    scale = 1 << zoom
    lng = x / scale * 360 - 180
    lat_rad = math.atan(math.sinh(math.pi * (1 - 2 * y / scale)))
    lat = math.degrees(lat_rad)
    return lat, lng

# Calculate pixel position of the center
pixel_center_x, pixel_center_y = lat_lng_to_pixel(latitude, longitude, zoom_start)

# Define the size of one tile in pixels (256x256)
tile_size = 256

# The visible width and height in pixels (adjust based on your desired output)
map_width, map_height = 1024, 768  # Resolution of the map area

# Calculate the number of pixels that correspond to the map's bounds
half_width = (map_width / 2) / (tile_size / 2)
half_height = (map_height / 2) / (tile_size / 2)

# Calculate SW and NE pixel positions
sw_x = pixel_center_x - half_width
ne_x = pixel_center_x + half_width
sw_y = pixel_center_y + half_height  # SW is lower on the screen
ne_y = pixel_center_y - half_height  # NE is higher on the screen

# Convert pixel bounds back to lat/lng
sw_lat, sw_lng = pixel_to_lat_lng(sw_x, sw_y, zoom_start)
ne_lat, ne_lng = pixel_to_lat_lng(ne_x, ne_y, zoom_start)

map_metadata = {
    "bounds": {
        "southwest": {"lat": sw_lat, "lng": sw_lng},
        "northeast": {"lat": ne_lat, "lng": ne_lng},
    },
    "center": {"lat": latitude, "lng": longitude},
    "zoom": zoom_start,
}

# Step 3: Save the metadata as a JSON file
with open(metadata_file, 'w') as f:
    json.dump(map_metadata, f, indent=4)

# Step 4: Use Selenium to take a screenshot of the map HTML
options = webdriver.ChromeOptions()
options.add_argument('--headless')
options.add_argument('--no-sandbox')
options.add_argument('--disable-dev-shm-usage')
driver = webdriver.Chrome(options=options)

# Open the HTML file in the browser
driver.get("file://" + os.path.abspath(html_file))

# Set the window size to capture the map
driver.set_window_size(map_width, map_height)

# Save screenshot as PNG
driver.save_screenshot(png_file)
driver.quit()

print("Map HTML, metadata, and PNG screenshot saved:")
print(f"HTML file: {html_file}")
print(f"Metadata file: {metadata_file}")
print(f"PNG file: {png_file}")
