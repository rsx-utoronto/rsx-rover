import json
from pathlib import Path
import urllib.request

ELEV_API_LIMIT = 100
GRID_PRECISION = 0.01


area = [
  (43.658313, -79.398170),
  (44.652854, -79.424138)
]

API_URL = 'https://api.open-meteo.com/v1/elevation'

def getElevFromPoints(lats: list[float], longs: list[float]) -> list[float]:
  """Get elevation using latitude and longitude from API."""
  with urllib.request.urlopen(
    f"{API_URL}?latitude={','.join(str(l) for l in lats)}&longitude={','.join(str(l) for l in longs)}"
  ) as response:
    res = response.read().decode('utf-8')
    res_json = json.loads(res)
    if "elevation" in res_json:
      return res_json["elevation"]
    return [-1.0]

def main():
  
  # Extract corner coordinates
  # Determine min and max for each dimension
  lat1, lon1 = area[0]
  lat2, lon2 = area[1]
  
  min_lat = min(lat1, lat2)
  max_lat = max(lat1, lat2)
  min_lon = min(lon1, lon2)
  max_lon = max(lon1, lon2)
  
  elevation_data = {}

  lats = []
  longs = []

  lat = min_lat
  while lat <= max_lat:
    lon = min_lon
    while lon <= max_lon:

      # start a fetch if there are 100 coords
      if len(lats) == ELEV_API_LIMIT:
        elevations = getElevFromPoints(lats, longs)
        for i in range(len(elevations)):
          elevation_data[f"{lats[i]},{longs[i]}"] = elevations[i]
        lats.clear()
        longs.clear()

      lat_rounded = round(lat, 2)
      lon_rounded = round(lon, 2)
      
      lats.append(lat_rounded)
      longs.append(lon_rounded)
      
      lon += GRID_PRECISION
    lat += GRID_PRECISION
  
  output_file = Path(__file__).parent.resolve() / "elevation.json"
  with open(output_file, 'x') as f:
    json.dump(elevation_data, f, indent=2)
  
  print(f"\nElevation data saved to {output_file}")
  print(f"Total coordinates processed: {len(elevation_data)}")

if __name__ == "__main__":
  main()
  