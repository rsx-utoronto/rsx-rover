import json
import math
from pathlib import Path
import urllib.request

ELEV_API_LIMIT = 100
GRID_PRECISION = 0.001


# area = [
#   (43.658313, -79.398170),
#   (44.652854, -79.424138)
# ]

front_campus = [
  (43.658313, -79.398170),
  (43.662854, -79.394138)
]

woodbine_beach = [
  (43.670502, -79.3165272),
  (43.6598290, -79.2778141)
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

def process_area(area):
  """Process a rectangular area defined by two corner coordinates."""
  
  # Extract corner coordinates
  # Determine min and max for each dimension
  lat1, lon1 = area[0]
  lat2, lon2 = area[1]
  
  min_lat = min(lat1, lat2)
  max_lat = max(lat1, lat2)
  min_lon = min(lon1, lon2)
  max_lon = max(lon1, lon2)
  
  elevation_data = {}

  lats_longs = []

  lat = min_lat
  while lat <= max_lat:
    lon = min_lon
    while lon <= max_lon:

      # start a fetch if there are 100 coords
      if len(lats_longs) == ELEV_API_LIMIT:
        print(lats_longs)
        elevations = getElevFromPoints(
          [p[0] for p in lats_longs], [p[1] for p in lats_longs]
        )
        for i in range(len(elevations)):
          elevation_data[f"{lats_longs[i][0]},{lats_longs[i][1]}"] = elevations[i]
        lats_longs.clear()

      decimals = -int(math.log10(GRID_PRECISION))
      lat_rounded = round(lat, decimals)
      lon_rounded = round(lon, decimals)
      print(lat, lat_rounded, lon, lon_rounded)
      lats_longs.append((lat_rounded, lon_rounded))      
      lon += GRID_PRECISION
    lat += GRID_PRECISION
  
  if len(lats_longs) > 0:
    elevations = getElevFromPoints(
      [p[0] for p in lats_longs], [p[1] for p in lats_longs]
    )
    for i in range(len(elevations)):
      elevation_data[f"{lats_longs[i][0]},{lats_longs[i][1]}"] = elevations[i]
  return elevation_data


if __name__ == "__main__":
  areas = [
    front_campus,
    # woodbine_beach
  ]
  elevation_data_all = {}
  for area in areas:
    elevation_data_all.update(process_area(area))

  output_file = Path(__file__).parent.resolve() / "elevation.json"
  with open(output_file, 'x' if not output_file.exists() else "w") as f:
    json.dump(elevation_data_all, f, indent=2)
  
  print(f"\nElevation data saved to {output_file}")
  print(f"Total coordinates processed: {len(elevation_data_all)}")
  