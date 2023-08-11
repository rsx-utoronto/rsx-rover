import math
def deg2num(lat_deg, lon_deg, zoom):
  lat_rad = math.radians(lat_deg)
  n = 1 << zoom
  xtile = int((lon_deg + 180.0) / 360.0 * n)
  ytile = int((1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n)
  return xtile, ytile

zoom = 18
lat = 43.660546
long = -79.395972
x, y = deg2num(lat, long, zoom)

print(f'https://tile.openstreetmap.org/{zoom}/{x}/{y}.png')