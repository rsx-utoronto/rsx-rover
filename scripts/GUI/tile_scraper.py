#!/usr/bin/env python3.8

import threading, queue
import math
from collections import namedtuple
from enum import Enum
from typing import List, Tuple
from pathlib import Path
import time

import pyproj
import urllib.request

tile_q = queue.Queue()
CACHE_DIR = Path(__file__).parent.resolve() / "tile_cache"

class TileRequest():
	def __init__(self, image_path: Path, tile_url: str):
		self.image_path = image_path
		self.tile_url = tile_url

MapServer = namedtuple('MapServer', 'name simple_name tile_url layer_count')

class MapServers(MapServer, Enum):
	ARCGIS_World_Imagery = MapServer(
		'ARCGIS World Imagery (Internet)',
		'arcgis_world_imagery',
		r"https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
		19)
	ARCGIS_World_Imagery_Cache = MapServer(
		'ARCGIS World Imagery (Cache)',
		'arcgis_world_imagery',
		str(CACHE_DIR) + '/arcgis_world_imagery/{z}/{y}/{x}.jpg',
		19)
	Laz_Slope_Costmap = MapServer(
		'Laz Slope Costmap (Cache)',
		'laz_slope_costmap',
		str(CACHE_DIR) + '/laz_slope_costmap/{z}/{y}/{x}.png',
		17)

	@classmethod
	def names(self) -> List[str]:
		def get_name(member):
			return member.name
		return list(map(get_name, self))

	@classmethod
	def values(self) -> List[MapServer]:
		def get_value(member):
			return member
		return list(map(get_value, self))

	@classmethod
	def zip(self) -> List[Tuple[str, MapServer]]:
		def get_name_and_value(member):
			return member.name, member
		return list(map(get_name_and_value, self))

def download_tile(tile_req: TileRequest):
	if tile_req.image_path.exists():
		return
	tile_req.image_path.parent.mkdir(parents=True, exist_ok=True)
	try:
		urllib.request.urlretrieve(tile_req.tile_url, str(tile_req.image_path))
		time.sleep(1)
	except urllib.error.HTTPError:
		pass

def tile_downloader():
	while True:
		try:
			tile = tile_q.get(timeout=1)
		except queue.Empty:
			return

		download_tile(tile)

		tile_q.task_done()

def minmax(*l):
	return min(l), max(l)

def main():
	morgantown = [  # lat, long
		(39.621425, -79.988966),  # bottom left
		(39.672974, -79.912062),  # top right
	]

	neo_mars = [
		(38.380691, -110.829048),
		(38.439607, -110.751286),
	]

	ashpits = [
		(39.715770, -79.948969),
		(39.747850, -79.899874),
	]

	farm = [
		(39.495907, -79.850414),
		(39.518943, -79.813609),
	]
	front_campus = [
		(43.6601930, -79.3965910),
		(43.6627688, -79.3944725)
	]

	locations = [
		front_campus
	]

	internet_map_servers = [MapServers.ARCGIS_World_Imagery]

	all_tile_requests: List[TileRequest] = []
	for map_server in internet_map_servers:
		for location in locations:
			all_tile_requests += get_tile_requests_for(*location, map_server)

	needed_tiles = []
	for tile_request in all_tile_requests:
		if not tile_request.image_path.exists():
			needed_tiles.append(tile_request)

	print(f"Caching {len(needed_tiles)} tiles")

	for _ in range(32):
		threading.Thread(target=tile_downloader, daemon=True).start()

	for tile in needed_tiles:
		tile_q.put(tile)

	tile_q.join()

	print(f"Done caching")

def get_tile_requests_for(cornera, cornerb, map_server: MapServer) -> List[TileRequest]:
	meters_crs = 3857
	latlon_crs = 4326
	latlon2meters = pyproj.Transformer.from_crs(latlon_crs, meters_crs)

	mxa, mya = latlon2meters.transform(*cornera)
	mxb, myb = latlon2meters.transform(*cornerb)

	mercator = GlobalMercator()
	tiles = []

	for tz in range(0, map_server.layer_count):
		txa, tya = mercator.GoogleTile(*mercator.MetersToTile(mxa, mya, tz), tz)
		txb, tyb = mercator.GoogleTile(*mercator.MetersToTile(mxb, myb, tz), tz)

		tminx, tmaxx = minmax(txa, txb)
		tminy, tmaxy = minmax(tya, tyb)

		for tx in range(tminx, tmaxx + 1):
			for ty in range(tminy, tmaxy + 1):
				image_path = Path(f"{CACHE_DIR}/{map_server.simple_name}/{tz}/{ty}/{tx}.jpg")
				tile_url = map_server.tile_url.format(z=tz, y=ty, x=tx)
				tiles.append(TileRequest(image_path, tile_url))

	return tiles

class GlobalMercator:
	# Based on: https://gist.github.com/maptiler/fddb5ce33ba995d5523de9afdf8ef118#file-globalmaptiles-py
	def __init__(self, tileSize=256):
		"Initialize the TMS Global Mercator pyramid"
		EQUITORIAL_RADIUS = 6378137
		self.tileSize = tileSize
		self.initialResolution = 2 * math.pi * EQUITORIAL_RADIUS / self.tileSize
		# 156543.03392804062 for tileSize 256 pixels
		self.originShift = 2 * math.pi * EQUITORIAL_RADIUS / 2.0
		# 20037508.342789244

	def LatLonToMeters(self, lat, lon ):
		"Converts given lat/lon in WGS84 Datum to XY in Spherical Mercator EPSG:900913"

		mx = lon * self.originShift / 180.0
		my = math.log( math.tan((90 + lat) * math.pi / 360.0 )) / (math.pi / 180.0)

		my = my * self.originShift / 180.0
		return mx, my

	def MetersToLatLon(self, mx, my ):
		"Converts XY point from Spherical Mercator EPSG:900913 to lat/lon in WGS84 Datum"

		lon = (mx / self.originShift) * 180.0
		lat = (my / self.originShift) * 180.0

		lat = 180 / math.pi * (2 * math.atan( math.exp( lat * math.pi / 180.0)) - math.pi / 2.0)
		return lat, lon

	def PixelsToMeters(self, px, py, zoom):
		"Converts pixel coordinates in given zoom level of pyramid to EPSG:900913"

		res = self.Resolution( zoom )
		mx = px * res - self.originShift
		my = py * res - self.originShift
		return mx, my
		
	def MetersToPixels(self, mx, my, zoom):
		"Converts EPSG:900913 to pyramid pixel coordinates in given zoom level"
				
		res = self.Resolution( zoom )
		px = (mx + self.originShift) / res
		py = (my + self.originShift) / res
		return px, py
	
	def PixelsToTile(self, px, py):
		"Returns a tile covering region in given pixel coordinates"

		tx = int( math.ceil( px / float(self.tileSize) ) - 1 )
		ty = int( math.ceil( py / float(self.tileSize) ) - 1 )
		return tx, ty

	def PixelsToRaster(self, px, py, zoom):
		"Move the origin of pixel coordinates to top-left corner"
		
		mapSize = self.tileSize << zoom
		return px, mapSize - py
		
	def MetersToTile(self, mx, my, zoom):
		"Returns tile for given mercator coordinates"
		
		px, py = self.MetersToPixels( mx, my, zoom)
		return self.PixelsToTile( px, py)

	def TileBounds(self, tx, ty, zoom):
		"Returns bounds of the given tile in EPSG:900913 coordinates"
		
		minx, miny = self.PixelsToMeters( tx*self.tileSize, ty*self.tileSize, zoom )
		maxx, maxy = self.PixelsToMeters( (tx+1)*self.tileSize, (ty+1)*self.tileSize, zoom )
		return ( minx, miny, maxx, maxy )

	def TileLatLonBounds(self, tx, ty, zoom ):
		"Returns bounds of the given tile in latutude/longitude using WGS84 datum"

		bounds = self.TileBounds( tx, ty, zoom)
		minLat, minLon = self.MetersToLatLon(bounds[0], bounds[1])
		maxLat, maxLon = self.MetersToLatLon(bounds[2], bounds[3])
		 
		return ( minLat, minLon, maxLat, maxLon )
		
	def Resolution(self, zoom ):
		"Resolution (meters/pixel) for given zoom level (measured at Equator)"
		
		# return (2 * math.pi * 6378137) / (self.tileSize * 2**zoom)
		return self.initialResolution / (2**zoom)
		
	def ZoomForPixelSize(self, pixelSize ):
		"Maximal scaledown zoom of the pyramid closest to the pixelSize."
		
		for i in range(30):
			if pixelSize > self.Resolution(i):
				return i-1 if i!=0 else 0 # We don't want to scale up

	def GoogleTile(self, tx, ty, zoom):
		"Converts TMS tile coordinates to Google Tile coordinates"
		
		# coordinate origin is moved from bottom-left to top-left corner of the extent
		return tx, (2**zoom - 1) - ty

	def QuadTree(self, tx, ty, zoom ):
		"Converts TMS tile coordinates to Microsoft QuadTree"
		
		quadKey = ""
		ty = (2**zoom - 1) - ty
		for i in range(zoom, 0, -1):
			digit = 0
			mask = 1 << (i-1)
			if (tx & mask) != 0:
				digit += 1
			if (ty & mask) != 0:
				digit += 2
			quadKey += str(digit)
			
		return quadKey

if __name__ == "__main__":
	main()