from dataclasses import dataclass
import threading, queue
from enum import Enum
from pathlib import Path
import time

import pyproj
import urllib.request
import urllib.error

from global_mercator import GlobalMercator

CACHE_DIR = Path(__file__).parent.resolve() / "tile_cache"
tile_q = queue.Queue()

@dataclass
class TileRequest:
	"""Represents a request for a map tile."""
	image_path: Path
	tile_url: str

@dataclass
class MapServer:
	"""Represents a map server configuration."""
	display_name: str
	simple_name: str
	tile_url: str
	layer_count: int

class MapServers(Enum):
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


def download_tile(tile_req: TileRequest):
	if tile_req.image_path.exists():
		return
	tile_req.image_path.parent.mkdir(parents=True, exist_ok=True)
	try:
		urllib.request.urlretrieve(tile_req.tile_url, str(tile_req.image_path))
		time.sleep(1)
	except urllib.error.HTTPError as e:
		print(e)


def tile_downloader():
	while True:
		try:
			tile = tile_q.get(timeout=1)
		except queue.Empty:
			return

		download_tile(tile)

		tile_q.task_done()

def minmax(*l: int):
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
	# (43.6601930, -79.3965910),
	# 	(43.6627688, -79.3944725)
	front_campus = [
		(43.658313, -79.398170),
		(43.662854, -79.394138)
	]

	woodbine_beach = [
		(43.670502, -79.3165272),
		(43.6598290, -79.2778141)
	]

	mdrs = [
		(38.431078, -110.810864),
		(38.395515, -110.769358)
	]

	utias = [
		(43.783178, -79.4682194),
		(43.780778, -79.46268056)
	]

	locations = [
		front_campus,
		woodbine_beach,
		# mdrs
	]

	internet_map_servers = [MapServers.ARCGIS_World_Imagery]

	all_tile_requests: list[TileRequest] = []
	for map_server in internet_map_servers:
		for location in locations:
			all_tile_requests += get_tile_requests_for(location[0], location[1], map_server.value)

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

def get_tile_requests_for(cornera: tuple[float, float], cornerb: tuple[float, float], map_server: MapServer) -> list[TileRequest]:
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


if __name__ == "__main__":
	main()