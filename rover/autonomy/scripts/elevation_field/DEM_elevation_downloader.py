#!/usr/bin/env python3


# this file prints the tile name that is in the coordinate we want and then the
# download url. To download the url, type wget in the terminal.

import geopandas as gpd
from shapely.geometry import Point


# Path to the .shp file (they should all be in teh same directory)
gdf = gpd.read_file("OntarioClassifiedPC_LidarDerived_TileIndex.shp")

print(gdf.head())
print(gdf.columns)
print(gdf.crs)


# campus point in lat/lon
point = gpd.GeoSeries([Point(-79.396, 43.660)], crs="EPSG:4326")

# convert to tile CRS
point_3857 = point.to_crs(gdf.crs)

# find tile containing it
tile = gdf[gdf.contains(point_3857.iloc[0])]

print(tile)
print(tile["Download"].iloc[0]) #trield iloc 1 and found that 0 contained our rover better 