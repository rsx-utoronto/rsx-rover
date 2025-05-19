#!/usr/bin/python3

import math
from collections import deque
import matplotlib.pyplot as plt

import yaml
import os

file_path = os.path.join(os.path.dirname(__file__), "sm_config.yaml")

with open(file_path, "r") as f:
    sm_config = yaml.safe_load(f)

DEFAULT_DIFFICULTY = {
    "GNSS1": 1,
    "GNSS2": 1,
    "AR1": 1,
    "AR2": 1,
    "AR3": 1,
    "OBJ1": 1,
    "OBJ2": 1
}

difficulty = sm_config.get("difficulty", DEFAULT_DIFFICULTY)

def get_loc_name(locations):
    locName = []
    for location in locations.keys():
        locName.append(location)
    return locName

# Calculate the distance between two points using the Haversine formula
# def calculate_distance(point1: tuple, point2: tuple) -> float:
#     lat1, lon1 = point1
#     lat2, lon2 = point2

#     # Convert latitude and longitude from d/home/rsx/rover_ws/src/rsx-rover/rover/autonomy/scripts/optimal_path.pyegrees to radians
#     lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

#     # Haversine formula
#     dlat = lat2 - lat1
#     dlon = lon2 - lon1
#     a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
#     c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

#     # Radius of Earth in kilometers
#     r = 6371.0
#     return r * c
def calculate_distance(point1: tuple, point2: tuple) -> float:
    lat1, lon1 = point1
    lat2, lon2 = point2

    # Convert latitude and longitude from degrees to radians
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

    # Haversine formula
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    # Radius of Earth in kilometers
    r = 6371.0
    return r * c
def calculate_distance_cart(point1: tuple, point2: tuple) -> float:
    dist = math.sqrt((point1[0]-point2[0])**2 + (point1[1]-point2[1])**2)
    return dist
# Calculate the weight of traveling to a location
# 0-1 scale, GNSS -> 1, 1
# AR --> 0.8 for without obstacles, 0.5 for obstacles
def calculate_location_weight(current: str, loc: str, locName: list, locations: dict, visited: set) -> float: #init: {"GNSS1": 0.5, "GNSS2": 0.5, "AR1": 0.8, "AR2": 0.8, "AR3": 1,  "OBJ1": 0.8, "OBJ2": 1}
    #difficulty = {"GNSS1": 1, "GNSS2": 1, "AR1": 1, "AR2": 1, "AR3": 1,  "OBJ1": 1, "OBJ2": 1}

    distance = calculate_distance_cart(locations[current], locations[loc]) #/2 (for scaling actual locations)
    proximity_weight = sum(calculate_distance_cart(locations[loc], locations[other]) for other in locName if other != loc and other not in visited) #/8.5 (worst case scenario - assuming all the points are on the opposite end)(open for change, for scaling actual locations)
    
    # Use the correct key for difficulty
    
    return (distance + proximity_weight) * difficulty[loc] #instead of loc_type

# Function to find the shortest path using a weighted nearest-neighbor heuristic
def find_shortest_path(start: str, locations: dict):
    visited = set()
    current = start
    visited.add(current)
    path = [current]
    total_distance = 0
    
    #initialize locName list
    locName = list(locations.keys())
    remaining = locations.copy()

    # Checking to see if rover traveled to all locations
    while len(visited) < len(locName):
        nearest = None
        min_weight = float('inf')

        for loc in locName:
            if loc not in visited:
                # Calculate the weight for traveling to this location
                weight = calculate_location_weight(current, loc, locName, locations, visited)
                if weight < min_weight:
                    nearest = loc
                    min_weight = weight

        if nearest != None:
            print(f"Weight for {nearest} is {min_weight}")
            visited.add(nearest)
            path.append(nearest)
            remaining.pop(nearest)
            total_distance += calculate_distance_cart(locations[current], locations[nearest])
            current = nearest
            
    

    return path, total_distance

#graphing the locations
def plot_locations(path, locations):
    fig, ax = plt.subplots()
    
    for loc in locations:
        ax.plot(locations[loc][1], locations[loc][0], 'o', label=loc)
        ax.text(locations[loc][1], locations[loc][0], loc)

    for i in range(len(path) - 1):
        start_loc = locations[path[i]]
        end_loc = locations[path[i + 1]]
        ax.plot([start_loc[1], end_loc[1]], [start_loc[0], end_loc[0]], 'k-')

    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.title('Shortest Path Visiting All Locations')
    plt.legend()
    plt.show()

def OPmain(start, locations):
    # Start the pathfinding from a specific location of choice based on the dictionary
    
    
    path, total_distance = find_shortest_path(start, locations)
    # print(path)
    
    #print("Shortest path visiting all locations:")
    #print(" -> ".join(path))
    #print(f"Total distance: {total_distance:.2f} km")
    plot_locations(path, locations)
    
    path.remove('start')

    return path

#OPmain()

if __name__ == "__main__":
    locations = {
    "start": (43.66124, -79.39581), # Toronto
    "GNSS1": (43.66156, -79.3958),   # Statue of Liberty
    "GNSS2": (43.66131, -79.39503),     # Eiffel Tower
    "AR1": (43.66181, -79.39459),    # Sydney Opera House
    "AR2": (43.66135, -79.39437),     # Great Wall of China (Mutianyu)
    "AR3": (43.66193, -79.39594),    # Christ the Redeemer
    "OBJ1": (43.66227, -79.39504),     # Taj Mahal
    "OBJ2": (43.66182, -79.39531)    # Colosseum
    }
    OPmain("start", locations)