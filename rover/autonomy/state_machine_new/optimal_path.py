import math 
from collections import deque


#0-1 for scoring
#task : [task_scoring, task_difficulty, task_location]
variable_dict = {"GNSS1": [1, 1, state_machine.potential_locations] , }


def shortest_path(curr_loc: tuple, rem_loc: dict): 

    s_path = ["", float('inf')]
    
    #Get position of curr_loc tuple
    lat1 = curr_loc[0]
    lon1 = curr_loc[1]

    for loc in rem_loc:
        #Get position of loc
        lat2 = rem_loc[loc][0]
        lon2 = rem_loc[loc][1]
        print(lat1, lon1, lat2, lon2)
        # Convert latitude and longitude from degrees to radians
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

        # Haversine formula
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a)) 

        # Radius of Earth in kilometers
        r = 6371.0
        dist = r * c

        if dist < s_path[1]:
            s_path[0], s_path[1] = loc, dist

    return (s_path[0], rem_loc[s_path[0]])


# BFS from given source s
def bfs(adj, s):
  
    # Create a queue for BFS
    q = deque()
    
    # Initially mark all the vertices as not visited
    # When we push a vertex into the q, we mark it as 
    # visited
    visited = [False] * len(adj)

    # Mark the source node as visited and enqueue it
    visited[s] = True
    q.append(s)

    # Iterate over the queue
    while q:
      
        # Dequeue a vertex from queue and print it
        curr = q.popleft()
        print(curr, end=" ")

        # Get all adjacent vertices of the dequeued 
        # vertex. If an adjacent has not been visited, 
        # mark it visited and enqueue it
        for x in adj[curr]:
            if not visited[x]:
                visited[x] = True
                q.append(x)

# Function to add an edge to the graph
def add_edge(adj, u, v):
    adj[u].append(v)
    adj[v].append(u)

def calculate_weight():
    pass

def main():
  
    # Number of vertices in the graph
    V = 7

    # Adjacency list representation of the graph
    adj = [[] for _ in range(V)]

    # Add edges to the graph
    add_edge(adj, 0, 1)
    add_edge(adj, 0, 2)
    add_edge(adj, 1, 3)
    add_edge(adj, 1, 4)
    add_edge(adj, 2, 4)

    print(adj)
    # Perform BFS traversal starting from vertex 0
    print("BFS starting from 0: ")
    #bfs(adj, 0) 

main()