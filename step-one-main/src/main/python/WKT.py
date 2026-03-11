import sys
import os
from shapely.wkt import loads

def render_boundaries(obstacles):
    for obs in obstacles:
        obstacleBoundaryCoords = obs.buffer(10).boundary.coords
        BoundaryCoordsList = list(obstacleBoundaryCoords)
        print(BoundaryCoordsList)
        print("---")

def check_collision(obstacles, path_segment):
    for obs in obstacles:
        if obs.buffer(0.5).boundary.intersects(path_segment):
            return True  # Collision detected
    return False  # No collision

# ---- Main ----
obstacleFile = sys.argv[1]
path_segment = loads(sys.argv[2])

obstacles = []

with open(obstacleFile, 'r') as f:
    for line in f:
        line = line.strip()
        if line:
            obstacles.append(loads(line))




file_path = obstacleFile.replace( "obstacles.wkt", "droneRoute.wkt")

if os.path.exists(file_path):
    with open(file_path, "r") as f:
      for line in f:
        line = line.strip()
        if line:
            obstacles.append(loads(line))

render_boundaries(obstacles)

if check_collision(obstacles, path_segment):
    sys.exit(0)  # Collision detected

sys.exit(1)  # No collision