import sys
from shapely.wkt import loads

obstacleFile = sys.argv[1]
with open(obstacleFile, 'r') as f:
    obstacleInfo = f.read()

obstacles = loads(obstacleInfo)
path_segment = loads(sys.argv[2])
obstacleBoundaryCoords = obstacles.buffer(10).boundary.coords
BoundaryCoordsList = list(obstacleBoundaryCoords)
print(BoundaryCoordsList)

# Buffer obstacles by 0.5 units and check for intersection
if obstacles.buffer(0.5).boundary.intersects(path_segment):
    sys.exit(0) # Collision detected (exit code 0)
sys.exit(1)