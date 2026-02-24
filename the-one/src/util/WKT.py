import sys
from shapely.wkt import loads
from shapely.geometry import LineString

# line = loads(sys.argv[1])
obstacleInfo = loads(sys.argv[1])
xnearrand = loads(sys.argv[2])
obstacleOutline = []
i = 0
for obstacle in obstacleInfo:
    obstacleOutline[i] = obstacle.buffer(0.5).boundary
    i += 1

for obstacle in obstacleOutline:
    if(xnearrand.intersects(obstacle)):
        sys.exit(0)
sys.exit(1)