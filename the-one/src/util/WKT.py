import sys
from shapely.wkt import loads

# -----------------------------
# Arguments from Java
# -----------------------------
obstacle_file = sys.argv[1]
edge_wkt = sys.argv[2]

BUFFER_DISTANCE = 10

try:
    # Load edge (new connection)
    edge = loads(edge_wkt)

    # Open obstacle file
    with open(obstacle_file, "r") as f:
        for line in f:
            line = line.strip()

            if not line:
                continue   # skip empty lines

            # Convert each LINESTRING to geometry
            obstacle = loads(line)

            # Add safety buffer
            buffered = obstacle.buffer(BUFFER_DISTANCE)
            print("Checking the collision between",edge,obstacle)
            # Collision check
            if edge.intersects(buffered):
                sys.exit(0)   # collision detected

    # If loop completes → no collision
    sys.exit(1)

except Exception as e:
    print("Python error:", e)
    sys.exit(2)   # crash indicator