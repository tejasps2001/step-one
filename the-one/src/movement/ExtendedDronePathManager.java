package movement;

import core.Coord;
import java.awt.geom.Line2D;

/**
 * Manages active drone paths to detect and prevent collisions.
 * This is an extension of the original DronePathManager.
 * This is a static singleton class.
 */
public class ExtendedDronePathManager extends DronePathManager {

    public static synchronized boolean requestPath(int droneId, Path requestedPath) {
        // First, remove the drone's old path so it doesn't collide with itself.
        activePaths.remove(droneId);

        if (isCollision(requestedPath)) {
            // Collision detected. Do not add the new path.
            return false;
        }
        
        // Path is clear. Add it to the active paths.
        activePaths.put(droneId, requestedPath);
        return true;
    }

    public static synchronized void setStationary(int droneId) {
        activePaths.remove(droneId);
    }

    private static boolean isCollision(Path requestedPath) {
        if (requestedPath == null || requestedPath.getCoords().size() < 2) {
            return false; // A stationary or invalid path can't collide.
        }
        Line2D.Double requesterSegment = createLineFromPath(requestedPath);

        for (Path otherPath : activePaths.values()) {
            if (otherPath == null || otherPath.getCoords().size() < 2) {
                continue;
            }
            Line2D.Double otherSegment = createLineFromPath(otherPath);

            // Check proximity buffer (intersection is handled inside getSegmentToSegmentDistance method).
            if (getSegmentToSegmentDistance(requesterSegment, otherSegment) < DRONE_BUFFER) {
                return true;
            }
        }
        return false;
    }

    private static Line2D.Double createLineFromPath(Path path) {
        Coord start = path.getCoords().get(0);
        Coord end = path.getCoords().get(path.getCoords().size() - 1);
        return new Line2D.Double(start.getX(), start.getY(), end.getX(), end.getY());
    }
}