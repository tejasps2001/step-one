package movement;

import java.util.HashMap;
import java.util.Map;
import core.Coord;
import java.awt.geom.Line2D;
import core.DTNSim;

/**
 * Exact clone of DronePathManager, decoupled for MILP and WOA scenarios.
 */
public class ExtendedDronePathManager {
    // Maps drone ID to its currently approved and active path segment
    private static final Map<Integer, Path> activePaths = new HashMap<>();
    
    // Safety buffer around drone paths. If two paths come within this distance, it's a collision.
    private static final double DRONE_BUFFER = 5.0; 
    
    static {
        DTNSim.registerForReset(ExtendedDronePathManager.class.getCanonicalName());
    }

    public static void reset() {
        activePaths.clear();
    }

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

            // Check proximity buffer (intersection is handled inside the helper method).
            if (getSegmentToSegmentDistance(requesterSegment, otherSegment) < DRONE_BUFFER) {
                return true;
            }
        }
        return false;
    }

    private static double getSegmentToSegmentDistance(Line2D.Double l1, Line2D.Double l2) {
        if (l1.intersectsLine(l2)) return 0.0;
        double d1 = l1.ptSegDist(l2.getP1());  double d2 = l1.ptSegDist(l2.getP2());
        double d3 = l2.ptSegDist(l1.getP1());  double d4 = l2.ptSegDist(l1.getP2());
        return Math.min(Math.min(d1, d2), Math.min(d3, d4));
    }

    private static Line2D.Double createLineFromPath(Path path) {
        Coord start = path.getCoords().get(0);
        Coord end = path.getCoords().get(path.getCoords().size() - 1);
        return new Line2D.Double(start.getX(), start.getY(), end.getX(), end.getY());
    }
}