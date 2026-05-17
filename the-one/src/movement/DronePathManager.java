package movement;

import java.util.HashMap;
import java.util.Map;
import java.util.List;
import java.util.ArrayList;
import core.Coord;
import java.awt.geom.Line2D;
import core.DTNSim;

/**
 * Manages active drone paths to detect and prevent collisions.
 * This is a static singleton class.
 */
public class DronePathManager {
    // Maps drone ID to its currently approved and active path segment
    protected static final Map<Integer, Path> activePaths = new HashMap<>();
    
    // Creates a buffer around drone paths. If two paths come within this distance, it's a collision.
    protected static final double DRONE_BUFFER = 5.0; 
    
    static {
        DTNSim.registerForReset(DronePathManager.class.getCanonicalName());
    }

    public static void reset() {
        activePaths.clear();
    }

    /**
     * A drone requests permission to take a path.
     * If the path is clear of collisions with other active paths,
     * the request is approved, the path is registered, and the method returns true.
     * If a collision is detected, the request is denied and the method returns false.
     * @param droneId The ID of the drone requesting the path.
     * @param requestedPath The proposed path segment.
     * @return True if the path is approved, false otherwise.
     */
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

    /**
     * Informs the manager that a drone is now stationary and its path should be cleared.
     * @param droneId The ID of the drone that is now waiting.
     * @param location The exact physical coordinate where the drone is hovering.
     */
    public static synchronized void setStationary(int droneId, Coord location) {
        Path stationaryPath = new Path(0);
        stationaryPath.addWaypoint(location.clone());
        activePaths.put(droneId, stationaryPath);
    }

    /**
     * Checks a proposed path for collisions against all other active paths.
     * @param requestedPath The path to check.
     * @return True if a collision is detected, false otherwise.
     */
    private static boolean isCollision(Path requestedPath) {
        if (requestedPath == null || requestedPath.getCoords().isEmpty()) {
            return false;
        }
        List<Line2D.Double> reqSegments = createSegmentsFromPath(requestedPath);

        for (Path otherPath : activePaths.values()) {
            if (otherPath == null || otherPath.getCoords().isEmpty()) {
                continue;
            }
            List<Line2D.Double> otherSegments = createSegmentsFromPath(otherPath);

            for (Line2D.Double reqSeg : reqSegments) {
                for (Line2D.Double otherSeg : otherSegments) {
                    if (getSegmentToSegmentDistance(reqSeg, otherSeg) < DRONE_BUFFER) {
                        return true;
                    }
                }
            }
        }
        return false;
    }

    protected static double getSegmentToSegmentDistance(Line2D.Double l1, Line2D.Double l2) {
        if (l1.intersectsLine(l2)) return 0.0;
        
        double d1 = l1.ptSegDist(l2.getP1());
        double d2 = l1.ptSegDist(l2.getP2());
        double d3 = l2.ptSegDist(l1.getP1());
        double d4 = l2.ptSegDist(l1.getP2());
        
        return Math.min(Math.min(d1, d2), Math.min(d3, d4));
    }

    private static List<Line2D.Double> createSegmentsFromPath(Path path) {
        List<Line2D.Double> segments = new ArrayList<>();
        List<Coord> coords = path.getCoords();
        if (coords.size() == 1) {
            Coord c = coords.get(0);
            segments.add(new Line2D.Double(c.getX(), c.getY(), c.getX(), c.getY()));
        } else {
            for (int i = 0; i < coords.size() - 1; i++) {
                Coord c1 = coords.get(i);
                Coord c2 = coords.get(i + 1);
                segments.add(new Line2D.Double(c1.getX(), c1.getY(), c2.getX(), c2.getY()));
            }
        }
        return segments;
    }
}