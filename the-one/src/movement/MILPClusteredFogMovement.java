package movement;

import core.Coord;
import core.Settings;
import core.SimClock;
import core.DTNHost;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Comparator;

/**
 * Movement model for a Fog UAV Base Station based on the MILP and Clustering approach 
 * presented in "Optimal Positioning of Unmanned Aerial Vehicle (UAV) Base Stations 
 * Using Mixed-Integer Linear Programming" (Premkumar & Van Scoy, 2025).
 * 
 * This model groups active drones into clusters to manage scalability and computational 
 * complexity. It evaluates the clusters based on drone priority weights and positions 
 * the Fog UAV at the weighted centroid of the highest-priority cluster to maximize 
 * connectivity time with the most critical targets.
 * 
 * Navigation to the optimal position is handled via the GDRRT algorithm.
 */
public class MILPClusteredFogMovement extends MovementModel {
    public static final String MILP_FOG_NS = "MILPClusteredFogMovement.";
    public static final String START_LOCATION_S = "startLocation";
    public static final String OBSTACLE_FILE_S = "obstacleFile";
    public static final String UPDATE_INTERVAL_S = "updateInterval"; // Interval to recalculate clustering (in seconds)
    public static final String NUM_CLUSTERS_S = "numClusters"; // Value 'k' for K-Means clustering

    private Coord startLoc;
    private Coord currentOptimalTarget;
    private String obstacleFilePath;
    
    private double updateInterval;
    private double lastUpdateTime = -1;
    private int numClusters;

    private GDRRTPlanner gdrrt;
    private int fvsId;
    private boolean isWaiting = false;
    private boolean initialPrioritiesPrinted = false;

    // Internal class to hold drone priority and location for clustering
    private class DroneData {
        String name;
        Coord loc;
        double priority;

        DroneData(String name, Coord loc, double priority) {
            this.name = name;
            this.loc = loc;
            this.priority = priority;
        }
    }

    public MILPClusteredFogMovement(Settings s) {
        super(s);
        
        int[] coords = s.getCsvInts(MILP_FOG_NS + START_LOCATION_S, 2);
        this.startLoc = new Coord(coords[0], coords[1]);
        this.obstacleFilePath = s.getSetting(MILP_FOG_NS + OBSTACLE_FILE_S);
        this.updateInterval = s.getDouble(MILP_FOG_NS + UPDATE_INTERVAL_S, 30.0);
        this.numClusters = s.getInt(MILP_FOG_NS + NUM_CLUSTERS_S, 3); // Default to 3 clusters
        this.fvsId = s.getInt(FogVehicleSystem.FOG_VEHICLE_SYSTEM_NR);
        
        this.gdrrt = new GDRRTPlanner(this.obstacleFilePath);
    }

    public MILPClusteredFogMovement(MILPClusteredFogMovement proto) {
        super(proto);
        this.startLoc = proto.startLoc.clone();
        this.obstacleFilePath = proto.obstacleFilePath;
        this.updateInterval = proto.updateInterval;
        this.numClusters = proto.numClusters;
        this.fvsId = proto.fvsId;
        
        this.gdrrt = new GDRRTPlanner(this.obstacleFilePath);
    }

    @Override
    public MovementModel replicate() {
        return new MILPClusteredFogMovement(this);
    }

    @Override
    public Coord getInitialLocation() {
        return startLoc.clone();
    }

    @Override
    public double nextPathAvailable() {
        if (isWaiting) {
            // Try again shortly to see if collision paths have cleared
            // Fog UAV to the ONE simulator: "The airspace is packed 
            // right now and hence I am stuck. Come back after sometime"
            return SimClock.getTime() + 1.0;
        }
        return 0;
    }

    @Override
    public Path getPath() {
        double currentTime = SimClock.getTime();

        // 1. Recalculate optimal position if the interval has elapsed
        if (lastUpdateTime < 0 || (currentTime - lastUpdateTime) >= updateInterval) {
            Coord newOptimalTarget = calculateOptimalPosition();
            
            // Re-initialize GDRRT towards the newly optimized destination if it changed
            if (newOptimalTarget != null && (currentOptimalTarget == null || !newOptimalTarget.equals(currentOptimalTarget))) {
                currentOptimalTarget = newOptimalTarget;
                gdrrt.init(getHost().getLocation(), currentOptimalTarget);
            }
            lastUpdateTime = currentTime;
        }

        if (currentOptimalTarget == null) {
            return null; // Nowhere to go, remain stationary
        }

        if (!gdrrt.isInitialized()) {
            gdrrt.init(getHost().getLocation(), currentOptimalTarget);
        }

        // 2. Plan path segment using GDRRT Planner
        GDRRTPlanner.PlannedSegment proposedSegment = gdrrt.planNextSegment();

        if (proposedSegment == null) {
            DronePathManager.setStationary(getHost().getAddress());
            return null;
        }

        // 3. Collision Avoidance & Path Commitment
        if (DronePathManager.requestPath(getHost().getAddress(), proposedSegment.path)) {
            isWaiting = false;
            gdrrt.commit(proposedSegment);

            if (proposedSegment.isFinalPath) {
                DronePathManager.setStationary(getHost().getAddress());
            }
            return proposedSegment.path;
        } else {
            // Wait if a collision is detected by the Path Manager
            isWaiting = true;
            DronePathManager.setStationary(getHost().getAddress());
            
            Path waitingPath = new Path(0);
            waitingPath.addWaypoint(getHost().getLocation());
            return waitingPath;
        }
    }

    /**
     * Extracts drones, groups them into K clusters, and returns the weighted 
     * centroid of the highest-priority cluster.
     */
    private Coord calculateOptimalPosition() {
        DTNHost fogHost = getHost();
        if (fogHost == null) {
            return null; // Stay put if no host exists
        }

        List<DroneData> droneDataList = new ArrayList<>();
        
        if (!initialPrioritiesPrinted) {
            System.out.println("--- Drone Priorities and Target Assignments ---");

            // 1. Build a map of target locations to their names
            Map<Coord, String> targetLocations = new java.util.HashMap<>();
            for (DTNHost host : core.SimScenario.getInstance().getHosts()) {
                if (host.getMovement() instanceof StationaryMovement) {
                    targetLocations.put(host.getLocation(), host.getName());
                }
            }

            // 2. Collect all drones
            List<DTNHost> drones = new ArrayList<>();
            for (DTNHost host : core.SimScenario.getInstance().getHosts()) {
                if (host.getMovement() instanceof GDRRTMovement) { // This covers FogDeployedGDRRTMovement
                    drones.add(host);
                }
            }
            // Sort by address for ordered output (D0, D1, ...)
            drones.sort(Comparator.comparingInt(DTNHost::getAddress));

            // 3. Iterate through sorted drones, find their target, and print info
            for (DTNHost drone : drones) {
                GDRRTMovement gdrrtMm = (GDRRTMovement) drone.getMovement();
                Coord endLoc = gdrrtMm.getEndLocation();
                double priority = 1.0 + drone.getAddress();
                String targetName = "unassigned";

                if (endLoc != null) {
                    // Find the closest target to the drone's end location
                    double min_dist = Double.MAX_VALUE;
                    for (Map.Entry<Coord, String> entry : targetLocations.entrySet()) {
                        double dist = entry.getKey().distance(endLoc);
                        if (dist < min_dist) {
                            min_dist = dist;
                            targetName = entry.getValue();
                        }
                    }
                    // If no target is reasonably close, something is wrong with the setup
                    if (min_dist > 1.0) {
                        targetName = "unmatched";
                    }
                }
                
                System.out.println("Drone: " + drone.getName() + " (ID: " + drone.getAddress() + 
                                   ", Priority: " + priority + ") -> Target: " + targetName);
            }

            System.out.println("---------------------------------------------");
            initialPrioritiesPrinted = true;
        }

        // Evaluate ALL active drones globally by checking all hosts in the scenario
        // This bypasses FogVehicleSystem which stays empty when using FogDeployedGDRRTMovement
        for (DTNHost host : core.SimScenario.getInstance().getHosts()) {
            if (host == fogHost) continue; // Skip itself
            
            MovementModel mm = host.getMovement();
            boolean isDrone = false;
            boolean hasReached = false;
            
            if (mm instanceof GDRRTMovement) { // Covers FogDeployedGDRRTMovement
                isDrone = true;
                hasReached = ((GDRRTMovement) mm).isDone();
            }

            if (isDrone && !hasReached) {
                System.out.println(host.getName() + " is active and considered for clustering.");
                Coord loc = host.getLocation();
                // Drones with higher priorities exert more gravity on the Fog UAV.
                double priority = 1.0 + host.getAddress();
                droneDataList.add(new DroneData(host.getName(), loc, priority));
            }
        }

        if (droneDataList.isEmpty()) {
            return getHost().getLocation(); // No connected drones found
        }

        if (droneDataList.size() == 1) {
            System.out.println("Fog UAV is currently prioritising drone: " + droneDataList.get(0).name + " (Only one active drone)");
            return droneDataList.get(0).loc.clone();
        }

        int k = Math.min(droneDataList.size(), numClusters);
        List<Coord> centroids = new ArrayList<>();
        
        // Initialize centroids with the first K drones
        for (int i = 0; i < k; i++) {
            centroids.add(droneDataList.get(i).loc.clone());
        }

        List<List<DroneData>> clusters = new ArrayList<>();
        for (int i = 0; i < k; i++) clusters.add(new ArrayList<>());

        // K-Means clustering based on Euclidean distance
        int maxIterations = 20;
        Coord bestTarget = null;
        List<DroneData> bestCluster = null;
        double finalBestScore = -1;
        
        for (int iter = 0; iter < maxIterations; iter++) {
            for (List<DroneData> cluster : clusters) cluster.clear();

            // Assignment step
            for (DroneData dd : droneDataList) {
                int bestK = 0;
                double bestDist = Double.MAX_VALUE;
                for (int i = 0; i < k; i++) {
                    double dist = dd.loc.distance(centroids.get(i));
                    if (dist < bestDist) {
                        bestDist = dist;
                        bestK = i;
                    }
                }
                clusters.get(bestK).add(dd);
            }

            // Update step & calculate the optimal cluster priority score inline
            double bestScore = -1;
            bestTarget = null;
            bestCluster = null;
            for (int i = 0; i < k; i++) {
                if (clusters.get(i).isEmpty()) continue;
                
                double sumX = 0, sumY = 0, clusterScore = 0;
                for (DroneData dd : clusters.get(i)) {
                    sumX += dd.loc.getX() * dd.priority;
                    sumY += dd.loc.getY() * dd.priority;
                    clusterScore += dd.priority; // Cluster's importance metric
                }
                centroids.set(i, new Coord(sumX / clusterScore, sumY / clusterScore));

                if (clusterScore > bestScore) {
                    bestScore = clusterScore;
                    bestTarget = centroids.get(i);
                    bestCluster = clusters.get(i);
                }
            }
            finalBestScore = bestScore;
            // In a strict K-Means we would check for convergence here.
            // We allow it to run for `maxIterations` for safety in dynamic moving targets.
        }

        if (bestCluster != null && !bestCluster.isEmpty()) {
            StringBuilder sb = new StringBuilder("Fog UAV is currently prioritising drones: ");
            for (int i = 0; i < bestCluster.size(); i++) {
                sb.append(bestCluster.get(i).name);
                if (i < bestCluster.size() - 1) sb.append(", ");
            }
            sb.append(" (Cluster Priority Score: ").append(finalBestScore).append(")");
            System.out.println(sb.toString());
        }

        return bestTarget != null ? bestTarget : getHost().getLocation();
    }
}