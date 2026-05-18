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
 * complexity using k-means clustering method. It evaluates the clusters based on drone
 * priority weights and positions the Fog UAV at the weighted centroid of the 
 * highest-priority cluster to maximize connectivity time with the most critical targets.
 * 
 * Navigation to the optimal position is handled via the GDRRT algorithm.
 */
public class MILPClusteredFogMovement extends MovementModel {
    public static final String MILP_FOG_NS = "MILPClusteredFogMovement.";
    public static final String START_LOCATION_S = "startLocation";
    public static final String OBSTACLE_FILE_S = "obstacleFile";
    public static final String UPDATE_INTERVAL_S = "updateInterval"; // Interval to recalculate clustering (in seconds)
    public static final String NUM_CLUSTERS_S = "numClusters"; // Value 'k' for K-Means clustering
    public static final String ENABLE_SHUTDOWN_S = "enableRandomShutdown";
    public static final String SHUTDOWN_TIME_S = "shutdownTime";

    private Coord startLoc;
    private Coord currentOptimalTarget;
    private String obstacleFilePath;
    
    private double updateInterval;
    private double lastUpdateTime = -1;
    private int numClusters;

    private GDRRTPlanner gdrrt;
    private boolean isWaiting = false;
    private boolean initialPrioritiesPrinted = false;
    
    private boolean enableShutdown = false;
    private double shutdownTime = -1.0;
    private boolean shutdownTriggered = false;
    private boolean reassignmentSuccessful = false;
    private double actualShutdownTime = -1.0;
    private double lastExecutionTimeMs = 0.0;

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
        
        this.gdrrt = new GDRRTPlanner(this.obstacleFilePath);
        
        if (s.contains(MILP_FOG_NS + ENABLE_SHUTDOWN_S)) {
            this.enableShutdown = s.getBoolean(MILP_FOG_NS + ENABLE_SHUTDOWN_S);
        }
        if (s.contains(MILP_FOG_NS + SHUTDOWN_TIME_S)) {
            this.shutdownTime = s.getDouble(MILP_FOG_NS + SHUTDOWN_TIME_S);
        }
    }

    public MILPClusteredFogMovement(MILPClusteredFogMovement proto) {
        super(proto);
        this.startLoc = proto.startLoc.clone();
        this.obstacleFilePath = proto.obstacleFilePath;
        this.updateInterval = proto.updateInterval;
        this.numClusters = proto.numClusters;
        this.enableShutdown = proto.enableShutdown;
        this.shutdownTime = proto.shutdownTime;
        this.shutdownTriggered = proto.shutdownTriggered;
        this.reassignmentSuccessful = proto.reassignmentSuccessful;
        this.actualShutdownTime = proto.actualShutdownTime;
        this.lastExecutionTimeMs = proto.lastExecutionTimeMs;
        
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
            return SimClock.getTime() + 1.0;
        }
        return 0;
    }

    public Coord getCurrentOptimalTarget() { return currentOptimalTarget; }
    public boolean isShutdownTriggered() { return shutdownTriggered; }
    public boolean isReassignmentSuccessful() { return reassignmentSuccessful; }
    public double getShutdownTime() { return actualShutdownTime; }
    
    public double pollExecutionTimeMs() { 
        double val = lastExecutionTimeMs;
        lastExecutionTimeMs = 0;
        return val;
    }

    @Override
    public Path getPath() {
        double currentTime = SimClock.getTime();
        
        // Random Drone Shutdown Event execution
        if (enableShutdown && !shutdownTriggered && currentTime >= shutdownTime) {
            triggerRandomShutdown();
        }

        // Recalculate optimal position if the interval has elapsed
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
            return null; // Nowhere to go therefore, remain stationary
        }

        // Stop planning and remain stationary if we are already at the target
        if (getHost() != null && getHost().getLocation().distance(currentOptimalTarget) < 1.0) {
            ExtendedDronePathManager.setStationary(getHost().getAddress());
            return null;
        }

        if (!gdrrt.isInitialized()) {
            gdrrt.init(getHost().getLocation(), currentOptimalTarget);
        }

        // Plan path segment using GDRRT Planner
        GDRRTPlanner.PlannedSegment proposedSegment = gdrrt.planNextSegment();

        if (proposedSegment == null) {
            ExtendedDronePathManager.setStationary(getHost().getAddress());
            gdrrt.init(getHost().getLocation(), currentOptimalTarget);
            return null;
        }

        // Collision Avoidance & Path Commitment
        if (ExtendedDronePathManager.requestPath(getHost().getAddress(), proposedSegment.path)) {
            isWaiting = false;
            gdrrt.commit(proposedSegment);
            return proposedSegment.path;
        } else {
            // Wait if a collision is detected by the Path Manager
            isWaiting = true;
            ExtendedDronePathManager.setStationary(getHost().getAddress());
            
            // Clear the tree so it doesn't grow infinitely if it is stuck
            gdrrt.init(getHost().getLocation(), currentOptimalTarget);

            Path waitingPath = new Path(0);
            waitingPath.addWaypoint(getHost().getLocation());
            return waitingPath;
        }
    }

    /**
     * Randomly shuts down an active drone and attempts to reassign its target
     * to a lower-priority drone that is currently within the Fog UAV's range.
     */
    private void triggerRandomShutdown() {
        System.out.println(">>> CRITICAL EVENT: Random Drone Shutdown Triggered at time " + SimClock.getTime() + " <<<");
        shutdownTriggered = true;
        actualShutdownTime = SimClock.getTime();
        List<DTNHost> activeDrones = new ArrayList<>();
        
        for (DTNHost host : core.SimScenario.getInstance().getHosts()) {
            if (host.getMovement() instanceof ExtendedGDRRTMovement) {
                ExtendedGDRRTMovement mm = (ExtendedGDRRTMovement) host.getMovement();
                if (!mm.isDone() && !mm.isDead()) activeDrones.add(host);
            }
        }

        if (activeDrones.isEmpty()) return;

        // Pick a random active drone to shut down
        java.util.Collections.shuffle(activeDrones, new java.util.Random((long)SimClock.getTime()));
        DTNHost killedDrone = activeDrones.get(0);
        ExtendedGDRRTMovement killedMm = (ExtendedGDRRTMovement) killedDrone.getMovement();
        
        double killedPriority = killedMm.getPriority();
        Coord killedTarget = killedMm.getEndLocation();

        System.out.println(">>> CRITICAL EVENT: Drone " + killedDrone.getName() + " HAS FAILED! <<<");
        killedMm.kill();

        // Scan for a lower priority replacement drone in range
        DTNHost replacement = null;
        double bestPriority = Integer.MAX_VALUE;
        double fogWifiRange = 500.0; // Matched with wifiInterface.transmitRange from settings file

        for (DTNHost host : activeDrones) {
            if (host == killedDrone) continue;
            ExtendedGDRRTMovement mm = (ExtendedGDRRTMovement) host.getMovement();
            double priority = mm.getPriority();
            
            // The below if condition is triggered only if the drone is not done, not dead, has a 
            // lower priority than the killed drone and is in Wi-Fi range of the Fog UAV
            if (!mm.isDone() && !mm.isDead() && priority < killedPriority) {
                if (host.getLocation().distance(getHost().getLocation()) <= fogWifiRange) {
                    // Pick the lowest priority out of the available lower-priority candidates
                    if (priority < bestPriority) {
                        bestPriority = priority;
                        replacement = host;
                    }
                }
            }
        }

        // Reassign if a suitable drone was found
        if (replacement != null) {
            System.out.println(">>> FOG UAV ACTION: Reassigning higher priority target " + killedTarget + 
                               " to lower priority in-range Drone " + replacement.getName() + " <<<");
            ((ExtendedGDRRTMovement) replacement.getMovement()).changeTarget(killedTarget, killedPriority);
            reassignmentSuccessful = true;
            System.out.println(replacement.getPath());
        } else {
            reassignmentSuccessful = false;
            System.out.println(">>> FOG UAV ACTION: Drone " + killedDrone.getName() + " failed, but NO lower priority drone is in range for reassignment. <<<");
        }
    }

    /**
     * Extracts drones, groups them into 'K' clusters, and returns the weighted 
     * centroid of the highest-priority cluster.
     */
    private Coord calculateOptimalPosition() {
        long startTime = System.nanoTime(); // For peformance profiling
        DTNHost fogHost = getHost();
        if (fogHost == null) {
            lastExecutionTimeMs = (System.nanoTime() - startTime) / 1_000_000.0;
            return null; // Stay still if no host exists
        }

        List<DroneData> droneDataList = new ArrayList<>();
        
        if (!initialPrioritiesPrinted) {
            System.out.println("--- Drone Priorities and Target Assignments ---");

            // Build a mapping of target locations to their names
            Map<Coord, String> targetLocations = new java.util.HashMap<>();
            for (DTNHost host : core.SimScenario.getInstance().getHosts()) {
                if (host.getMovement() instanceof StationaryMovement) {
                    targetLocations.put(host.getLocation(), host.getName());
                }
            }

            // Collect all drones
            List<DTNHost> drones = new ArrayList<>();
            for (DTNHost host : core.SimScenario.getInstance().getHosts()) {
                if (host.getMovement() instanceof GDRRTMovement || host.getMovement() instanceof DroneMovement) { 
                    drones.add(host);
                }
            }
            // Sort by address for ordered output eg: D0, D1, and so on
            drones.sort(Comparator.comparingInt(DTNHost::getAddress));

            // Iterate through sorted drones, find their target, and print info
            for (DTNHost drone : drones) {
                ExtendedGDRRTMovement gdrrtMm = (ExtendedGDRRTMovement) drone.getMovement();
                Coord endLoc = gdrrtMm.getEndLocation();
                double priority = gdrrtMm.getPriority();
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
                    // If no target is reasonably close, there is some logical error. Therefore, 
                    // print "unmatched".
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
        for (DTNHost host : core.SimScenario.getInstance().getHosts()) {
            if (host == fogHost) continue; // Skip itself
            
            MovementModel mm = host.getMovement();
            boolean isDrone = false;
            boolean hasReached = false;
            double priority = 1.0;
            
            if (mm instanceof ExtendedGDRRTMovement || mm instanceof DroneMovement) {
                isDrone = true;
                hasReached = ((ExtendedGDRRTMovement) mm).isDone();
                priority = ((ExtendedGDRRTMovement) mm).getPriority();
            } else if (mm instanceof DroneMovement) {
                isDrone = true;
                hasReached = ((DroneMovement) mm).hasReachedTarget();
                priority = 1.0 + host.getAddress(); // Fallback for DroneMovement
            }

            if (isDrone && !hasReached) {
                Coord loc = host.getLocation();
                // Drones with higher priorities exert more gravity on the Fog UAV.
                droneDataList.add(new DroneData(host.getName(), loc, priority));
            }
        }

        if (droneDataList.isEmpty()) {
            lastExecutionTimeMs = (System.nanoTime() - startTime) / 1_000_000.0;
            return getHost().getLocation(); // No connected drones found
        }

        if (droneDataList.size() == 1) {
            System.out.println("Fog UAV is currently prioritising drone: " + droneDataList.get(0).name + " (Only one active drone)");
            lastExecutionTimeMs = (System.nanoTime() - startTime) / 1_000_000.0;
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
                    clusterScore += dd.priority;
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
            // But since we have dynamic moving drones with different priorities, there
            // is a probability that trying to converge will form a ping-pong infinite loop.
            // Therefore, we will just run a fixed number of iterations to get a good enough clustering.
        }

        if (bestCluster != null && !bestCluster.isEmpty()) {
            StringBuilder sb = new StringBuilder("Fog UAV is currently prioritising drones: ");
            for (int i = 0; i < bestCluster.size(); i++) {
                sb.append(bestCluster.get(i).name);
                if (i < bestCluster.size() - 1) sb.append(", ");
            }
            sb.append(" (Cluster Priority Score: ").append(finalBestScore).append(")");
        }

        lastExecutionTimeMs = (System.nanoTime() - startTime) / 1_000_000.0;
        return bestTarget != null ? bestTarget : getHost().getLocation();
    }
}