package report;

import core.Coord;
import core.DTNHost;
import core.UpdateListener;
import movement.DroneMovement;
import movement.GDRRTMovement;
import movement.UAVWaypointMovement;

import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

public class UavPathPlanningReport extends Report implements UpdateListener {

    public static final String COLLISION_THRESHOLD_S = "collisionThreshold";
    private static final double DEFAULT_COLLISION_THRESHOLD = 20.0;

    private double collisionThreshold;

    private Map<Integer, Double> startTimes = new HashMap<>();
    private Map<Integer, Double> finishTimes = new HashMap<>();
    private Map<Integer, Double> distanceTraveled = new HashMap<>();
    private Map<Integer, Coord> lastKnownLocation = new HashMap<>();
    private Map<Integer, Boolean> isFinished = new HashMap<>();

    private Set<String> activeCollisions = new HashSet<>();
    private int totalCollisions = 0;

    public UavPathPlanningReport() {
        super();
        if (getSettings().contains(COLLISION_THRESHOLD_S)) {
            this.collisionThreshold = getSettings().getDouble(COLLISION_THRESHOLD_S);
        } else {
            this.collisionThreshold = DEFAULT_COLLISION_THRESHOLD;
        }
    }

    @Override
    public void updated(List<DTNHost> hosts) {
        if (isWarmup()) return;

        double currentTime = getSimTime();

        // Track distances and completion
        for (DTNHost host : hosts) {
            int id = host.getAddress();
            movement.MovementModel mm = host.getMovement();
            
            boolean isTrackedDrone = (mm instanceof GDRRTMovement) || 
                                     (mm instanceof UAVWaypointMovement) || 
                                     (mm instanceof DroneMovement);

            if (!isTrackedDrone) continue;

            Coord currentLocation = host.getLocation();

            if (!startTimes.containsKey(id)) {
                startTimes.put(id, currentTime);
                distanceTraveled.put(id, 0.0);
                isFinished.put(id, false);
            }

            if (lastKnownLocation.containsKey(id) && !isFinished.get(id)) {
                double dist = currentLocation.distance(lastKnownLocation.get(id));
                distanceTraveled.put(id, distanceTraveled.get(id) + dist);
            }
            lastKnownLocation.put(id, currentLocation.clone());

            if (!isFinished.get(id)) {
                boolean done = false;
                if (mm instanceof GDRRTMovement) {
                    done = ((GDRRTMovement) mm).isDone() || ((GDRRTMovement) mm).isDead();
                } else if (mm instanceof UAVWaypointMovement) {
                    done = ((UAVWaypointMovement) mm).isReachedFinalGoal();
                } else if (mm instanceof DroneMovement) {
                    done = ((DroneMovement) mm).hasReachedTarget();
                }

                if (done) {
                    isFinished.put(id, true);
                    finishTimes.put(id, currentTime);
                }
            }
        }

        // Track collisions
        for (int i = 0; i < hosts.size(); i++) {
            DTNHost h1 = hosts.get(i);
            if (!startTimes.containsKey(h1.getAddress())) continue;

            for (int j = i + 1; j < hosts.size(); j++) {
                DTNHost h2 = hosts.get(j);
                if (!startTimes.containsKey(h2.getAddress())) continue;

                double dist = h1.getLocation().distance(h2.getLocation());
                String pairId = h1.getAddress() + "-" + h2.getAddress();

                if (dist < collisionThreshold) {
                    if (!activeCollisions.contains(pairId)) {
                        activeCollisions.add(pairId);
                        totalCollisions++;
                        
                        System.out.printf("[GUI/LOG] COLLISION at T=%.1fs between %s and %s! Dist: %.2fm | Pos1:(%.1f, %.1f) Pos2:(%.1f, %.1f)%n",
                                currentTime, h1.getName(), h2.getName(), dist,
                                h1.getLocation().getX(), h1.getLocation().getY(),
                                h2.getLocation().getX(), h2.getLocation().getY());
                    }
                } else {
                    activeCollisions.remove(pairId);
                }
            }
        }
    }

    @Override
    public void done() {
        write("=================================================");
        write("UAV Path Planning Report for " + getScenarioName());
        write("Collision Threshold: " + collisionThreshold + "m");
        write("=================================================");
        write("NodeID\tDistance(m)\tTravelTime(s)\tFinished");

        double totalDistance = 0.0;
        double totalTime = 0.0;
        int finishedCount = 0;
        int trackedCount = startTimes.size();

        for (Integer id : startTimes.keySet()) {
            double dist = distanceTraveled.getOrDefault(id, 0.0);
            boolean finished = isFinished.getOrDefault(id, false);
            double time = finished ? (finishTimes.get(id) - startTimes.get(id)) : (getSimTime() - startTimes.get(id));

            write(id + "\t" + format(dist) + "\t" + format(time) + "\t" + finished);

            if (finished) {
                totalDistance += dist;
                totalTime += time;
                finishedCount++;
            }
        }

        write("-------------------------------------------------");
        write("Total Tracked UAVs: " + trackedCount);
        write("Total UAVs Finished: " + finishedCount);
        if (finishedCount > 0) {
            write("Average Distance (Finished): " + format(totalDistance / finishedCount) + "m");
            write("Average Travel Time (Finished): " + format(totalTime / finishedCount) + "s");
        }
        write("Total Inter-UAV Collisions: " + totalCollisions);
        write("=================================================");

        super.done();
    }
}
