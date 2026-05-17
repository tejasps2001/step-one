package report;

import core.DTNHost;
import core.SimClock;
import core.UpdateListener;
import movement.MovementModel;
import movement.WOAFogMovement;
import movement.MILPClusteredFogMovement;
import movement.GDRRTMovement;
import movement.ExtendedGDRRTMovement;

import java.util.List;
import java.util.ArrayList;
import java.util.Map;
import java.util.HashMap;

/**
 * Monitors and reports performance metrics for Fog-UAV-assisted swarms.
 * Tracks coverage, priority-awareness, resilience, and computational efficiency.
 */
public class FogPerformanceReport extends Report implements UpdateListener {

    private DTNHost fogUAV = null;       // The central Fog coordinator
    private double commRange = 500.0;    // Maximum communication distance
    
    // Total Swarm Coverage Metric
    private int totalUpdates = 0;
    private double totalSwarmCoverageSum = 0;
    private Map<DTNHost, Double> totalDisconnectionTime = new HashMap<>();
    
    // Starvation Metric
    private double totalWeightedCoverageScore = 0;
    private double maxHighPriorityStarvationTime = 0;
    private Map<DTNHost, Double> currentStarvationTime = new HashMap<>();
    
    // Recovery Metrics
    private boolean shutdownLogged = false;
    private double timeToRecovery = -1;
    private boolean recoveryFound = false;
    
    // Distance Metrics
    private double totalDistanceTraveled = 0;
    private core.Coord lastFogLoc = null;
    private core.Coord lastOptimalTarget = null;
    private int targetChurnCount = 0; // how often Fog moves target
    
    // Computational performance Metrics
    private double totalExecutionTimeMs = 0;
    private int executionCount = 0;
    
    private double lastUpdate = 0;

    public FogPerformanceReport() {
        super();
    }

    @Override
    public void updated(List<DTNHost> hosts) {
        // Identify the Fog UAV host if not yet assigned
        if (fogUAV == null) {
            for (DTNHost h : hosts) {
                if (h.getMovement() instanceof WOAFogMovement || h.getMovement() instanceof MILPClusteredFogMovement) {
                    fogUAV = h;
                    lastFogLoc = fogUAV.getLocation().clone();
                    break;
                }
            }
            if (fogUAV == null) return; // Still not found
        }
        
        // Calculate time elapsed since last update
        double currentTime = SimClock.getTime();
        double updateInterval = currentTime - lastUpdate;
        if (updateInterval <= 0) updateInterval = 1.0; // fallback if first update
        lastUpdate = currentTime;
        
        totalUpdates++;
        
        List<DTNHost> activeDrones = new ArrayList<>();
        double highestPriority = -1;
        DTNHost highestPriorityDrone = null;

        // Identify active GDRRT drones and the highest priority target
        for (DTNHost h : hosts) {
            MovementModel mm = h.getMovement();
            if (mm instanceof GDRRTMovement || mm instanceof ExtendedGDRRTMovement) {
                boolean isDone = false;
                boolean isDead = false;
                double priority = 0.0;
                
                if (mm instanceof GDRRTMovement) {
                    isDone = ((GDRRTMovement) mm).isDone();
                    isDead = ((GDRRTMovement) mm).isDead();
                    priority = ((GDRRTMovement) mm).getPriority();
                } else {
                    isDone = ((ExtendedGDRRTMovement) mm).isDone();
                    isDead = ((ExtendedGDRRTMovement) mm).isDead();
                    priority = ((ExtendedGDRRTMovement) mm).getPriority();
                }
                
                if (!isDone && !isDead) {
                    activeDrones.add(h);
                    if (priority > highestPriority) {
                        highestPriority = priority;
                        highestPriorityDrone = h;
                    }
                }
            }
        }
        
        if (activeDrones.isEmpty()) return;

        int coveredCount = 0;
        double currentWeightedScore = 0;

        // Check connectivity and calculate coverage scores
        for (DTNHost drone : activeDrones) {
            double dist = drone.getLocation().distance(fogUAV.getLocation());
            if (dist <= commRange) {
                coveredCount++;
                
                double priority = 0.0;
                if (drone.getMovement() instanceof GDRRTMovement) {
                    priority = ((GDRRTMovement)drone.getMovement()).getPriority();
                } else if (drone.getMovement() instanceof ExtendedGDRRTMovement) {
                    priority = ((ExtendedGDRRTMovement)drone.getMovement()).getPriority();
                }
                
                currentWeightedScore += priority;
                currentStarvationTime.put(drone, 0.0); // reset starvation
            } else {
                totalDisconnectionTime.put(drone, totalDisconnectionTime.getOrDefault(drone, 0.0) + updateInterval);
                currentStarvationTime.put(drone, currentStarvationTime.getOrDefault(drone, 0.0) + updateInterval);
            }
        }
        
        totalSwarmCoverageSum += (double) coveredCount / activeDrones.size();
        totalWeightedCoverageScore += currentWeightedScore * updateInterval;
        
        if (highestPriorityDrone != null) {
            double starvation = currentStarvationTime.getOrDefault(highestPriorityDrone, 0.0);
            if (starvation > maxHighPriorityStarvationTime) {
                maxHighPriorityStarvationTime = starvation;
            }
        }
        
        double distMoved = fogUAV.getLocation().distance(lastFogLoc);
        totalDistanceTraveled += distMoved;
        lastFogLoc = fogUAV.getLocation().clone();
        
        // Collect state from the Fog UAV
        MovementModel fogMm = fogUAV.getMovement();
        core.Coord currentTarget = null;
        double execTime = 0;
        boolean shutdownTriggered = false;
        boolean reassignmentSuccess = false;
        double shutdownTime = -1;

        // Different ways to collect information based on algorithm
        if (fogMm instanceof WOAFogMovement) {
            WOAFogMovement woa = (WOAFogMovement) fogMm;
            currentTarget = woa.getCurrentOptimalTarget();
            execTime = woa.pollExecutionTimeMs();
            shutdownTriggered = woa.isShutdownTriggered();
            reassignmentSuccess = woa.isReassignmentSuccessful();
            shutdownTime = woa.getShutdownTime();
        } else if (fogMm instanceof MILPClusteredFogMovement) {
            MILPClusteredFogMovement milp = (MILPClusteredFogMovement) fogMm;
            currentTarget = milp.getCurrentOptimalTarget();
            execTime = milp.pollExecutionTimeMs();
            shutdownTriggered = milp.isShutdownTriggered();
            reassignmentSuccess = milp.isReassignmentSuccessful();
            shutdownTime = milp.getShutdownTime();
        }
        
        // Track how often the target point shifts significantly
        if (currentTarget != null && lastOptimalTarget != null) {
            if (currentTarget.distance(lastOptimalTarget) > 10.0) {
                targetChurnCount++;
            }
        }
        if (currentTarget != null) {
            lastOptimalTarget = currentTarget.clone();
        }
        
        // Record computational overhead
        if (execTime > 0) {
            totalExecutionTimeMs += execTime;
            executionCount++;
        }
        
        // Log shutdown events
        if (shutdownTriggered && !shutdownLogged) {
            shutdownLogged = true;
            String status = reassignmentSuccess ? "SUCCESS" : "FAILED";
            write("--- SHUTDOWN EVENT DETECTED ---");
            write("Reassignment Status: " + status);
        }

        // Measure time to reach recovery target
        if (shutdownLogged && reassignmentSuccess && !recoveryFound) {
            if (currentTarget != null && fogUAV.getLocation().distance(currentTarget) <= 10.0) {
                timeToRecovery = SimClock.getTime() - shutdownTime;
                recoveryFound = true;
                write("Fog UAV recovered optimally at t=" + SimClock.getTime() + ". Time-to-Recovery: " + timeToRecovery + "s");
            }
        }
    }

    @Override
    public void done() {
        write("=================================================");
        write("          FOG PERFORMANCE REPORT                 ");
        write("=================================================");
        
        write("Category 1: Coverage and Connectivity Metrics");
        write("Average Swarm Coverage: " + format(totalSwarmCoverageSum / Math.max(1, totalUpdates) * 100) + "%");
        double avgDisconn = 0;
        for (Double d : totalDisconnectionTime.values()) avgDisconn += d;
        if (!totalDisconnectionTime.isEmpty()) avgDisconn /= totalDisconnectionTime.size();
        write("Average Drone Disconnection Time: " + format(avgDisconn) + "s");
        write("");
        
        write("Category 2: Priority-Awareness Metrics");
        write("Total Weighted Coverage Score: " + format(totalWeightedCoverageScore));
        write("Max High-Priority Starvation Time: " + format(maxHighPriorityStarvationTime) + "s");
        write("");
        
        write("Category 3: Resilience and Event Responsiveness");
        if (shutdownLogged) {
            write("Reassignment Attempted: YES");
            if (recoveryFound) {
                write("Time-to-Recovery: " + format(timeToRecovery) + "s");
            } else {
                write("Time-to-Recovery: N/A (Did not fully recover)");
            }
        } else {
            write("Reassignment Attempted: NO (No shutdown event)");
        }
        write("");
        
        write("Category 4: Movement Efficiency (Kinematic Metrics)");
        write("Total Distance Traveled: " + format(totalDistanceTraveled) + "m");
        write("Path Stability / Target Churn Count: " + targetChurnCount);
        write("");
        
        write("Category 5: Computational Overhead");
        double avgExec = executionCount > 0 ? totalExecutionTimeMs / executionCount : 0;
        write("Average Algorithm Execution Time: " + format(avgExec) + " ms per cycle");
        write("=================================================");
        super.done();
    }
}