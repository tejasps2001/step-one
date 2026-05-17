package movement;

import core.Coord;
import core.DTNHost;
import core.Settings;
import core.SettingsError;
import core.SimClock;
import core.SimScenario;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;
import java.util.Random;

/**
 *  Movement model for a Fog UAV Base Station using the
 *  Whale Optimization Algorithm (WOA), based on Naouri et al. (2024),
 *  to find the better position to move 
 */
public class WOAFogMovement extends MovementModel {

   

    /** Namespace prefix for all WOAFogMovement settings. */
    public static final String WOA_FOG_NS         = "WOAFogMovement.";

   

    /** Per-node-group setting for the fog UAV's initial spawn coordinate
     *   Expects two comma-separated integers: {x,y}. */
    public static final String START_LOCATION_S   = "startLocation";

    /** Per-node-group setting for the obstacle map file used by GDRRT */
    public static final String OBSTACLE_FILE_S    = "obstacleFile";

   

    /** Seconds between successive WOA optimisation cycles . */
    public static final String UPDATE_INTERVAL_S  = "updateInterval";

    /** Communication and coverage radius R in metres . */
    public static final String COMM_RANGE_S       = "communicationRange";

    /** Number of candidate whale positions in the WOA population . */
    public static final String POPULATION_SIZE_S  = "populationSize";

    /** Maximum WOA iterations executed per optimisation cycle . */
    public static final String MAX_ITERATIONS_S   = "maxIterations";

    /** Spiral shape constant b used in the WOA bubble-net update . */
    public static final String SPIRAL_B_S         = "woaSpiralB";

    /** Minimum positional shift (metres) required before re-planning the GDRRT
     *  path to a newly chosen target . */
    public static final String TARGET_THRESHOLD_S = "targetMoveThreshold";



    /** Coverage weight alpha for the NCV2 term in H(X) . */
    public static final String COVERAGE_WEIGHT_S  = "coverageWeight";   // alpha

    /** Priority weight beta for the priority term in H(X) . */
    public static final String PRIORITY_WEIGHT_S  = "priorityWeight";   // beta

    /** Semicolon-separated list of per-drone priority values .
     *  Format: {@code [w0; w1; w2; ...]}. Missing entries default to 1.0. */
    public static final String DRONE_PRIORITIES_S = "dronePriorities";



    /** Enables the random drone shutdown event . */
    public static final String ENABLE_SHUTDOWN_S  = "enableRandomShutdown";

    /** Simulation time at which to trigger the random shutdown . */
    public static final String SHUTDOWN_TIME_S    = "shutdownTime";

   /** 
    * A default placeholder indicating that the initial network scan 
    * hasn't yet determined the base address for the drone group. 
    */
   
    private static final int DRONE_GROUP_ADDR_UNKNOWN = -1;

    

    private final Coord  startLoc;            // fog UAV spawn point
    private final String obstacleFilePath;    // obstacle map file for GDRRT
    private final double updateInterval;      // seconds between WOA cycles
    private final double communicationRange;  // coverage radius R (metres)
    private final int    populationSize;      // number of WOA whales
    private final int    maxIterations;       // WOA iterations per cycle
    private final double spiralB;             // spiral shape constant b
    private final double targetMoveThreshold; // min positional shift (m) to re-plan path
    private final double coverageWeight;      // alpha — NCV2 term weight (normalised)
    private final double priorityWeight;      // beta  — priority term weight (normalised)

    private final boolean enableShutdown;     // random drone shutdown toggle
    private final double  shutdownTime;       // configured trigger time

    /** Maps zero-based drone index to its configured priority value. */
    private final java.util.Map<Integer, Double> dronePriorityMap;

    /** Lowest absolute address among mission-drone hosts; set on first scan.
     *  Used to convert absolute addresses to readable D0, D1, D2... labels. */
    private int droneGroupBaseAddr = DRONE_GROUP_ADDR_UNKNOWN;

    
    private Coord   currentOptimalTarget  = null;  // last WOA-chosen position
    private double  lastUpdateTime        = -1.0;  // sim time of last WOA run
    private boolean isWaiting             = false; // collision-avoidance hold flag
    private int     woaCycleCount         = 0;     // total WOA cycles run (for logging)
    private boolean shutdownTriggered     = false; // event latch — fires only once
    private boolean reassignmentSuccessful = false; // outcome of last shutdown event
    private double  actualShutdownTime    = -1.0;  // real sim time the shutdown fired
    private double  lastExecutionTimeMs   = 0.0;   // wall-clock duration of last WOA run

    private final GDRRTPlanner gdrrt;       // obstacle-aware path planner
    private final Random       rng = new Random(); // WOA random source

    private double worldWidth  = -1;   // set once by initWorldBounds()
    private double worldHeight = -1;


    /**
     * Immutable snapshot of a mission drone's state as seen by the fog UAV
     * during a WOA cycle. All fields are captured at the moment of the scan
     * and are not updated mid-cycle.
     */
    private static class DroneInfo {
        /** Zero-based drone index derived from the group base address; used
         *  in log output for readability (e.g. D0, D1). */
        final int    droneIndex;
        /** Location of the drone at the time of the snapshot. */
        final Coord  location;
        /** Mission priority assigned to this drone. */
        final double priority;
        /** Distance from the fog UAV's current position to this drone
         *  at snapshot time (not from a candidate whale position). */
        final double distanceToFog;

        /**
         * Creates a DroneInfo snapshot.
         
         */
        DroneInfo(int droneIndex, Coord location, double priority,
                  double distToFog) {
            this.droneIndex    = droneIndex;
            this.location      = location;
            this.priority      = priority;
            this.distanceToFog = distToFog;
        }

        @Override
        public String toString() {
            return String.format("D%d@(%.0f,%.0f) prio=%.1f curDist=%.1f",
                    droneIndex, location.getX(), location.getY(),
                    priority, distanceToFog);
        }
    }


    /**
     * Represents a single WOA search agent (whale) as a 2-D candidate
     * position with an associated fitness value. Used internally by
     * runWOA only.
     */
    private static class Whale {
        double x;
        double y;
        double fitness;

        /**
         * Creates a whale at the given position with fitness initialised to
         * negative infinity (unevaluated).
         */
        Whale(double x, double y) {
            this.x       = x;
            this.y       = y;
            this.fitness = Double.NEGATIVE_INFINITY;
        }

        /**
         * Returns a deep copy of this whale, including its fitness value.
                  */
        Whale copy() {
            Whale w   = new Whale(this.x, this.y);
            w.fitness = this.fitness;
            return w;
        }

        /**
         * Converts this whale's position to a  Coord
         */
        Coord toCoord() { return new Coord(x, y); }

        @Override
        public String toString() {
            return String.format("pos=(%.1f,%.1f) fit=%.4f", x, y, fitness);
        }
    }

    

    /**
     * Creates a new WOAFogMovement prototype from a settings object.
     * All configuration fields are read and validated here. The coverage and
     * priority weights are normalised so that they always sum to exactly 1.0.
     */
    public WOAFogMovement(Settings s) {
        super(s);

        int[] coords = s.getCsvInts(WOA_FOG_NS + START_LOCATION_S, 2);
        this.startLoc = new Coord(coords[0], coords[1]);

        this.obstacleFilePath   = s.getSetting(WOA_FOG_NS + OBSTACLE_FILE_S);

        this.updateInterval     = s.getDouble(WOA_FOG_NS + UPDATE_INTERVAL_S,   15.0);
        this.communicationRange = s.getDouble(WOA_FOG_NS + COMM_RANGE_S,        500.0);
        this.populationSize     = s.getInt   (WOA_FOG_NS + POPULATION_SIZE_S,   20);
        this.maxIterations      = s.getInt   (WOA_FOG_NS + MAX_ITERATIONS_S,    100);
        this.spiralB            = s.getDouble(WOA_FOG_NS + SPIRAL_B_S,          1.0);
        this.targetMoveThreshold= s.getDouble(WOA_FOG_NS + TARGET_THRESHOLD_S,  10.0);

        // Read alpha / beta and normalise so they always sum to exactly 1.0
        double rawAlpha = s.getDouble(WOA_FOG_NS + COVERAGE_WEIGHT_S, 0.5);
        double rawBeta  = s.getDouble(WOA_FOG_NS + PRIORITY_WEIGHT_S, 0.5);
        double sum      = rawAlpha + rawBeta;
        if (sum <= 0) {
            throw new SettingsError("coverageWeight + priorityWeight must be > 0");
        }
        this.coverageWeight = rawAlpha / sum;
        this.priorityWeight = rawBeta  / sum;

        this.dronePriorityMap = parseDronePriorities(s);

        this.enableShutdown = s.getBoolean(WOA_FOG_NS + ENABLE_SHUTDOWN_S, false);
        this.shutdownTime   = s.getDouble (WOA_FOG_NS + SHUTDOWN_TIME_S,   -1.0);

        this.gdrrt = new GDRRTPlanner(obstacleFilePath);
    }

    /**
     * Copy constructor. Creates a new WOAFogMovement instance from a
     * prototype, sharing immutable configuration and the GDRRT planner
     * while starting with a clean runtime state.
     */
    public WOAFogMovement(WOAFogMovement proto) {
        super(proto);
        this.startLoc               = proto.startLoc.clone();
        this.obstacleFilePath       = proto.obstacleFilePath;
        this.updateInterval         = proto.updateInterval;
        this.communicationRange     = proto.communicationRange;
        this.populationSize         = proto.populationSize;
        this.maxIterations          = proto.maxIterations;
        this.spiralB                = proto.spiralB;
        this.targetMoveThreshold    = proto.targetMoveThreshold;
        this.coverageWeight         = proto.coverageWeight;
        this.priorityWeight         = proto.priorityWeight;
        this.enableShutdown         = proto.enableShutdown;
        this.shutdownTime           = proto.shutdownTime;
        this.shutdownTriggered      = proto.shutdownTriggered;
        this.reassignmentSuccessful = proto.reassignmentSuccessful;
        this.actualShutdownTime     = proto.actualShutdownTime;
        this.lastExecutionTimeMs    = proto.lastExecutionTimeMs;
        this.dronePriorityMap       = new java.util.HashMap<>(proto.dronePriorityMap);
        /* The GDRRT planner is shared; each instance initialises its own
         * planning state via gdrrt.init() on the first getPath() call. */
        this.gdrrt                  = proto.gdrrt;
    }

    @Override
    public Coord getInitialLocation() {
        return startLoc.clone();
    }

    /**
     * Returns the next path segment for the fog UAV. Called by the simulator
     * each time the node finishes its current movement.
     *
     */
    @Override
    public Path getPath() {
        double now    = SimClock.getTime();
        Coord  fogPos = (getHost() != null)
                      ? getHost().getLocation() : startLoc;

        // Random Drone Shutdown Event execution
        if (enableShutdown && !shutdownTriggered && now >= shutdownTime) {
            triggerRandomShutdown();
        }

    
        boolean firstRun        = (lastUpdateTime < 0);
        boolean intervalElapsed = !firstRun && (now - lastUpdateTime) >= updateInterval;

        if (firstRun || intervalElapsed) {
            Coord newTarget = runWOA();

            if (newTarget != null) {
                boolean noCurrentTarget = (currentOptimalTarget == null);
                double  shift = noCurrentTarget ? Double.NaN
                        : newTarget.distance(currentOptimalTarget);
                boolean targetChanged = noCurrentTarget
                        || shift > targetMoveThreshold;

             
                if (targetChanged) {
                    currentOptimalTarget = newTarget;
                    if (getHost() != null) {
                        gdrrt.init(getHost().getLocation(), currentOptimalTarget);
                    }
                }
            }
            lastUpdateTime = now;
        }

    
        if (currentOptimalTarget == null) {
            return null;
        }

     
        if (!gdrrt.isInitialized() && getHost() != null) {
            gdrrt.init(getHost().getLocation(), currentOptimalTarget);
        }
        if (!gdrrt.isInitialized()) {
            return null;
        }

        GDRRTPlanner.PlannedSegment proposed = gdrrt.planNextSegment();
        if (proposed == null) {
            ExtendedDronePathManager.setStationary(getHost().getAddress());
            return null;
        }

      
        if (ExtendedDronePathManager.requestPath(getHost().getAddress(), proposed.path)) {
            isWaiting = false;
            gdrrt.commit(proposed);
            return proposed.path;
        } else {
            
            isWaiting = true;
            ExtendedDronePathManager.setStationary(getHost().getAddress());
            gdrrt.init(getHost().getLocation(), currentOptimalTarget);

            Path hold = new Path(0);
            hold.addWaypoint(getHost().getLocation());
            return hold;
        }
    }

    @Override
    public WOAFogMovement replicate() {
        return new WOAFogMovement(this);
    }

    /**
     * Returns the sim time at which the next path will be available.
     * When the node is in a collision-avoidance hold, this adds a 1-second
     * delay to prevent infinite replanning loops.
    
     */
    @Override
    public double nextPathAvailable() {
        if (isWaiting) {
            /* Back-off delay: retry after 1 simulated second */
            return core.SimClock.getTime() + 1.0;
        }
        return 0;
    }

    /**
     * Returns the most recent WOA-optimal target position, or 
     * if no WOA cycle has completed yet, then it returns
     * the current optimal target coordinate
     */
    public Coord getCurrentOptimalTarget() { return currentOptimalTarget; }

    /**
     * Returns whether the random drone shutdown event has been triggered,
     * that is, if the shutdown event has fired
     */
    public boolean isShutdownTriggered() { return shutdownTriggered; }

    /**
     * Returns whether the most recent shutdown event successfully reassigned
     * the killed drone's target to another in-range drone
     * that is, if reassignment succeeded
     */
    public boolean isReassignmentSuccessful() { return reassignmentSuccessful; }

    /**
     * Returns the actual simulation time at which the shutdown event fired
    *  Returns shutdown trigger time in seconds
     */
    public double getShutdownTime() { return actualShutdownTime; }

    /**
     * Returns and resets the wall-clock execution time of the most recent
     * WOA cycle in milliseconds. Resets to 0.0 after each call.
     * 
     */
    public double pollExecutionTimeMs() {
        double val = lastExecutionTimeMs;
        lastExecutionTimeMs = 0;
        return val;
    }

    /**
     * Executes one full WOA optimisation cycle and returns the fog UAV
     * position that maximises the hybrid fitness function
     */
    private Coord runWOA() {
        long startTime = System.nanoTime();
        woaCycleCount++;
        double  now     = SimClock.getTime();
        DTNHost fogHost = getHost();

        if (fogHost == null) {
            lastExecutionTimeMs = (System.nanoTime() - startTime) / 1_000_000.0;
            return null;
        }

        
        List<DroneInfo> droneInfos = collectActiveDroneInfo(fogHost);
        int    m      = droneInfos.size();
        double pTotal = computePTotal(droneInfos);

        if (m == 0) {
            // No drones visible — stay at current position until next cycle 
            lastExecutionTimeMs = (System.nanoTime() - startTime) / 1_000_000.0;
            return fogHost.getLocation();
        }

        initWorldBounds();
        /* Search bounds: keep fog UAV at least one comm-range distance from world edges
         * to preserve coverage headroom. */
        double margin = communicationRange;
        double minX   = margin;
        double maxX   = worldWidth  - margin;
        double minY   = margin;
        double maxY   = worldHeight - margin;

        
        List<Whale> population = new ArrayList<>(populationSize);
        Coord fogPos = fogHost.getLocation();


        for (int i = 0; i < populationSize - 1; i++) {
            DroneInfo d = droneInfos.get(rng.nextInt(droneInfos.size()));
            double dx   = (rng.nextDouble() - 0.5) * 2.0 * communicationRange;
            double dy   = (rng.nextDouble() - 0.5) * 2.0 * communicationRange;
            double wx   = Math.max(minX, Math.min(maxX, d.location.getX() + dx));
            double wy   = Math.max(minY, Math.min(maxY, d.location.getY() + dy));
            population.add(new Whale(wx, wy));
        }
        
        population.add(new Whale(fogPos.getX(), fogPos.getY()));

    
        for (Whale w : population) {
            w.fitness = evaluate(w, droneInfos, m, pTotal);
        }

        Whale bestWhale = findBest(population);

        
        for (int iter = 1; iter <= maxIterations; iter++) {
            
            double a  = 2.0 - 2.0 * iter / (double) maxIterations;
            double a2 = -1.0 + (-1.0) * iter / (double) maxIterations;

            for (Whale whale : population) {
                double r = rng.nextDouble();
                double A = 2.0 * a * r - a;
                double C = 2.0 * r;
                double l = (a2 - 1.0) * rng.nextDouble() + 1.0;
                double p = rng.nextDouble();

                if (p < 0.5) {
                    if (Math.abs(A) < 1) {
                        // Encircling prey: move toward the current best whale
                        double D_x = Math.abs(C * bestWhale.x - whale.x);
                        double D_y = Math.abs(C * bestWhale.y - whale.y);
                        whale.x = bestWhale.x - A * D_x;
                        whale.y = bestWhale.y - A * D_y;
                    } else {
                        // Random search: move toward a randomly selected whale
                        // (exploration phase)
                        Whale randWhale = population.get(rng.nextInt(population.size()));
                        double D_x = Math.abs(C * randWhale.x - whale.x);
                        double D_y = Math.abs(C * randWhale.y - whale.y);
                        whale.x = randWhale.x - A * D_x;
                        whale.y = randWhale.y - A * D_y;
                    }
                } else {
                    // Bubble-net attack: spiral update toward best whale
                    double dist = Math.sqrt(
                        (bestWhale.x - whale.x) * (bestWhale.x - whale.x)
                      + (bestWhale.y - whale.y) * (bestWhale.y - whale.y)
                    );
                    whale.x = dist * Math.exp(spiralB * l) * Math.cos(2 * Math.PI * l)
                            + bestWhale.x;
                    whale.y = dist * Math.exp(spiralB * l) * Math.sin(2 * Math.PI * l)
                            + bestWhale.y;
                }

                
                whale.x = Math.max(minX, Math.min(maxX, whale.x));
                whale.y = Math.max(minY, Math.min(maxY, whale.y));

                // Evaluate new position and update global best if improved
                whale.fitness = evaluate(whale, droneInfos, m, pTotal);
                if (whale.fitness > bestWhale.fitness) {
                    bestWhale = whale.copy();
                }
            }
        }

        lastExecutionTimeMs = (System.nanoTime() - startTime) / 1_000_000.0;
        return bestWhale.toCoord();
    }

    

    /**
     * Selects one random active mission drone, kills it, and attempts to
     * reassign its target to a lower-priority drone within communication range.     
     */
    private void triggerRandomShutdown() {
        shutdownTriggered  = true;
        actualShutdownTime = SimClock.getTime();
        System.out.println("[WOA-EVENT] >>> CRITICAL EVENT: Random Drone Shutdown "
                + "Triggered at t=" + fmt(SimClock.getTime()) + " <<<");

        // Collect all active mission drones
        List<DTNHost> activeDrones = new ArrayList<>();
        for (DTNHost host : SimScenario.getInstance().getHosts()) {
            MovementModel mm = host.getMovement();
         
            if (mm instanceof ExtendedGDRRTMovement) {
                ExtendedGDRRTMovement gmm = (ExtendedGDRRTMovement) mm;
                if (!gmm.isDone() && !gmm.isDead()) {
                    activeDrones.add(host);
                }
            }
        }

        if (activeDrones.isEmpty()) {
            System.out.println("[WOA-EVENT]   Shutdown aborted: no active "
                    + "mission drones found.");
            return;
        }

        //  Pick a random active drone to shut down
        Collections.shuffle(activeDrones, new Random(System.nanoTime()));
        DTNHost killedDrone = activeDrones.get(0);

        
        if (!(killedDrone.getMovement() instanceof ExtendedGDRRTMovement)) {
            System.out.println("[WOA-EVENT]   Shutdown aborted: selected drone "
                    + "does not have an ExtendedGDRRTMovement — cannot kill.");
            return;
        }
        ExtendedGDRRTMovement killedMm =
                (ExtendedGDRRTMovement) killedDrone.getMovement();

        double killedPriority = killedMm.getPriority();
        Coord  killedTarget   = killedMm.getEndLocation();

        System.out.println("[WOA-EVENT] >>> DRONE FAILURE: " + killedDrone.getName()
                + " (prio=" + fmt(killedPriority) + ") has been killed! <<<");
        killedMm.kill();

        activeDrones.remove(killedDrone);

        // Scan for a lower-priority replacement drone in communication range
        DTNHost replacement       = null;
        double  bestPriorityFound = Double.MAX_VALUE;
        Coord   fogPos            = getHost().getLocation();

        for (DTNHost host : activeDrones) {
          
            if (!(host.getMovement() instanceof ExtendedGDRRTMovement)) continue;
            ExtendedGDRRTMovement mm = (ExtendedGDRRTMovement) host.getMovement();
            double priority = mm.getPriority();

            /* Criteria: not done, not dead, lower priority than the killed
             * drone, and within communicationRange 
            . Among all candidates, pick the one with the
             * lowest priority. */
            if (!mm.isDone() && !mm.isDead() && priority < killedPriority) {
                double dist = host.getLocation().distance(fogPos);
                if (dist <= communicationRange) {
                    if (priority < bestPriorityFound) {
                        bestPriorityFound = priority;
                        replacement = host;
                    }
                }
            }
        }

        // Reassign the killed drone's target if a suitable replacement was found
        if (replacement != null) {
    
            ExtendedGDRRTMovement replacementMm =
                    (ExtendedGDRRTMovement) replacement.getMovement();
            System.out.println("[WOA-EVENT] >>> FOG ACTION: Reassigning target "
                    + fmtCoord(killedTarget)
                    + " to lower priority in-range Drone " + replacement.getName()
                    + " (previous prio=" + fmt(replacementMm.getPriority())
                    + " -> new prio=" + fmt(killedPriority) + ") <<<");
            replacementMm.changeTarget(killedTarget, killedPriority);
            reassignmentSuccessful = true;
        } else {
            reassignmentSuccessful = false;
            System.out.println("[WOA-EVENT] >>> FOG ACTION: Drone "
                    + killedDrone.getName()
                    + " failed, but NO lower priority drone is in range ("
                    + fmt(communicationRange) + "m) for reassignment. <<<");
        }
    }

    

    /**
     * Scans all hosts in the scenario and builds a list of DroneInfo
     * snapshots for drones and Fog UAVs and other DTNHosts
     */
    private List<DroneInfo> collectActiveDroneInfo(DTNHost fogHost) {
        List<DroneInfo> infos  = new ArrayList<>();
        Coord           fogPos = fogHost.getLocation();

        // ── Pass 1 (one-time): discover the drone group's base address
        // Converts absolute host addresses to readable D0, D1... labels.
        if (droneGroupBaseAddr == DRONE_GROUP_ADDR_UNKNOWN) {
            for (DTNHost h : SimScenario.getInstance().getHosts()) {
                if (h == fogHost) continue;
                MovementModel mm = h.getMovement();
                boolean isMission =
                        (mm.getClass() == ExtendedGDRRTMovement.class) ||
                        (mm.getClass() == FogDeployedGDRRTMovement.class);
                if (!isMission) continue;
                if (droneGroupBaseAddr == DRONE_GROUP_ADDR_UNKNOWN
                        || h.getAddress() < droneGroupBaseAddr) {
                    droneGroupBaseAddr = h.getAddress();
                }
            }
        }

        // ── Pass 2: evaluate every host against all filter criteria
        for (DTNHost host : SimScenario.getInstance().getHosts()) {
            // Skip the fog UAV itself
            if (host == fogHost) {
                continue;
            }

            MovementModel mm = host.getMovement();

            // Only accept exact mission-drone classes (no non-mission hosts)
            boolean isMission =
                    (mm.getClass() == ExtendedGDRRTMovement.class) ||
                    (mm.getClass() == FogDeployedGDRRTMovement.class);
            if (!isMission) {
                continue;
            }

            /* Use instanceof for the isDone() check — safe for both classes. */
            int     dIdx = (droneGroupBaseAddr != DRONE_GROUP_ADDR_UNKNOWN)
                         ? host.getAddress() - droneGroupBaseAddr : -1;
            boolean done = (mm instanceof ExtendedGDRRTMovement)
                        && ((ExtendedGDRRTMovement) mm).isDone();

            // Skip drones that have completed their mission
            if (done) {
                continue;
            }

            // range filter — the core local-knowledge assumption
            Coord  dronePos  = host.getLocation();
            double distToFog = fogPos.distance(dronePos);
            double priority  = getDronePriority(host);

            if (distToFog > communicationRange) {
                continue;
            }

            // Drone passes all filters — include in this cycle's snapshot
            infos.add(new DroneInfo(dIdx, dronePos.clone(), priority, distToFog));
        }

        return infos;
    }

    

    /**
     * Evaluates the hybrid coverage-priority fitness function H(X) for one
     * candidate position (whale).
     * the candidate position to evaluate
     *  snapshot of in-range drones for this WOA cycle
     * number of in-range drones  droneInfos.size()
     * priority denominator from computePTotal
     */
    private double evaluate(Whale whale, List<DroneInfo> droneInfos,
                            int m, double pTotal) {
        if (m == 0) return 0.0;

        double ncv2    = 0.0;
        double pScore  = 0.0;
        int    covered = 0;

        Coord whalePos = whale.toCoord();

        for (DroneInfo d : droneInfos) {
            double dist = whalePos.distance(d.location);
            if (dist <= communicationRange) {
                covered++;
                double proximity = 1.0 - (dist / communicationRange);
                ncv2   += proximity * proximity;
                pScore += d.priority * (1.0 + proximity);
            }
        }

        double coverageTerm = ncv2 / m;
        double priorityTerm = (pTotal > 0) ? (pScore / pTotal) : 0.0;

        return coverageWeight * coverageTerm + priorityWeight * priorityTerm;
    }

    /**
     * Returns the priority weight for one mission drone host.
     * Converts the host's absolute address to a zero-based drone index using
     * droneGroupBaseAddr, then looks up #dronePriorityMap.
     * Defaults to 1.0 if the drone has no explicit priority entry, or if the
     * base address has not yet been discovered.
     */
    private double getDronePriority(DTNHost droneHost) {
        if (droneGroupBaseAddr == DRONE_GROUP_ADDR_UNKNOWN) {
            return 1.0;
        }
        int    idx      = droneHost.getAddress() - droneGroupBaseAddr;
        double priority = dronePriorityMap.getOrDefault(idx, 1.0);
        return priority;
    }

    /**
     * Parses the DRONE_PRIORITIES_S setting into an index-to-priority
     * map. The expected format is  [w0; w1; w2; ...], where each entry
     * is a positive double. Returns an empty map if the key is absent, in
     * which case all drones default to priority 1.0.
     */
    private java.util.Map<Integer, Double> parseDronePriorities(Settings s) {
        java.util.Map<Integer, Double> map = new java.util.HashMap<>();
        String fullKey = WOA_FOG_NS + DRONE_PRIORITIES_S;

        if (!s.contains(fullKey)) {
            return map;
        }

        String raw = s.getRawSetting(fullKey);
        raw = raw.replace("[", "").replace("]", "");

        int idx = 0;
        for (String part : raw.split(";")) {
            part = part.trim();
            if (part.isEmpty()) continue;
            double w = Double.parseDouble(part);
            if (w <= 0) {
                throw new SettingsError("[WOAFogMovement] dronePriorities"
                        + " entry at index " + idx
                        + " must be positive, got: " + w);
            }
            map.put(idx, w);
            idx++;
        }

        return map;
    }

    /**
     * Computes the P_total denominator for the priority term in H(X).     
     */
    private double computePTotal(List<DroneInfo> droneInfos) {
        double sum = 0.0;
        for (DroneInfo d : droneInfos) sum += d.priority * 2.0;
        return sum;
    }

    /**
     * Returns the whale with the highest fitness value from the population.     
     * 
     */
    private Whale findBest(List<Whale> population) {
        Whale best = population.get(0);
        for (Whale w : population) {
            if (w.fitness > best.fitness) best = w;
        }
        return best;
    }

    /**
     * Returns the whale with the lowest fitness value from the population.
     */
    private Whale findWorst(List<Whale> population) {
        Whale worst = population.get(0);
        for (Whale w : population) {
            if (w.fitness < worst.fitness) worst = w;
        }
        return worst;
    }

    /**
     * Reads world dimensions 
     */
    private void initWorldBounds() {
        if (worldWidth < 0) {
            worldWidth  = getMaxX();
            worldHeight = getMaxY();
        }
    }

    

    /**
     * Formats a double to 4 decimal places.
     */
    private static String fmt(double v) {
        return String.format("%.4f", v);
    }

    /**
     Right-pads an integer to a fixed column width.
     */
    private static String padInt(int v, int width) {
        return String.format("%" + width + "d", v);
    }

    /**
     * Formats the coordinat with 1 decimal place.
     */
    private static String fmtCoord(Coord c) {
        return String.format("(%.1f, %.1f)", c.getX(), c.getY());
    }
}
