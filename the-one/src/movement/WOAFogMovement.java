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
 * Movement model for a Fog UAV Base Station using the Whale Optimization
 * Algorithm (WOA), based on Naouri et al. (2024).
 *
 * ── DESIGN CHOICE: OPTION B (LOCAL KNOWLEDGE ONLY) ──────────────────────────
 * The fog UAV only knows about drones currently within its communicationRange.
 * It cannot plan for drones it cannot communicate with. Consequences:
 *   - Priority system only applies to VISIBLE drones (in range right now).
 *   - WOA explores positions biased around currently-visible drones.
 *   - High-priority drones outside range are unknown and uninfluential.
 *   - m and pTotal are computed from the in-range snapshot only.
 *
 * ── PRIORITY-WEIGHTED OBJECTIVE ─────────────────────────────────────────────
 *     H(X) = alpha * NCV2(G*)/m  +  beta * PriorityScore(G*)/P_total
 *
 * where all quantities are computed from the in-range drone snapshot only.
 *
 * ── PROXIMITY-WEIGHTED PRIORITY TERM (B-08 DOCUMENTATION) ───────────────────
 * The priority term uses a DISTANCE-WEIGHTED score, not a simple binary one.
 * For each covered drone i:
 *
 *   proximity_i   = 1 - dist(fog, drone_i) / R        (0.0 at edge, 1.0 co-located)
 *   weightedPrio_i = priority_i * (1 + proximity_i)   (range: [prio, 2*prio])
 *   pScore        = SUM of all weightedPrio_i for covered drones
 *   P_total       = SUM of (priority_i * 2) for ALL snapshot drones
 *
 * This means:
 *   - priorityTerm = pScore / P_total  is correctly bounded to [0, 1].
 *   - However, at typical coverage distances (not co-located), priorityTerm
 *     will be in the range [0.5, 1.0] rather than [0.0, 1.0].
 *   - As a result, priorityWeight behaves as if it has LESS influence than
 *     its face value suggests compared to coverageWeight.
 *
 * TUNING GUIDANCE:
 *   To achieve a perceived 50/50 balance between coverage and priority,
 *   set priorityWeight higher than coverageWeight, e.g.:
 *     coverageWeight = 0.4
 *     priorityWeight = 0.6
 *   The auto-normaliser will scale these to sum to 1.0 automatically.
 *   The benefit of this design is a smooth fitness GRADIENT that pulls the
 *   fog UAV closer to high-priority drones rather than treating any in-range
 *   position as equally fit.
 */
public class WOAFogMovement extends MovementModel {

    // ═══════════════════════════════════════════════════════════════════════
    //  SECTION 1 — Settings Keys
    //  All keys are prefixed with WOA_FOG_NS = "WOAFogMovement."
    //  In the settings file use: Group3.WOAFogMovement.<key> = <value>
    // ═══════════════════════════════════════════════════════════════════════
    public static final String WOA_FOG_NS          = "WOAFogMovement.";

    // ── Core location / path ─────────────────────────────────────────────
    public static final String START_LOCATION_S    = "startLocation";
    public static final String OBSTACLE_FILE_S     = "obstacleFile";

    // ── WOA algorithm parameters ─────────────────────────────────────────
    public static final String UPDATE_INTERVAL_S   = "updateInterval";
    public static final String COMM_RANGE_S        = "communicationRange";
    public static final String POPULATION_SIZE_S   = "populationSize";
    public static final String MAX_ITERATIONS_S    = "maxIterations";
    public static final String SPIRAL_B_S          = "woaSpiralB";
    public static final String TARGET_THRESHOLD_S  = "targetMoveThreshold";

    // ── Objective function weights ────────────────────────────────────────
    public static final String COVERAGE_WEIGHT_S   = "coverageWeight";   // alpha
    public static final String PRIORITY_WEIGHT_S   = "priorityWeight";   // beta
    public static final String DRONE_PRIORITIES_S  = "dronePriorities";

    // ── Random Shutdown parameters ────────────────────────────────────────
    /** Enable random drone shutdown event. */
    public static final String ENABLE_SHUTDOWN_S   = "enableRandomShutdown";
    /** Sim time at which to trigger the shutdown. */
    public static final String SHUTDOWN_TIME_S     = "shutdownTime";

    private static final int DRONE_GROUP_ADDR_UNKNOWN = -1;

    // ═══════════════════════════════════════════════════════════════════════
    //  SECTION 2 — Configuration Fields
    //  All fields are set once in the constructor and never mutated.
    // ═══════════════════════════════════════════════════════════════════════
    private final Coord  startLoc;            // fog UAV spawn point
    private final String obstacleFilePath;    // obstacle map for GDRRT
    private final double updateInterval;      // seconds between WOA cycles
    private final double communicationRange;  // coverage radius R (metres)
    private final int    populationSize;      // number of WOA whales
    private final int    maxIterations;       // WOA iterations per cycle
    private final double spiralB;             // spiral shape constant b
    private final double targetMoveThreshold; // min shift (m) to re-plan path
    private final double coverageWeight;      // alpha — NCV2 term weight
    private final double priorityWeight;      // beta  — priority term weight

    private final boolean enableShutdown;     // random shutdown toggle
    private final double  shutdownTime;       // trigger time

    private final java.util.Map<Integer, Double> dronePriorityMap;

    // Lazily discovered on the first scan; never changes once set
    private int droneGroupBaseAddr = DRONE_GROUP_ADDR_UNKNOWN;

    // ═══════════════════════════════════════════════════════════════════════
    //  SECTION 3 — Runtime State Fields
    //  Mutable fields updated every WOA cycle / movement step.
    // ═══════════════════════════════════════════════════════════════════════
    private Coord   currentOptimalTarget = null; // last WOA-chosen position
    private double  lastUpdateTime       = -1.0; // sim time of last WOA run
    private boolean isWaiting            = false; // collision-avoidance hold
    private int     woaCycleCount        = 0;     // readable log counter
    private boolean shutdownTriggered    = false; // event latch

    private final GDRRTPlanner gdrrt;       // obstacle-aware path planner
    private final Random       rng = new Random(); // WOA random source

    private double worldWidth  = -1;   // set once by initWorldBounds()
    private double worldHeight = -1;

    // -----------------------------------------------------------------------
    //  Inner class: DroneInfo
    //  Captures everything the fog UAV knows about one in-range drone.
    // -----------------------------------------------------------------------
    private static class DroneInfo {
        final int    droneIndex;    // zero-based, used for log readability
        final Coord  location;
        final double priority;
        final double distanceToFog; // distance from fog's CURRENT position (not candidate)

        DroneInfo(int droneIndex, Coord location, double priority, double distToFog) {
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

    // -----------------------------------------------------------------------
    //  Inner class: Whale
    //  One candidate 2-D position for the fog vehicle.
    // -----------------------------------------------------------------------
    private static class Whale {
        double x;
        double y;
        double fitness;

        Whale(double x, double y) {
            this.x       = x;
            this.y       = y;
            this.fitness = Double.NEGATIVE_INFINITY;
        }

        Whale copy() {
            Whale w   = new Whale(this.x, this.y);
            w.fitness = this.fitness;
            return w;
        }

        Coord toCoord() { return new Coord(x, y); }

        @Override
        public String toString() {
            return String.format("pos=(%.1f,%.1f) fit=%.4f", x, y, fitness);
        }
    }

    // =======================================================================
    //  Constructors
    // =======================================================================

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

    public WOAFogMovement(WOAFogMovement proto) {
        super(proto);
        this.startLoc            = proto.startLoc.clone();
        this.obstacleFilePath    = proto.obstacleFilePath;
        this.updateInterval      = proto.updateInterval;
        this.communicationRange  = proto.communicationRange;
        this.populationSize      = proto.populationSize;
        this.maxIterations       = proto.maxIterations;
        this.spiralB             = proto.spiralB;
        this.targetMoveThreshold = proto.targetMoveThreshold;
        this.coverageWeight      = proto.coverageWeight;
        this.priorityWeight      = proto.priorityWeight;
        this.enableShutdown      = proto.enableShutdown;
        this.shutdownTime        = proto.shutdownTime;
        this.dronePriorityMap    = new java.util.HashMap<>(proto.dronePriorityMap);
        this.gdrrt               = proto.gdrrt;
    }

    @Override
    public Coord getInitialLocation() {
        return startLoc.clone();
    }

    @Override
    public Path getPath() {
        double now    = SimClock.getTime();
        Coord  fogPos = (getHost() != null)
                      ? getHost().getLocation() : startLoc;

        // Random Drone Shutdown Event execution
        if (enableShutdown && !shutdownTriggered && now >= shutdownTime) {
            triggerRandomShutdown();
        }

        // ------------------------------------------------------------------
        // 1. Re-run WOA when the update interval has elapsed
        // ------------------------------------------------------------------
        boolean firstRun       = (lastUpdateTime < 0);
        boolean intervalElapsed= !firstRun && (now - lastUpdateTime) >= updateInterval;

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

        // ------------------------------------------------------------------
        // 2. Nothing to navigate toward yet
        // ------------------------------------------------------------------
        if (currentOptimalTarget == null) {
            return null;
        }

        // ------------------------------------------------------------------
        // 3. Ensure GDRRT is initialised
        // ------------------------------------------------------------------
        if (!gdrrt.isInitialized() && getHost() != null) {
            gdrrt.init(getHost().getLocation(), currentOptimalTarget);
        }
        if (!gdrrt.isInitialized()) {
            return null;
        }

        // ------------------------------------------------------------------
        // 4. Plan next GDRRT segment
        // ------------------------------------------------------------------
        GDRRTPlanner.PlannedSegment proposed = gdrrt.planNextSegment();
        if (proposed == null) {
            DronePathManager.setStationary(getHost().getAddress());
            return null;
        }

        // ------------------------------------------------------------------
        // 5. Collision-avoidance handshake with DronePathManager
        // ------------------------------------------------------------------
        if (DronePathManager.requestPath(getHost().getAddress(), proposed.path)) {
            isWaiting = false;
            gdrrt.commit(proposed);
            if (proposed.isFinalPath) {
                DronePathManager.setStationary(getHost().getAddress());
            }
            return proposed.path;
        } else {
            isWaiting = true;
            DronePathManager.setStationary(getHost().getAddress());
            Path hold = new Path(0);
            hold.addWaypoint(getHost().getLocation());
            return hold;
        }
    }

    @Override
    public WOAFogMovement replicate() {
        return new WOAFogMovement(this);
    }

    // =======================================================================
    //  WOA core
    // =======================================================================

    /**
     * Executes one full WOA cycle.
     * Finds the fog UAV position that maximises:
     *   H(X) = alpha * NCV2(G*)/m  +  beta * PriorityScore(G*)/P_total
     *
     * All quantities are computed from the in-range drone snapshot (Option B).
     *
     * @return optimal Coord to move toward, or null if no drones are in range.
     */
    private Coord runWOA() {
        woaCycleCount++;
        double  now     = SimClock.getTime();
        DTNHost fogHost = getHost();

        if (fogHost == null) {
            return null;
        }

        // ── OPTION B: collect only in-range drones ────────────────────────
        List<DroneInfo> droneInfos = collectActiveDroneInfo(fogHost);
        int    m      = droneInfos.size();
        double pTotal = computePTotal(droneInfos);

        if (m == 0) {
            return fogHost.getLocation();
        }

        initWorldBounds();
        // Search bounds: keep fog away from world edges by one comm-range
        double margin = communicationRange;
        double minX   = margin;
        double maxX   = worldWidth  - margin;
        double minY   = margin;
        double maxY   = worldHeight - margin;

        // ── Initialise WOA population ──────────────────────────────────────
        List<Whale> population = new ArrayList<>(populationSize);
        Coord fogPos = fogHost.getLocation();

        // Seeded positions around visible drones
        for (int i = 0; i < populationSize - 1; i++) {
            DroneInfo d = droneInfos.get(rng.nextInt(droneInfos.size()));
            double dx   = (rng.nextDouble() - 0.5) * 2.0 * communicationRange;
            double dy   = (rng.nextDouble() - 0.5) * 2.0 * communicationRange;
            double wx   = Math.max(minX, Math.min(maxX, d.location.getX() + dx));
            double wy   = Math.max(minY, Math.min(maxY, d.location.getY() + dy));
            population.add(new Whale(wx, wy));
        }
        // Elite whale at fog's current position (exploitation anchor)
        population.add(new Whale(fogPos.getX(), fogPos.getY()));

        // ── Initial fitness evaluation ─────────────────────────────────────
        for (Whale w : population) {
            w.fitness = evaluate(w, droneInfos, m, pTotal);
        }

        Whale bestWhale = findBest(population);

        // ── WOA main iterations ────────────────────────────────────────────
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
                    // Encircling or random search
                    if (Math.abs(A) < 1) {
                        // Encircling prey (best whale)
                        double D_x = Math.abs(C * bestWhale.x - whale.x);
                        double D_y = Math.abs(C * bestWhale.y - whale.y);
                        whale.x = bestWhale.x - A * D_x;
                        whale.y = bestWhale.y - A * D_y;
                    } else {
                        // Random search (explore)
                        Whale randWhale = population.get(rng.nextInt(population.size()));
                        double D_x = Math.abs(C * randWhale.x - whale.x);
                        double D_y = Math.abs(C * randWhale.y - whale.y);
                        whale.x = randWhale.x - A * D_x;
                        whale.y = randWhale.y - A * D_y;
                    }
                } else {
                    // Spiral updating position
                    double dist = Math.sqrt(
                        (bestWhale.x - whale.x)*(bestWhale.x - whale.x)
                      + (bestWhale.y - whale.y)*(bestWhale.y - whale.y)
                    );
                    whale.x = dist * Math.exp(spiralB * l) * Math.cos(2 * Math.PI * l)
                            + bestWhale.x;
                    whale.y = dist * Math.exp(spiralB * l) * Math.sin(2 * Math.PI * l)
                            + bestWhale.y;
                }

                // Clamp to search bounds
                whale.x = Math.max(minX, Math.min(maxX, whale.x));
                whale.y = Math.max(minY, Math.min(maxY, whale.y));

                // Evaluate new position
                whale.fitness = evaluate(whale, droneInfos, m, pTotal);

                // Update best if improved
                if (whale.fitness > bestWhale.fitness) {
                    bestWhale = whale.copy();
                }
            }
        }

        return bestWhale.toCoord();
    }

    // =======================================================================
    //  Random Drone Shutdown Event (FIX-B05)
    // =======================================================================

    /**
     * Selects one random ACTIVE drone, kills it, and attempts to reassign
     * its target to a lower-priority drone within communication range.
     *
     * FIX-B05: This completely replaces the previous triggerRandomShutdown()
     *          approach and removes the need for the identity guard entirely.
     */
    private void triggerRandomShutdown() {
        shutdownTriggered = true;
        System.out.println("[WOA-EVENT] >>> CRITICAL EVENT: Random Drone Shutdown Triggered at t="
                + fmt(SimClock.getTime()) + " <<<");

        // Collect all active mission drones
        List<DTNHost> activeDrones = new ArrayList<>();
        for (DTNHost host : SimScenario.getInstance().getHosts()) {
            MovementModel mm = host.getMovement();
            // FIX-B01: instanceof is correct here — it accepts both
            // GDRRTMovement and FogDeployedGDRRTMovement (a subclass).
            if (mm instanceof GDRRTMovement) {
                GDRRTMovement gmm = (GDRRTMovement) mm;
                if (!gmm.isDone() && !gmm.isDead()) {
                    activeDrones.add(host);
                }
            }
        }

        if (activeDrones.isEmpty()) {
            System.out.println("[WOA-EVENT]   Shutdown aborted: no active mission drones found.");
            return;
        }

        // 1. Pick a random active drone to shut down
        // FIX-B06: Use System.nanoTime() so the shuffle is truly random across
        //          simulation runs. The old SimClock-seeded RNG produced the
        //          same permutation every run because shutdownTime is fixed.
        Collections.shuffle(activeDrones, new Random(System.nanoTime()));
        DTNHost killedDrone = activeDrones.get(0);

        // FIX-B01: Guard the cast with instanceof before committing to it.
        if (!(killedDrone.getMovement() instanceof GDRRTMovement)) {
            System.out.println("[WOA-EVENT]   Shutdown aborted: selected drone does not have"
                    + " a GDRRTMovement — cannot kill.");
            return;
        }
        GDRRTMovement killedMm = (GDRRTMovement) killedDrone.getMovement();

        double killedPriority = killedMm.getPriority();
        Coord  killedTarget   = killedMm.getEndLocation();

        System.out.println("[WOA-EVENT] >>> DRONE FAILURE: " + killedDrone.getName()
                + " (prio=" + fmt(killedPriority) + ") has been killed! <<<");
        killedMm.kill();

        // FIX-B07: Remove the killed drone from the candidate list immediately
        //          after kill(). This ensures the replacement search cannot
        //          accidentally select the dead drone, regardless of whether
        //          isDead() is yet observable by the loop (avoids relying on
        //          the fragile identity check `host == killedDrone`).
        activeDrones.remove(killedDrone);

        // 2. Scan for a lower-priority replacement drone in range
        DTNHost replacement      = null;
        double  bestPriorityFound = Double.MAX_VALUE;
        Coord   fogPos            = getHost().getLocation();

        for (DTNHost host : activeDrones) {
            // FIX-B01: Guard the cast inside the replacement loop.
            if (!(host.getMovement() instanceof GDRRTMovement)) continue;
            GDRRTMovement mm = (GDRRTMovement) host.getMovement();
            double priority  = mm.getPriority();

            // Criteria: not done, not dead, lower priority than killed drone,
            //           AND within communicationRange (settings-driven, not hard-coded).
            if (!mm.isDone() && !mm.isDead() && priority < killedPriority) {
                double dist = host.getLocation().distance(fogPos);
                if (dist <= communicationRange) {
                    // Pick the lowest-priority candidate from those available
                    if (priority < bestPriorityFound) {
                        bestPriorityFound = priority;
                        replacement = host;
                    }
                }
            }
        }

        // 3. Reassign if a suitable drone was found
        if (replacement != null) {
            // FIX-B01: Guard this cast too for consistency.
            GDRRTMovement replacementMm = (GDRRTMovement) replacement.getMovement();
            System.out.println("[WOA-EVENT] >>> FOG ACTION: Reassigning target "
                    + fmtCoord(killedTarget)
                    + " to lower priority in-range Drone " + replacement.getName()
                    + " (previous prio=" + fmt(replacementMm.getPriority())
                    + " -> new prio=" + fmt(killedPriority) + ") <<<");
            replacementMm.changeTarget(killedTarget, killedPriority);
        } else {
            System.out.println("[WOA-EVENT] >>> FOG ACTION: Drone " + killedDrone.getName()
                    + " failed, but NO lower priority drone is in range ("
                    + fmt(communicationRange) + "m) for reassignment. <<<");
        }
        
    }

    // =======================================================================
    //  Drone Scanning & Filtering (Option B: in-range only)
    // =======================================================================

    /**
     * Scans all hosts in the scenario and builds a list of DroneInfo objects
     * for those that are:
     *   - active mission drones (not the fog UAV, not a non-mission host)
     *   - not yet done (isDone() == false via instanceof GDRRTMovement check)
     *   - within communicationRange of the fog UAV (Option B constraint)
     *
     * Also updates droneGroupBaseAddr on first encounter with a mission drone.
     */
    private List<DroneInfo> collectActiveDroneInfo(DTNHost fogHost) {
        List<DroneInfo> infos   = new ArrayList<>();
        Coord           fogPos  = fogHost.getLocation();

        // ── Pass 1 (one-time): discover the drone group's base address ────
        // This lets us convert absolute host addresses to readable D0, D1...
        if (droneGroupBaseAddr == DRONE_GROUP_ADDR_UNKNOWN) {
            for (DTNHost h : SimScenario.getInstance().getHosts()) {
                if (h == fogHost) continue;
                MovementModel mm = h.getMovement();
                boolean isMission =
                        (mm.getClass() == GDRRTMovement.class) ||
                        (mm.getClass() == FogDeployedGDRRTMovement.class);
                if (!isMission) continue;
                if (droneGroupBaseAddr == DRONE_GROUP_ADDR_UNKNOWN
                        || h.getAddress() < droneGroupBaseAddr) {
                    droneGroupBaseAddr = h.getAddress();
                }
            }
        }

        // ── Pass 2: evaluate every host ───────────────────────────────────
        for (DTNHost host : SimScenario.getInstance().getHosts()) {
            // Skip the fog UAV itself
            if (host == fogHost) {
                continue;
            }

            MovementModel mm = host.getMovement();

            // Only accept exact mission-drone classes
            boolean isMission =
                    (mm.getClass() == GDRRTMovement.class) ||
                    (mm.getClass() == FogDeployedGDRRTMovement.class);
            if (!isMission) {
                continue;
            }

            // Use instanceof for the isDone() check — safe for both classes
            int     dIdx = (droneGroupBaseAddr != DRONE_GROUP_ADDR_UNKNOWN)
                         ? host.getAddress() - droneGroupBaseAddr : -1;
            boolean done = (mm instanceof GDRRTMovement)
                        && ((GDRRTMovement) mm).isDone();

            // Skip drones that have completed their mission
            if (done) {
                continue;
            }

            // OPTION B: range filter — the core local-knowledge assumption
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

    // =======================================================================
    //  WOA Algorithm: Fitness Evaluation (H function)
    // =======================================================================

    /**
     * Evaluates the hybrid coverage-priority fitness function H(X) for one
     * candidate position (whale).
     *
     * H(X) = alpha * (NCV2(G*) / m)  +  beta * (PriorityScore(G*) / P_total)
     *
     * where:
     *   - NCV2(G*) is the sum of proximity² for all drones covered by X
     *   - m        is the total number of in-range drones (Option B)
     *   - PriorityScore(G*) is the sum of priority*(1+proximity) for covered drones
     *   - P_total  is the sum of priority*2 across all in-range drones
     *   - alpha    is coverageWeight (normalised)
     *   - beta     is priorityWeight (normalised)
     */
    private double evaluate(Whale whale, List<DroneInfo> droneInfos, int m, double pTotal) {
        if (m == 0) return 0.0;

        double ncv2  = 0.0;
        double pScore = 0.0;
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
     * Converts absolute address to zero-based drone index using
     * droneGroupBaseAddr, then looks up dronePriorityMap.
     * Defaults to 1.0 if the drone has no explicit priority entry.
     */
    private double getDronePriority(DTNHost droneHost) {
        if (droneGroupBaseAddr == DRONE_GROUP_ADDR_UNKNOWN) {
            return 1.0;
        }
        int    idx      = droneHost.getAddress() - droneGroupBaseAddr;
        double  priority = dronePriorityMap.getOrDefault(idx, 1.0);
        return priority;
    }

    /**
     * Parses Group3.WOAFogMovement.dronePriorities from settings.
     * Format: "[w0; w1; w2; ...]" — one positive double per drone.
     * Returns an empty map if the key is absent (all drones default to 1.0).
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
     * Computes the P_total denominator for the priority term.
     *
     * FIX-B08 (Option 2 — document, don't change the maths):
     * P_total = SUM(priority_i * 2) for all snapshot drones.
     * The factor of 2 is intentional: it accounts for the maximum possible
     * proximity bonus in evaluate(). When fog is co-located with a drone,
     * the weighted contribution is priority*(1+1) = 2*priority. Dividing by
     * P_total = 2*sum(priorities) therefore correctly bounds priorityTerm to
     * [0, 1].
     *
     * SIDE EFFECT (documented): at typical coverage distances where proximity
     * is between 0 and 1, pScore per drone is between priority and 2*priority,
     * so priorityTerm operates in [0.5, 1.0] rather than the full [0, 1.0]
     * range. This makes priorityWeight effectively less influential than
     * coverageWeight at equal numeric values. See class-level Javadoc for
     * tuning guidance.
     */
    private double computePTotal(List<DroneInfo> droneInfos) {
        double sum = 0.0;
        for (DroneInfo d : droneInfos) sum += d.priority * 2.0;
        return sum;
    }

    /** Returns the whale with the highest fitness. Does NOT copy. */
    private Whale findBest(List<Whale> population) {
        Whale best = population.get(0);
        for (Whale w : population) {
            if (w.fitness > best.fitness) best = w;
        }
        return best;
    }

    /** Returns the whale with the lowest fitness. Does NOT copy. */
    private Whale findWorst(List<Whale> population) {
        Whale worst = population.get(0);
        for (Whale w : population) {
            if (w.fitness < worst.fitness) worst = w;
        }
        return worst;
    }

    /** Reads world dimensions from MovementModel on first call only. */
    private void initWorldBounds() {
        if (worldWidth < 0) {
            worldWidth  = getMaxX();
            worldHeight = getMaxY();
        }
    }

    // ── Formatting helpers ───────────────────────────────────────────────────

    /** Format a double to 4 decimal places. */
    private static String fmt(double v) {
        return String.format("%.4f", v);
    }

    /** Right-pad an integer to a fixed width. */
    private static String padInt(int v, int width) {
        return String.format("%" + width + "d", v);
    }

    /** Format a Coord as "(x, y)" with 1 decimal place. */
    private static String fmtCoord(Coord c) {
        return String.format("(%.1f, %.1f)", c.getX(), c.getY());
    }
}
