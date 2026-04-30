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
import java.io.FileWriter;
import java.io.PrintWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;

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
 * ── HOW TO READ THE LOGS ─────────────────────────────────────────────────────
 * Every log line is tagged so you can grep/filter for exactly what you need:
 *
 *   [WOA-DIAG]   General lifecycle events (startup, cycle begin/end)
 *   [WOA-SCAN]   Drone visibility scan — which drones are seen and why
 *   [WOA-INIT]   WOA population initialisation (whale starting positions)
 *   [WOA-ITER]   Per-iteration fitness progress (~10 checkpoints per run)
 *   [WOA-WHALE]  Per-whale position update details (enable LOG_PER_WHALE)
 *   [WOA-EVAL]   Detailed fitness breakdown (printed for the winning whale)
 *   [WOA-RESULT] Final WOA outcome per cycle
 *   [WOA-PATH]   Path planning and movement decisions
 *   [WOA-EVENT]  Special scenario events (like drone shutdowns)
 *   [WOA-PRIO]   Priority weight loading and per-drone lookup
 *
 * Example grep commands:
 *   grep "\[WOA-SCAN\]"   sim.log   # see which drones were visible each cycle
 *   grep "\[WOA-RESULT\]" sim.log   # see where the fog moved and why
 *   grep "OUT-OF-RANGE"   sim.log   # see which high-priority drones were missed
 *   grep "WARNING"        sim.log   # see all anomaly warnings
 */
public class WOAFogMovement extends MovementModel {

    // ═══════════════════════════════════════════════════════════════════════
    //  SECTION 1 — Compile-time constants
    // ═══════════════════════════════════════════════════════════════════════
    // LOG_PER_WHALE removed — now controlled at runtime via settings key:
    //   Group3.WOAFogMovement.logPerWhale = true/false   (default: false)
    // enableLogging also controlled via:
    //   Group3.WOAFogMovement.enableLogging = true/false (default: true)

    /** Shared column separator in log lines for easy visual parsing. */
    private static final String SEP = " | ";

    // ═══════════════════════════════════════════════════════════════════════
    //  SECTION 2 — Singleton Logger
    //  Controlled at runtime by enableLogging / logPerWhale settings.
    //  Toggle logging without recompiling — just change the settings file.
    // ═══════════════════════════════════════════════════════════════════════
    /**
     * Singleton logger shared by all WOAFogMovement instances.
     *
     * Runtime flags (set once from settings in constructor):
     *   enabled  — when false, ALL [WOA-*] output is suppressed (log file
     *              still created but stays empty). Default: true.
     *   perWhale — when true, one extra line per whale per iteration is
     *              written (very verbose). Default: false.
     *
     * Settings keys:
     *   Group3.WOAFogMovement.enableLogging = true   # master on/off switch
     *   Group3.WOAFogMovement.logPerWhale   = false  # per-whale verbosity
     *
     * Grep cheat-sheet:
     *   grep "[WOA-SCAN]"    woa_fog_movement.log   # drone visibility
     *   grep "[WOA-RESULT]"  woa_fog_movement.log   # fog movement decisions
     *   grep "OUT-OF-RANGE"  woa_fog_movement.log   # missed high-prio drones
     *   grep "WARNING"       woa_fog_movement.log   # all anomaly warnings
     */
    private static final class WOALogger {

        private static final WOALogger INSTANCE = new WOALogger();
        private static final DateTimeFormatter WALL_FMT =
                DateTimeFormatter.ofPattern("HH:mm:ss.SSS");

        /**
         * Master on/off switch — set via settings key enableLogging.
         * volatile so it is visible across threads immediately.
         */
        static volatile boolean enabled  = true;

        /**
         * Per-whale verbosity toggle — set via settings key logPerWhale.
         * WARNING: 20 whales x 100 iterations = 2000 extra lines per cycle.
         */
        static volatile boolean perWhale = false;

        // Non-final so open() can (re)assign it from the settings-driven path.
        private PrintWriter writer = null;

        /** Only registers the shutdown hook. File is opened lazily via open(). */
        private WOALogger() {
            Runtime.getRuntime().addShutdownHook(new Thread(() -> {
                if (writer != null) {
                    writer.println("# Log closed at " + LocalDateTime.now());
                    writer.flush();
                    writer.close();
                }
            }));
        }

        static WOALogger get() { return INSTANCE; }

        /**
         * Opens (or re-opens) the log file at the given path.
         * Called once from WOAFogMovement(Settings) before any LOG() calls.
         * If already open, the previous writer is closed first.
         *
         * @param filePath path read from settings key WOAFogMovement.logFile
         */
        void open(String filePath) {
            if (writer != null) {
                writer.println("# Log re-opened — redirecting to: " + filePath);
                writer.flush();
                writer.close();
                writer = null;
            }
            try {
                // Explicitly delete the existing log file (if any) so the new
                // run always starts with a completely fresh, empty file.
                Files.deleteIfExists(Paths.get(filePath));
                writer = new PrintWriter(new FileWriter(filePath, false), true);
                writer.println("# WOAFogMovement log — started " + LocalDateTime.now());
                writer.println("# Log file  : " + filePath);
                writer.println("# Format    : [WALL HH:mm:ss.SSS | SIM t=<simTime>] <tag> <message>");
                writer.println("# grep tags : [WOA-DIAG] [WOA-SCAN] [WOA-INIT] [WOA-ITER]");
                writer.println("#             [WOA-WHALE] [WOA-EVAL] [WOA-RESULT] [WOA-PATH] [WOA-PRIO]");
                writer.println("#");
                System.out.println("[WOAFogMovement] Log opened: " + filePath);
            } catch (IOException e) {
                System.err.println("[WOAFogMovement] FATAL: cannot open log file '"
                        + filePath + "': " + e.getMessage());
            }
        }

        /** Write one timestamped log line. No-op when enabled=false or file not opened. */
        void log(String message) {
            if (!enabled || writer == null) return;
            String wall = LocalDateTime.now().format(WALL_FMT);
            double sim  = SimClock.getTime();
            writer.println("[" + wall + " | SIM t=" + String.format("%10.4f", sim) + "] " + message);
        }

        /** Write a WARNING line; always echoed to stderr even when enabled=false. */
        void warn(String message) {
            System.err.println(message);   // always visible
            log(message);                  // silenced when enabled=false
        }
    }

    /** Write a normal log line (no-op when enableLogging=false). */
    private static void LOG(String msg)  { WOALogger.get().log(msg);  }
    /** Write a WARNING — always echoed to stderr. */
    private static void WARN(String msg) { WOALogger.get().warn(msg); }

    // ═══════════════════════════════════════════════════════════════════════
    //  SECTION 3 — Settings Keys
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

    // ── Logging toggles (NEW) ─────────────────────────────────────────────
    /** Master logging on/off switch. Default: true. */
    public static final String ENABLE_LOGGING_S    = "enableLogging";
    /** Enable per-whale verbose output. Default: false. */
    public static final String LOG_PER_WHALE_S     = "logPerWhale";
    /**
     * Path (relative or absolute) of the WOA log file.
     * Default: "woa_fog_movement.log" (written next to the simulator launch dir).
     * Example:
     *   Group3.WOAFogMovement.logFile = reports/woa_fog_movement.log
     */
    public static final String LOG_FILE_S          = "logFile";

    // ── Random Shutdown parameters (from MILP reference) ─────────────────
    /** Enable random drone shutdown event. */
    public static final String ENABLE_SHUTDOWN_S   = "enableRandomShutdown";
    /** Sim time at which to trigger the shutdown. */
    public static final String SHUTDOWN_TIME_S     = "shutdownTime";

    private static final int DRONE_GROUP_ADDR_UNKNOWN = -1;

    // ═══════════════════════════════════════════════════════════════════════
    //  SECTION 4 — Configuration Fields
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
    //  SECTION 5 — Runtime State Fields
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

        // ── Logging must be configured FIRST so every subsequent LOG() call
        //    writes to the correct file with the correct verbosity. ────────────
        String  logFile   = s.getSetting(WOA_FOG_NS + LOG_FILE_S,
                                         "woa_fog_movement.log");
        boolean enableLog = s.getBoolean(WOA_FOG_NS + ENABLE_LOGGING_S, true);
        boolean perWhale  = s.getBoolean(WOA_FOG_NS + LOG_PER_WHALE_S,  false);
        WOALogger.enabled  = enableLog;
        WOALogger.perWhale = perWhale;
        WOALogger.get().open(logFile);   // opens the file at the settings-driven path

        LOG("\n[WOA-DIAG] ════════════════════════════════════════════");
        LOG("[WOA-DIAG] WOAFogMovement constructor — loading settings");
        LOG("[WOA-DIAG]   logFile             = " + logFile);
        LOG("[WOA-DIAG]   enableLogging       = " + enableLog
                + "  (set " + WOA_FOG_NS + LOG_FILE_S + "=false to silence output)");
        LOG("[WOA-DIAG]   logPerWhale         = " + perWhale);
        LOG("[WOA-DIAG] ════════════════════════════════════════════");

        int[] coords = s.getCsvInts(WOA_FOG_NS + START_LOCATION_S, 2);
        this.startLoc = new Coord(coords[0], coords[1]);
        LOG("[WOA-DIAG]   startLocation       = " + startLoc);

        this.obstacleFilePath   = s.getSetting(WOA_FOG_NS + OBSTACLE_FILE_S);
        LOG("[WOA-DIAG]   obstacleFile        = " + obstacleFilePath);

        this.updateInterval     = s.getDouble(WOA_FOG_NS + UPDATE_INTERVAL_S,   15.0);
        this.communicationRange = s.getDouble(WOA_FOG_NS + COMM_RANGE_S,        500.0);
        this.populationSize     = s.getInt   (WOA_FOG_NS + POPULATION_SIZE_S,   20);
        this.maxIterations      = s.getInt   (WOA_FOG_NS + MAX_ITERATIONS_S,    100);
        this.spiralB            = s.getDouble(WOA_FOG_NS + SPIRAL_B_S,          1.0);
        this.targetMoveThreshold= s.getDouble(WOA_FOG_NS + TARGET_THRESHOLD_S,  10.0);

        LOG("[WOA-DIAG]   updateInterval      = " + updateInterval + "s");
        LOG("[WOA-DIAG]   communicationRange  = " + communicationRange + "m");
        LOG("[WOA-DIAG]   populationSize      = " + populationSize + " whales");
        LOG("[WOA-DIAG]   maxIterations       = " + maxIterations);
        LOG("[WOA-DIAG]   spiralB             = " + spiralB);
        LOG("[WOA-DIAG]   targetMoveThreshold = " + targetMoveThreshold + "m");

        // Read alpha / beta and normalise so they always sum to exactly 1.0
        double rawAlpha = s.getDouble(WOA_FOG_NS + COVERAGE_WEIGHT_S, 0.5);
        double rawBeta  = s.getDouble(WOA_FOG_NS + PRIORITY_WEIGHT_S,  0.5);
        double sum      = rawAlpha + rawBeta;

        if (Math.abs(sum - 1.0) > 1e-6) {
            WARN("[WOA-DIAG] WARNING: coverageWeight (" + rawAlpha
                    + ") + priorityWeight (" + rawBeta + ") = " + sum
                    + " != 1.0 — auto-normalising to sum=1.0");
            rawAlpha /= sum;
            rawBeta  /= sum;
            WARN("[WOA-DIAG]   After normalisation: alpha="
                    + rawAlpha + "  beta=" + rawBeta);
        }
        this.coverageWeight = rawAlpha;
        this.priorityWeight = rawBeta;
        LOG("[WOA-DIAG]   coverageWeight alpha=" + coverageWeight
                + "  (pure-coverage term NCV2/m gets this weight)");
        LOG("[WOA-DIAG]   priorityWeight beta =" + priorityWeight
                + "  (priority term PScore/P_total gets this weight)");

        // Parse per-drone priorities — owned entirely by WOAFogMovement
        this.dronePriorityMap = parseDronePriorities(s);

        this.enableShutdown = s.getBoolean(WOA_FOG_NS + ENABLE_SHUTDOWN_S, false);
        this.shutdownTime   = s.getDouble(WOA_FOG_NS + SHUTDOWN_TIME_S,   -1.0);
        LOG("[WOA-DIAG]   enableShutdown      = " + enableShutdown);
        LOG("[WOA-DIAG]   shutdownTime        = " + shutdownTime + "s");

        LOG("[WOA-DIAG] ════════════════════════════════════════════\n");

        this.gdrrt = new GDRRTPlanner(this.obstacleFilePath);
    }

    public WOAFogMovement(WOAFogMovement proto) {
        super(proto);
        this.startLoc             = proto.startLoc.clone();
        this.obstacleFilePath     = proto.obstacleFilePath;
        this.updateInterval       = proto.updateInterval;
        this.communicationRange   = proto.communicationRange;
        this.populationSize       = proto.populationSize;
        this.maxIterations        = proto.maxIterations;
        this.spiralB              = proto.spiralB;
        this.targetMoveThreshold  = proto.targetMoveThreshold;
        this.coverageWeight       = proto.coverageWeight;
        this.priorityWeight       = proto.priorityWeight;
        this.dronePriorityMap     = proto.dronePriorityMap; // immutable — safe to share
        this.enableShutdown       = proto.enableShutdown;
        this.shutdownTime         = proto.shutdownTime;
        this.shutdownTriggered    = proto.shutdownTriggered;
        this.gdrrt = new GDRRTPlanner(this.obstacleFilePath);
    }

    @Override
    public MovementModel replicate() {
        return new WOAFogMovement(this);
    }

    // =======================================================================
    //  ONE Simulator MovementModel interface
    // =======================================================================

    @Override
    public Coord getInitialLocation() {
        LOG("[WOA-DIAG] getInitialLocation() -> " + startLoc);
        return startLoc.clone();
    }

    /**
     * Returns when getPath() should next be called by the simulator.
     * During a collision-avoidance wait we retry every 1 simulated second.
     * Otherwise we wait until the next WOA update interval is due.
     */
    @Override
    public double nextPathAvailable() {
        if (isWaiting) {
            // Collision-avoidance hold: keep retrying every second until
            // DronePathManager grants us the path.
            double retry = SimClock.getTime() + 1.0;
            LOG("[WOA-PATH] t=" + fmt(SimClock.getTime())
                    + " isWaiting=true -> retrying in 1s"
                    + " (waiting for DronePathManager to clear collision)");
            return retry;
        }
        if (lastUpdateTime >= 0) {
            double next = lastUpdateTime + updateInterval;
            LOG("[WOA-PATH] t=" + fmt(SimClock.getTime())
                    + " next WOA cycle due at t=" + fmt(next)
                    + " (lastUpdate=" + fmt(lastUpdateTime)
                    + " + interval=" + updateInterval + "s)");
            return next;
        }
        // First ever call: run immediately
        LOG("[WOA-PATH] t=" + fmt(SimClock.getTime())
                + " first call -> nextPathAvailable=0 (immediate)");
        return 0;
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

        LOG("\n[WOA-PATH] ── getPath() t=" + fmt(now)
                + "  fogPos=" + fmtCoord(fogPos)
                + "  currentTarget="
                + (currentOptimalTarget == null ? "NONE"
                   : fmtCoord(currentOptimalTarget)));

        // ------------------------------------------------------------------
        // 1. Re-run WOA when the update interval has elapsed
        // ------------------------------------------------------------------
        boolean firstRun       = (lastUpdateTime < 0);
        boolean intervalElapsed= !firstRun && (now - lastUpdateTime) >= updateInterval;

        if (firstRun || intervalElapsed) {
            LOG("[WOA-PATH] WOA trigger: "
                    + (firstRun ? "first run"
                       : "elapsed=" + fmt(now - lastUpdateTime)
                         + "s >= interval=" + updateInterval + "s")
                    + " -> launching WOA cycle #" + (woaCycleCount + 1));

            Coord newTarget = runWOA();

            if (newTarget != null) {
                boolean noCurrentTarget = (currentOptimalTarget == null);
                double  shift = noCurrentTarget ? Double.NaN
                        : newTarget.distance(currentOptimalTarget);
                boolean targetChanged = noCurrentTarget
                        || shift > targetMoveThreshold;

                LOG("[WOA-PATH] WOA output: " + fmtCoord(newTarget));

                if (targetChanged) {
                    LOG("[WOA-PATH] Target CHANGED -> updating GDRRT"
                            + (noCurrentTarget
                               ? " (no previous target)"
                               : " shift=" + fmt(shift) + "m > threshold="
                                 + targetMoveThreshold + "m"));
                    currentOptimalTarget = newTarget;
                    if (getHost() != null) {
                        gdrrt.init(getHost().getLocation(), currentOptimalTarget);
                        LOG("[WOA-PATH] GDRRT re-initialised:"
                                + " from=" + fmtCoord(getHost().getLocation())
                                + " to=" + fmtCoord(currentOptimalTarget));
                    }
                } else {
                    LOG("[WOA-PATH] Target UNCHANGED (shift="
                            + fmt(shift) + "m <= threshold="
                            + targetMoveThreshold + "m)"
                            + " -> GDRRT plan kept as-is");
                }
            } else {
                LOG("[WOA-PATH] WOA returned null"
                        + " (no drones in range) -> fog stays put");
            }
            lastUpdateTime = now;

        } else {
            double remaining = updateInterval - (now - lastUpdateTime);
            LOG("[WOA-PATH] No WOA trigger yet"
                    + " (elapsed=" + fmt(now - lastUpdateTime)
                    + "s, next in=" + fmt(remaining) + "s)"
                    + " -> following existing GDRRT path");
        }

        // ------------------------------------------------------------------
        // 2. Nothing to navigate toward yet
        // ------------------------------------------------------------------
        if (currentOptimalTarget == null) {
            LOG("[WOA-PATH] No target set -> returning null path");
            return null;
        }

        // ------------------------------------------------------------------
        // 3. Ensure GDRRT is initialised
        // ------------------------------------------------------------------
        if (!gdrrt.isInitialized() && getHost() != null) {
            LOG("[WOA-PATH] GDRRT not yet initialised -> init now");
            gdrrt.init(getHost().getLocation(), currentOptimalTarget);
        }
        if (!gdrrt.isInitialized()) {
            LOG("[WOA-PATH] GDRRT still not initialised"
                    + " (host=null?) -> returning null path");
            return null;
        }

        // ------------------------------------------------------------------
        // 4. Plan next GDRRT segment
        // ------------------------------------------------------------------
        GDRRTPlanner.PlannedSegment proposed = gdrrt.planNextSegment();
        if (proposed == null) {
            LOG("[WOA-PATH] GDRRT planNextSegment()=null"
                    + " -> fog UAV has arrived at target "
                    + fmtCoord(currentOptimalTarget)
                    + ". Setting stationary.");
            DronePathManager.setStationary(getHost().getAddress());
            return null;
        }
        LOG("[WOA-PATH] GDRRT proposed segment"
                + " isFinalPath=" + proposed.isFinalPath);

        // ------------------------------------------------------------------
        // 5. Collision-avoidance handshake with DronePathManager
        // ------------------------------------------------------------------
        if (DronePathManager.requestPath(getHost().getAddress(), proposed.path)) {
            isWaiting = false;
            gdrrt.commit(proposed);
            LOG("[WOA-PATH] Path APPROVED by DronePathManager"
                    + (proposed.isFinalPath ? " (final segment — now stationary)" : ""));
            if (proposed.isFinalPath) {
                DronePathManager.setStationary(getHost().getAddress());
            }
            return proposed.path;
        } else {
            isWaiting = true;
            DronePathManager.setStationary(getHost().getAddress());
            LOG("[WOA-PATH] Path REJECTED by DronePathManager"
                    + " (collision avoidance) -> holding position, will retry in 1s");
            Path hold = new Path(0);
            hold.addWaypoint(getHost().getLocation());
            return hold;
        }
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

        LOG("\n[WOA-DIAG] ╔══════════════════════════════════════════════");
        LOG("[WOA-DIAG] ║ WOA CYCLE #" + woaCycleCount
                + "  t=" + fmt(now)
                + "  fogPos=" + fmtCoord(fogHost != null
                        ? fogHost.getLocation() : startLoc));
        LOG("[WOA-DIAG] ╚══════════════════════════════════════════════");

        if (fogHost == null) {
            LOG("[WOA-DIAG] fogHost is null -> aborting cycle");
            return null;
        }

        // ── OPTION B: collect only in-range drones ────────────────────────
        LOG("[WOA-DIAG] OPTION B: only drones within "
                + communicationRange + "m of current fog position matter.");
        LOG("[WOA-DIAG] Drones outside this radius are invisible"
                + " — their priorities have zero effect this cycle.");

        List<DroneInfo> droneInfos = collectActiveDroneInfo(fogHost);
        int    m      = droneInfos.size();
        double pTotal = computePTotal(droneInfos);

        if (m == 0) {
            LOG("[WOA-DIAG] No active drones visible in range."
                    + " Fog stays at current position. No WOA search performed.");
            return fogHost.getLocation();
        }

        LOG("[WOA-DIAG] WOA will optimise over m=" + m
                + " visible drones with P_total=" + fmt(pTotal));
        LOG("[WOA-DIAG] Drone snapshot for this full cycle:");
        for (int i = 0; i < droneInfos.size(); i++) {
            DroneInfo d = droneInfos.get(i);
            double share = (pTotal > 0) ? d.priority / pTotal * 100 : 0;
            LOG("[WOA-DIAG]   [" + i + "] " + d
                    + "  priority-share=" + fmt(share) + "%");
        }

        initWorldBounds();
        // Search bounds: keep fog away from world edges by one comm-range
        double margin = communicationRange;
        double minX   = margin;
        double minY   = margin;
        double maxX   = Math.max(minX + 1, worldWidth  - margin);
        double maxY   = Math.max(minY + 1, worldHeight - margin);
        LOG("[WOA-DIAG] Search space bounds:"
                + " X=[" + fmt(minX) + ", " + fmt(maxX) + "]"
                + " Y=[" + fmt(minY) + ", " + fmt(maxY) + "]"
                + " (margin=" + fmt(margin) + "m from world edges)");

        // ── Initialise population ─────────────────────────────────────────
        LOG("\n[WOA-INIT] Spawning " + populationSize
                + " random whale positions across the search space...");

        List<Whale> population = new ArrayList<>(populationSize);
        double initBestFit  = Double.NEGATIVE_INFINITY;
        double initWorstFit = Double.POSITIVE_INFINITY;

        for (int i = 0; i < populationSize; i++) {
            double wx = minX + rng.nextDouble() * (maxX - minX);
            double wy = minY + rng.nextDouble() * (maxY - minY);
            Whale  w  = new Whale(wx, wy);
            w.fitness = evaluate(w.toCoord(), droneInfos, m, pTotal, false);
            population.add(w);
            if (w.fitness > initBestFit)  initBestFit  = w.fitness;
            if (w.fitness < initWorstFit) initWorstFit = w.fitness;
            if (WOALogger.perWhale) {
                LOG("[WOA-INIT]   whale[" + i + "] " + w);
            }
        }

        // FIX-SEED: Always place whale[0] at the fog's current position.
        // Guarantees a non-zero seed at t=0 (before drones have dispersed)
        // and anchors future cycles near the fog's existing location.
        {
            Whale seed = population.get(0);
            seed.x = fogHost.getLocation().getX();
            seed.y = fogHost.getLocation().getY();
            double seedFit = evaluate(seed.toCoord(), droneInfos, m, pTotal, false);
            if (seedFit > initBestFit) initBestFit = seedFit;
            if (seedFit < initWorstFit) initWorstFit = seedFit;
            seed.fitness = seedFit;
            LOG("[WOA-INIT] FIX-SEED: whale[0] forced to fogPos="
                    + fmtCoord(seed.toCoord()) + " fitness=" + fmt(seedFit));
        }

        Whale best = findBest(population).copy();
        LOG("[WOA-INIT] Initial population fitness range:"
                + " worst=" + fmt(initWorstFit)
                + " best=" + fmt(initBestFit));
        LOG("[WOA-INIT] Initial best whale: " + best);
        LOG("[WOA-INIT] Note: a fitness=0 whale is one whose"
                + " candidate position is isolated (not in G*) — the fog would"
                + " become a connectivity island there.");

        // ── WOA main loop — Algorithm 1 from paper ───────────────────────
        LOG("\n[WOA-ITER] Starting WOA main loop"
                + " (" + maxIterations + " iterations"
                + ", population=" + populationSize + " whales)...");
        LOG("[WOA-ITER] 'a' decreases linearly 2->0 over iterations,"
                + " shifting behaviour from exploration to exploitation.");

        // FIX-3: divisor = maxIterations-1 so 'a' reaches exactly 0 on last step
        double aDivisor = (maxIterations > 1) ? (maxIterations - 1) : 1;

        // Log a checkpoint every ~10% of iterations
        int logEvery = Math.max(1, maxIterations / 10);

        // Counters to explain which WOA mechanism dominated
        int shrinkCount      = 0;
        int exploreCount     = 0;
        int spiralCount      = 0;
        int clampCount       = 0;
        int improvementCount = 0;

        for (int t = 0; t < maxIterations; t++) {

            // 'a' decreases linearly from 2 to 0 (controls exploration/exploitation)
            double a = 2.0 - 2.0 * t / aDivisor;

            for (Whale whale : population) {
                double r1 = rng.nextDouble();
                double r2 = rng.nextDouble();
                double A  = 2.0 * a * r1 - a;          // Eq. 24
                double C  = 2.0 * r2;                   // Eq. 25
                double p  = rng.nextDouble();
                double l  = rng.nextDouble() * 2.0 - 1.0; // [-1, 1] per paper Eq. 26

                double oldX = whale.x;
                double oldY = whale.y;
                String mode;

                if (p < 0.5) {
                    if (Math.abs(A) < 1.0) {
                        // ── Shrinking Encircling (Eqs. 22-23) ─────────────
                        // Exploitation: move toward best, shrinking distance
                        double Dx = Math.abs(C * best.x - whale.x);
                        double Dy = Math.abs(C * best.y - whale.y);
                        whale.x   = best.x - A * Dx;
                        whale.y   = best.y - A * Dy;
                        mode = "SHRINK-ENCIRCLE (|A|=" + fmt(Math.abs(A))
                                + "<1, exploiting best @ " + fmtCoord(best.toCoord()) + ")";
                        shrinkCount++;
                    } else {
                        // ── Random exploration (Eqs. 28-29) ───────────────
                        // Exploration: move toward a random whale to avoid local optima
                        Whale rand = population.get(rng.nextInt(populationSize));
                        double Dx  = Math.abs(C * rand.x - whale.x);
                        double Dy  = Math.abs(C * rand.y - whale.y);
                        whale.x    = rand.x - A * Dx;
                        whale.y    = rand.y - A * Dy;
                        mode = "RANDOM-EXPLORE (|A|=" + fmt(Math.abs(A))
                                + ">=1, rand @ " + fmtCoord(rand.toCoord()) + ")";
                        exploreCount++;
                    }
                } else {
                    // ── Spiral Updating Position (Eq. 26) — FIX-2 ────────
                    // Applies the spiral scalar per-dimension (signed distance)
                    // so the whale orbits around the current best.
                    double Dx  = best.x - whale.x;   // signed x-distance to best
                    double Dy  = best.y - whale.y;   // signed y-distance to best
                    double ebl = Math.exp(spiralB * l) * Math.cos(2.0 * Math.PI * l);
                    whale.x = Dx * ebl + best.x;
                    whale.y = Dy * ebl + best.y;
                    mode = "SPIRAL (l=" + fmt(l) + " ebl=" + fmt(ebl) + ")";
                    spiralCount++;
                }

                // Clamp to search space (Algorithm 1, line 20)
                boolean clamped = whale.x < minX || whale.x > maxX
                                || whale.y < minY || whale.y > maxY;
                if (clamped) {
                    whale.x = Math.max(minX, Math.min(maxX, whale.x));
                    whale.y = Math.max(minY, Math.min(maxY, whale.y));
                    clampCount++;
                }

                whale.fitness = evaluate(whale.toCoord(), droneInfos,
                        m, pTotal, false);

                if (WOALogger.perWhale) {
                    LOG("[WOA-WHALE] iter=" + t
                            + " mode=" + mode
                            + " (" + fmt(oldX) + "," + fmt(oldY) + ")"
                            + " -> (" + fmt(whale.x) + "," + fmt(whale.y) + ")"
                            + (clamped ? " [CLAMPED-to-bounds]" : "")
                            + " fitness=" + fmt(whale.fitness));
                }
            }

            // Update global best
            Whale candidate = findBest(population);
            if (candidate.fitness > best.fitness) {
                double delta = candidate.fitness - best.fitness;
                best = candidate.copy();
                improvementCount++;
                if (WOALogger.perWhale) {
                    LOG("[WOA-ITER] iter=" + t
                            + " NEW BEST: " + best
                            + " (improved by +" + fmt(delta) + ")");
                }
            }

            // Print a checkpoint ~10 times total across the run
            if (t % logEvery == 0 || t == maxIterations - 1) {
                LOG("[WOA-ITER] iter=" + padInt(t, 4)
                        + "/" + maxIterations
                        + SEP + "a=" + fmt(a)
                        + SEP + "bestFit=" + fmt(best.fitness)
                        + SEP + "bestPos=" + fmtCoord(best.toCoord())
                        + SEP + "shrink=" + shrinkCount
                        + " explore=" + exploreCount
                        + " spiral=" + spiralCount
                        + " clamped=" + clampCount
                        + " improvements=" + improvementCount);
            }
        }

        // ── Final result ──────────────────────────────────────────────────
        LOG("\n[WOA-RESULT] ╔─── Cycle #" + woaCycleCount
                + " complete ───────────────────────────");
        LOG("[WOA-RESULT] ║ Best candidate  : " + fmtCoord(best.toCoord()));
        LOG("[WOA-RESULT] ║ Best fitness    : " + fmt(best.fitness)
                + " / 1.0");
        LOG("[WOA-RESULT] ║ Move breakdown  : shrink=" + shrinkCount
                + " explore=" + exploreCount + " spiral=" + spiralCount
                + " clamped=" + clampCount);
        LOG("[WOA-RESULT] ║ Best updated    : " + improvementCount
                + " time(s) during this cycle");
        LOG("[WOA-RESULT] ║ Travel needed   : "
                + fmt(fogHost.getLocation().distance(best.toCoord())) + "m");

        // Detailed evaluation of the winner — explains WHY this position won
        LOG("[WOA-RESULT] ║ Detailed fitness breakdown for winner:");
        evaluate(best.toCoord(), droneInfos, m, pTotal, true);

        LOG("[WOA-RESULT] ╚────────────────────────────────────────────\n");

        return best.toCoord();
    }

    /**
     * Randomly shuts down an active mission drone and reassigns its target
     * to a lower-priority drone within communicationRange.
     * Based on behavior from MILPClusteredFogMovement.
     */
    private void triggerRandomShutdown() {
        shutdownTriggered = true;
        LOG("\n[WOA-EVENT] >>> CRITICAL EVENT: Random Drone Shutdown Triggered at t=" + fmt(SimClock.getTime()) + " <<<");

        List<DTNHost> activeDrones = new ArrayList<>();
        for (DTNHost host : SimScenario.getInstance().getHosts()) {
            MovementModel mm = host.getMovement();
            if (mm instanceof GDRRTMovement) {
                GDRRTMovement gmm = (GDRRTMovement) mm;
                if (!gmm.isDone() && !gmm.isDead()) {
                    activeDrones.add(host);
                }
            }
        }

        if (activeDrones.isEmpty()) {
            LOG("[WOA-EVENT]   Shutdown aborted: no active mission drones found.");
            return;
        }

        // 1. Pick a random active drone to shut down
        Collections.shuffle(activeDrones, new Random((long) SimClock.getTime()));
        DTNHost killedDrone = activeDrones.get(0);
        GDRRTMovement killedMm = (GDRRTMovement) killedDrone.getMovement();

        double killedPriority = killedMm.getPriority();
        Coord killedTarget = killedMm.getEndLocation();

        LOG("[WOA-EVENT] >>> DRONE FAILURE: " + killedDrone.getName() + " (prio=" + fmt(killedPriority) + ") has been killed! <<<");
        killedMm.kill();

        // 2. Scan for a lower priority replacement drone in range
        DTNHost replacement = null;
        double bestPriorityFound = Double.MAX_VALUE;
        Coord fogPos = getHost().getLocation();

        for (DTNHost host : activeDrones) {
            if (host == killedDrone) continue;
            GDRRTMovement mm = (GDRRTMovement) host.getMovement();
            double priority = mm.getPriority();

            // Criteria: not done, not dead, lower priority than killed drone, AND within comm range
            if (!mm.isDone() && !mm.isDead() && priority < killedPriority) {
                double dist = host.getLocation().distance(fogPos);
                if (dist <= communicationRange) {
                    // Pick the lowest priority out of the available lower-priority candidates
                    if (priority < bestPriorityFound) {
                        bestPriorityFound = priority;
                        replacement = host;
                    }
                }
            }
        }

        // 3. Reassign if a suitable drone was found
        if (replacement != null) {
            LOG("[WOA-EVENT] >>> FOG ACTION: Reassigning target " + fmtCoord(killedTarget)
                    + " to lower priority in-range Drone " + replacement.getName()
                    + " (previous prio=" + fmt(((GDRRTMovement)replacement.getMovement()).getPriority())
                    + " -> new prio=" + fmt(killedPriority) + ") <<<");
            ((GDRRTMovement) replacement.getMovement()).changeTarget(killedTarget, killedPriority);
        } else {
            LOG("[WOA-EVENT] >>> FOG ACTION: Drone " + killedDrone.getName()
                    + " failed, but NO lower priority drone is in range ("
                    + fmt(communicationRange) + "m) for reassignment. <<<");
        }
        LOG("[WOA-EVENT] ─────────────────────────────────────────────────────────\n");
    }

    // =======================================================================
    //  Objective function
    //  H(X) = alpha * NCV2(G*)/m  +  beta * PriorityScore(G*)/P_total
    // =======================================================================

    /**
     * Evaluates the blended objective for one candidate fog vehicle position.
     *
     * Step-by-step:
     *   1. Build adjacency between fogPos and all snapshot drones.
     *   2. BFS to find connected components; G* = largest (Eq. 4).
     *   3. If fogPos not in G* -> return 0 (gateway connectivity rule).
     *   4. Count covered drones (within R of fogPos) and sum their priorities.
     *   5. Return alpha*NCV2/m + beta*PScore/P_total.
     *
     * @param fogPos     candidate fog UAV position
     * @param droneInfos in-range snapshot (Option B — fixed for the cycle)
     * @param m          stable drone count denominator
     * @param pTotal     stable priority sum denominator
     * @param logDetail  if true, print a full human-readable breakdown
     * @return fitness in [0, 1]
     */
    private double evaluate(Coord fogPos,
                            List<DroneInfo> droneInfos,
                            int m,
                            double pTotal,
                            boolean logDetail) {

        int n      = droneInfos.size();
        int fogIdx = n;       // fog UAV is node index n in the adjacency matrix
        int total  = n + 1;   // n drones + 1 fog node

        // ── Step 1: Build adjacency ───────────────────────────────────────
        boolean[] adj = new boolean[total * total];

        // Drone-drone edges (these don't depend on fogPos)
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                double d = droneInfos.get(i).location
                        .distance(droneInfos.get(j).location);
                if (d <= communicationRange) {
                    adj[i * total + j] = true;
                    adj[j * total + i] = true;
                }
            }
        }
        // Fog-drone edges (these DO depend on fogPos — the variable we optimise)
        for (int i = 0; i < n; i++) {
            double d = fogPos.distance(droneInfos.get(i).location);
            if (d <= communicationRange) {
                adj[fogIdx * total + i] = true;
                adj[i * total + fogIdx] = true;
            }
        }

        // ── Step 2: BFS — find connected components ───────────────────────
        boolean[] visited  = new boolean[total];
        int[]     compId   = new int[total];
        int[]     compSize = new int[total];
        int       numComps = 0;

        for (int start = 0; start < total; start++) {
            if (visited[start]) continue;
            Queue<Integer> queue = new LinkedList<>();
            queue.add(start);
            visited[start] = true;
            int size = 0;
            while (!queue.isEmpty()) {
                int cur = queue.poll();
                compId[cur] = numComps;
                size++;
                for (int nb = 0; nb < total; nb++) {
                    if (!visited[nb] && adj[cur * total + nb]) {
                        visited[nb] = true;
                        queue.add(nb);
                    }
                }
            }
            compSize[numComps] = size;
            numComps++;
        }

        // ── Step 3: Identify G* (largest component) — Eq. 4 ─────────────
        int gStarId = 0, gStarSize = 0;
        for (int c = 0; c < numComps; c++) {
            if (compSize[c] > gStarSize) {
                gStarSize = compSize[c];
                gStarId   = c;
            }
        }

        // Connectivity gateway check:
        // If fog is NOT in G*, it is isolated — fitness = 0.
        boolean fogInGStar = (compId[fogIdx] == gStarId);

        if (!fogInGStar) {
            if (logDetail) {
                LOG("[WOA-EVAL]   Candidate " + fmtCoord(fogPos)
                        + " -> fog NOT in G*");
                LOG("[WOA-EVAL]   Reason: fog is in component "
                        + compId[fogIdx] + " (size=" + compSize[compId[fogIdx]]
                        + ") but G* is component " + gStarId
                        + " (size=" + gStarSize + ")");
                LOG("[WOA-EVAL]   This candidate position would"
                        + " isolate the fog from the main drone sub-network.");
                LOG("[WOA-EVAL]   -> fitness = 0.0 (connectivity gateway rule)");
            }
            return 0.0;
        }

        // ── Step 4: Count covered drones and sum priorities ───────────────
        // FIX-4: per paper Eq. 7, drone i is covered iff:
        //   (a) fog is in G* (confirmed above), AND
        //   (b) distance(fogPos, drone_i) <= R
        // The drone itself does NOT need to be in G*.
        //
        // FIX-PROXIMITY: Priority score is now DISTANCE-WEIGHTED.
        // Old (binary): pScore += priority   -- any in-range position scores the same
        // New (weighted): pScore += priority * (1 + proximity)
        //   where proximity = 1 - dist/R  (1.0 when co-located, 0.0 at range edge)
        // This creates a real fitness gradient so WOA pulls toward high-priority drones
        // rather than treating every in-range position as equally fit.
        // P_total is also scaled by 2 (see computePTotal) to keep the term in [0,1].
        int    ncv2   = 0;
        double pScore = 0.0;

        List<String> coveredLines   = logDetail ? new ArrayList<>() : null;
        List<String> uncoveredLines = logDetail ? new ArrayList<>() : null;

        for (int i = 0; i < n; i++) {
            DroneInfo d    = droneInfos.get(i);
            double    dist = fogPos.distance(d.location);
            boolean covered = (dist <= communicationRange);

            if (covered) {
                ncv2++;
                // Proximity bonus: ranges from 0.0 (at range edge) to 1.0 (co-located)
                double proximity = 1.0 - (dist / communicationRange);
                double weightedPrio = d.priority * (1.0 + proximity);
                pScore += weightedPrio;
                if (logDetail) coveredLines.add(String.format(
                        "      D%-2d pos=%s dist=%.1fm prio=%.1f prox=%.3f weightedPrio=%.3f  ✓ covered",
                        d.droneIndex, fmtCoord(d.location), dist,
                        d.priority, proximity, weightedPrio));
            } else {
                if (logDetail) uncoveredLines.add(String.format(
                        "      D%-2d pos=%s dist=%.1fm > range=%.0fm  ✗ too far"
                        + " (was in snapshot but candidate too far from it)",
                        d.droneIndex, fmtCoord(d.location), dist,
                        communicationRange));
            }
        }

        // ── Step 5: Compute blended fitness ──────────────────────────────
        double coverageTerm = (m > 0)      ? (double) ncv2 / m : 0.0;
        double priorityTerm = (pTotal > 0) ? pScore / pTotal   : 0.0;
        double fitness      = coverageWeight * coverageTerm
                            + priorityWeight * priorityTerm;

        if (logDetail) {
            LOG("[WOA-EVAL]   Candidate: " + fmtCoord(fogPos));
            LOG("[WOA-EVAL]   Network:   " + numComps
                    + " component(s), G*=comp" + gStarId
                    + " size=" + gStarSize + "/" + total
                    + ", fog in G*=YES");
            LOG("[WOA-EVAL]   Covered drones (" + ncv2
                    + " of " + m + " in snapshot):");
            if (coveredLines.isEmpty()) {
                LOG("[WOA-EVAL]     (none)");
            } else {
                for (String l : coveredLines) LOG("[WOA-EVAL] " + l);
            }
            LOG("[WOA-EVAL]   Not covered by this candidate:");
            if (uncoveredLines.isEmpty()) {
                LOG("[WOA-EVAL]     (none — all snapshot drones within range)");
            } else {
                for (String l : uncoveredLines) LOG("[WOA-EVAL] " + l);
            }
            LOG("[WOA-EVAL]   NCV2          = " + ncv2);
            LOG("[WOA-EVAL]   m (Option B)  = " + m
                    + "  <- denominator = in-range drone count only");
            LOG("[WOA-EVAL]   PScore        = " + fmt(pScore));
            LOG("[WOA-EVAL]   P_total (B)   = " + fmt(pTotal)
                    + "  <- denominator = sum of in-range priorities only");
            LOG("[WOA-EVAL]   coverageTerm  = NCV2/m          = "
                    + ncv2 + "/" + m + " = " + fmt(coverageTerm));
            LOG("[WOA-EVAL]   priorityTerm  = PScore/P_total  = "
                    + fmt(pScore) + "/" + fmt(pTotal) + " = " + fmt(priorityTerm));
            LOG("[WOA-EVAL]   H(X) = " + coverageWeight
                    + " * " + fmt(coverageTerm)
                    + " + " + priorityWeight
                    + " * " + fmt(priorityTerm)
                    + " = " + fmt(coverageWeight * coverageTerm)
                    + " + " + fmt(priorityWeight * priorityTerm)
                    + " = " + fmt(fitness));
        }

        return fitness;
    }

    // =======================================================================
    //  Helpers
    // =======================================================================

    /**
     * OPTION B: Scans all simulator hosts and returns active mission drones
     * that are currently within communicationRange of the fog UAV.
     *
     * Every include/exclude decision is printed with a clear reason so you
     * can understand exactly why each drone is or isn't part of this cycle.
     */
    private List<DroneInfo> collectActiveDroneInfo(DTNHost fogHost) {
        List<DroneInfo> infos   = new ArrayList<>();
        Coord           fogPos  = fogHost.getLocation();

        LOG("\n[WOA-SCAN] ── Drone visibility scan ─────────────────");
        LOG("[WOA-SCAN] fogPos=" + fmtCoord(fogPos)
                + "  commRange=" + communicationRange + "m");
        LOG("[WOA-SCAN] OPTION B active: drones > "
                + communicationRange + "m from fog are invisible this cycle.");

        // ── Pass 1 (one-time): discover the drone group's base address ────
        // This lets us convert absolute host addresses to readable D0, D1...
        if (droneGroupBaseAddr == DRONE_GROUP_ADDR_UNKNOWN) {
            LOG("[WOA-SCAN] Pass 1 (one-time init):"
                    + " finding droneGroupBaseAddr...");
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
            if (droneGroupBaseAddr == DRONE_GROUP_ADDR_UNKNOWN) {
                LOG("[WOA-SCAN] WARNING: no mission drones found"
                        + " in Pass 1! droneGroupBaseAddr stays unknown.");
            } else {
                LOG("[WOA-SCAN] droneGroupBaseAddr=" + droneGroupBaseAddr
                        + " (drone D0 has this address; D1=addr+"
                        + (droneGroupBaseAddr+1) + ", etc.)");
            }
        }

        // ── Pass 2: evaluate every host ───────────────────────────────────
        int cntTotal      = 0;
        int cntFogSelf    = 0;
        int cntNonMission = 0;
        int cntDone       = 0;
        int cntOutRange   = 0;
        int cntIncluded   = 0;

        for (DTNHost host : SimScenario.getInstance().getHosts()) {
            cntTotal++;

            // Skip the fog UAV itself
            if (host == fogHost) {
                cntFogSelf++;
                continue;
            }

            MovementModel mm = host.getMovement();

            // Only accept exact mission-drone classes (FIX-7)
            boolean isMission =
                    (mm.getClass() == GDRRTMovement.class) ||
                    (mm.getClass() == FogDeployedGDRRTMovement.class);
            if (!isMission) {
                cntNonMission++;
                LOG("[WOA-SCAN]   SKIP host=addr:" + host.getAddress()
                        + " class=" + mm.getClass().getSimpleName()
                        + " -> not a mission drone (not GDRRTMovement"
                        + " or FogDeployedGDRRTMovement)");
                continue;
            }

            // FIX-CAST: use instanceof instead of a raw cast so this works
            // for both GDRRTMovement and FogDeployedGDRRTMovement without
            // risk of a ClassCastException if the hierarchy ever changes.
            int     dIdx = (droneGroupBaseAddr != DRONE_GROUP_ADDR_UNKNOWN)
                         ? host.getAddress() - droneGroupBaseAddr : -1;
            boolean done = (mm instanceof GDRRTMovement)
                        && ((GDRRTMovement) mm).isDone();

            // Skip drones that have completed their mission
            if (done) {
                cntDone++;
                LOG("[WOA-SCAN]   SKIP D" + dIdx
                        + " addr=" + host.getAddress()
                        + " -> isDone()=true (mission complete,"
                        + " drone is stationary at destination)");
                continue;
            }

            // OPTION B: range filter — the core local-knowledge assumption
            Coord  dronePos  = host.getLocation();
            double distToFog = fogPos.distance(dronePos);
            double priority  = getDronePriority(host);

            if (distToFog > communicationRange) {
                cntOutRange++;
                LOG("[WOA-SCAN]   OUT-OF-RANGE D" + dIdx
                        + " addr=" + host.getAddress()
                        + " pos=" + fmtCoord(dronePos)
                        + " dist=" + fmt(distToFog) + "m"
                        + " range=" + communicationRange + "m"
                        + " prio=" + fmt(priority)
                        + "  -> EXCLUDED");
                LOG("[WOA-SCAN]     *** Option B: D" + dIdx
                        + " is invisible. Its priority=" + fmt(priority)
                        + " has NO effect this cycle."
                        + " Fog must travel within " + communicationRange
                        + "m to see it. ***");
                continue;
            }

            // Drone passes all filters — include in this cycle's snapshot
            cntIncluded++;
            infos.add(new DroneInfo(dIdx, dronePos.clone(), priority, distToFog));
            LOG("[WOA-SCAN]   IN-RANGE  D" + dIdx
                    + " addr=" + host.getAddress()
                    + " pos=" + fmtCoord(dronePos)
                    + " dist=" + fmt(distToFog) + "m"
                    + " prio=" + fmt(priority)
                    + "  -> INCLUDED");
        }

        // ── Scan summary ──────────────────────────────────────────────────
        LOG("[WOA-SCAN] ── Scan summary ──────────────────────────");
        LOG("[WOA-SCAN]   Hosts scanned     : " + cntTotal);
        LOG("[WOA-SCAN]   Skipped (self)    : " + cntFogSelf);
        LOG("[WOA-SCAN]   Skipped (non-mission): " + cntNonMission);
        LOG("[WOA-SCAN]   Skipped (done)    : " + cntDone);
        LOG("[WOA-SCAN]   Excluded (out of range): " + cntOutRange
                + "  <-- OPTION B exclusions");
        LOG("[WOA-SCAN]   Included (visible): " + cntIncluded);

        if (cntOutRange > 0) {
            LOG("[WOA-SCAN] *** WARNING: " + cntOutRange
                    + " active mission drone(s) are out of range and"
                    + " therefore invisible to this WOA cycle."
                    + " Any high-priority drones in this group will NOT"
                    + " influence the fog UAV's positioning decision"
                    + " until the fog moves close enough to see them. ***");
        }
        if (cntIncluded == 0) {
            LOG("[WOA-SCAN] *** WARNING: zero drones in range."
                    + " Fog UAV has nothing to optimise for"
                    + " and will stay at current position. ***");
        }
        LOG("[WOA-SCAN] ─────────────────────────────────────────\n");

        return infos;
    }

    /**
     * Returns the priority weight for one mission drone host.
     * Converts absolute address to zero-based drone index using
     * droneGroupBaseAddr, then looks up dronePriorityMap.
     * Defaults to 1.0 if the drone has no explicit priority entry.
     */
    private double getDronePriority(DTNHost droneHost) {
        if (droneGroupBaseAddr == DRONE_GROUP_ADDR_UNKNOWN) {
            LOG("[WOA-PRIO] getDronePriority: base addr not yet"
                    + " known -> returning default 1.0");
            return 1.0;
        }
        int    idx      = droneHost.getAddress() - droneGroupBaseAddr;
        boolean hasEntry = dronePriorityMap.containsKey(idx);
        double  priority = dronePriorityMap.getOrDefault(idx, 1.0);
        if (WOALogger.perWhale) {
            LOG("[WOA-PRIO] D" + idx
                    + " addr=" + droneHost.getAddress()
                    + " priority=" + priority
                    + (hasEntry ? "" : " (DEFAULT — no entry in dronePriorities)"));
        }
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

        LOG("[WOA-PRIO] Looking for key: " + fullKey);

        if (!s.contains(fullKey)) {
            LOG("[WOA-PRIO] Key absent -> all drones default"
                    + " to priority 1.0 (equal weighting, pure coverage mode)");
            return map;
        }

        String raw = s.getRawSetting(fullKey);
        LOG("[WOA-PRIO] Raw value: " + raw);
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
            LOG("[WOA-PRIO]   D" + idx + " -> priority=" + w);
            idx++;
        }

        LOG("[WOA-PRIO] Loaded " + map.size() + " priorities.");
        LOG("[WOA-PRIO] IMPORTANT: under Option B, priorities only"
                + " affect drones that are WITHIN communicationRange of the fog UAV."
                + " A high-priority drone outside range is invisible this cycle.");
        return map;
    }

    /**
     * Sums the maximum possible weighted priority for all drones in the snapshot.
     * Because evaluate() now awards up to priority*(1+1)=2*priority per drone
     * (when the fog is co-located with a drone), P_total must reflect that
     * maximum so the priorityTerm = pScore/pTotal stays in [0, 1].
     */
    private double computePTotal(List<DroneInfo> droneInfos) {
        double sum = 0.0;
        // ×2 because the maximum weighted priority per drone is priority*(1+1)
        for (DroneInfo d : droneInfos) sum += d.priority * 2.0;
        LOG("[WOA-DIAG] P_total=" + fmt(sum)
                + " computed from " + droneInfos.size()
                + " in-range drones (Option B: in-range only)"
                + " [×2 for proximity-weighted max]");
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
            LOG("[WOA-DIAG] World bounds initialised:"
                    + " width=" + worldWidth + "  height=" + worldHeight);
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