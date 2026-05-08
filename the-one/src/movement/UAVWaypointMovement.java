package movement;

import core.Coord;
import core.Settings;
import core.SimClock;
import core.SimError;

import java.io.IOException;
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * ============================================================
 *  UAVWaypointMovement — Full Fusion Algorithm Implementation
 *  ONE Simulator Custom Movement Model
 * ============================================================
 *
 * Implements the double-layer path planning algorithm from:
 *
 *   He, Hou & Wang (2024). "A new method for unmanned aerial
 *   vehicle path planning in complex environments."
 *   Scientific Reports 14:9257.
 *
 * ── Algorithm layers ──────────────────────────────────────────
 *
 *  LAYER 1 — Improved A* (global planner)
 *    • Grid-based A* over MovementModel.worldSize at gridCellM resolution.
 *    • Adaptive heuristic: f(n) = g(n) + (1 + sigmoid(ξ)) * h(n)
 *      where ξ is the obstacle coverage rate in the bounding box
 *      between current node and goal (paper Eq. 7–9).
 *    • Neighbour clipping: straight/diagonal pruning rules that skip
 *      nodes reachable more cheaply without passing through the current
 *      node (paper Fig. 1).
 *    • Bresenham key-node extraction: line-of-sight checks compress the
 *      raw A* path to only the critical turning-point waypoints (paper Fig. 2).
 *
 *  LAYER 2 — Improved DWA (local planner)
 *    • Discrete velocity sampling in [speedMin, speedMax].
 *    • Evaluation function:
 *        G = α·heading + β·vel + λ·dist_obs + η·dist_fol + VO_penalty
 *      dist_fol pulls the local path toward the global A* path (paper Eq. 10).
 *    • Adaptive weights λ and η based on proximity to nearest obstacle
 *      (paper Table 4, steps a–d).
 *
 *  FUSION — Dynamic Re-planning (paper Fig. 3)
 *    Key nodes from Layer 1 become sequential DWA sub-goals.
 *    Bresenham LOS check triggers re-plan when a sub-goal is blocked.
 *    Stall counter forces re-plan after STALL_LIMIT idle steps.
 *    When A* finds no route a DWA escape step moves the drone away and
 *    schedules a retry.
 *
 * ── Inter-UAV Collision Avoidance (Velocity Obstacle) ─────────
 *
 *  Every UAV writes {x, y, vx, vy} to the shared PEER_REGISTRY at the
 *  end of each getPath() call.  During dwaStep() every candidate velocity
 *  is tested against each peer:
 *    1. Relative velocity v_rel = v_candidate − v_peer
 *    2. VO cone half-angle sin θ = uavSeparationM / dist(self, peer)
 *    3. If angle(v_rel, peer−self) < θ → inside cone → penalty applied
 *    4. Penetration depth and proximity scale the penalty.
 *    5. ID-based tiebreaker prevents symmetric head-on oscillation.
 *
 * ── Per-group configuration ────────────────────────────────────
 *
 *  ONE calls new UAVWaypointMovement(Settings s) once per GroupN block.
 *  The Settings object s is scoped to that group, so s.contains("spawn")
 *  is true only if GroupN.spawn is present in the settings file.
 *
 *  Every parameter is read via readDouble/readInt/readBoolean/readString
 *  which checks s (per-group) first, then falls back to the shared
 *  UAVWaypointMovement.* namespace (cfg).  This gives each drone its
 *  own spawn/target/speed while still inheriting shared defaults for
 *  any key not explicitly set in the group block.
 *
 *  Example — three drones with independent start/goal positions:
 *    UAVWaypointMovement.obstacleWktFile = samples/obstacles.wkt  ← shared
 *    UAVWaypointMovement.gridCellM       = 15                     ← shared
 *    Group1.spawn  = 80, 80     Group1.target  = 920, 920
 *    Group2.spawn  = 500, 80    Group2.target  = 500, 920
 *    Group3.spawn  = 200, 200   Group3.target  = 800, 800
 *
 * ── Obstacle model ────────────────────────────────────────────
 *
 *  WKT file (POINT / LINESTRING / MULTILINESTRING / MULTISTRING / POLYGON /
 *  MULTIPOLYGON / GEOMETRYCOLLECTION) rasterised onto the A* grid.
 *  Note: "MULTISTRING" is a non-standard alias for MULTILINESTRING emitted by
 *  some GIS exporters; it is accepted and handled identically.
 *  All group obstacle files are UNION-MERGED into one shared obstacleGrid
 *  so every drone sees the combined environment.
 *  Files are de-duplicated by canonical path via loadedWktFiles.
 *  UAVWaypointMovement.obstacleWktFile provides a shared baseline;
 *  GroupN.obstacleWktFile adds per-group extras.
 *  Multiple files may be provided as a single comma-separated value:
 *    UAVWaypointMovement.obstacleWktFile = a.wkt, b.wkt, c.wkt
 *  This works because readObstacleFilePaths() uses getCsvSetting() (which
 *  returns ALL tokens) rather than getSetting() (which returns only [0]).
 *
 *  !! IMPORTANT: The constructor does NOT write planningGridSnapshot.
 *  Writing the snapshot in the constructor would blank the GUI obstacle
 *  overlay every time a new group prototype is constructed — which
 *  happens after previous groups have already loaded their obstacles.
 *  The snapshot is written only inside mergeObstaclesIfNeeded().
 *
 * ── ONE Simulator integration ─────────────────────────────────
 *
 *  • getInitialLocation() — merge obstacles, spawn UAV, build POI tour,
 *                           run Layer 1 to populate keyNodes.
 *  • getPath()            — LOS check → optional re-plan → DWA step →
 *                           publish to peer registry → return segment.
 *  • nextPathAvailable()  — dwell time at a POI.
 *  • replicate()          — per-host copy (preserves all per-group params).
 *
 * ── Config keys ───────────────────────────────────────────────
 *
 *  Every key can be set in the shared namespace (UAVWaypointMovement.key)
 *  as a default for all drones, or in a per-group block (GroupN.key) to
 *  override for that drone only.  Per-group always wins.
 *
 *    spawn               csv(x,y)  required — drone start position (metres)
 *    target              csv(x,y)  required — drone goal position (metres)
 *    obstacleWktFile     string "" — WKT obstacle file; union-merged globally
 *    dwellMin            double 5.0     s    hover time at POI, min
 *    dwellMax            double 15.0    s    hover time at POI, max
 *    speedMin            double 0.8  m/s    cruise speed, min
 *    speedMax            double 1.0  m/s    cruise speed, max
 *    cruiseAlt           double 30.0    m   cruise altitude (metadata)
 *    gridCellM           double 5.0     m   A* grid cell side length
 *    pointObstacleRadius double gridCellM   POINT obstacle buffer radius
 *    lineObstacleHalfWidth double gridCellM LINESTRING obstacle half-width
 *    poiGridCols         int    0           POI lattice columns (0 = off)
 *    poiGridRows         int    0           POI lattice rows
 *    snapLocationsToGrid bool   true        snap spawn/goal/POIs to free cell
 *    showPlanningGrid    bool   false       GUI: draw A* grid overlay
 *    dwaSteps            int    8           DWA velocity samples
 *    distAlert           double 25.0    m   DWA alert distance
 *    distRisk            double 12.0    m   DWA danger distance
 *    uavSeparationM      double 20.0    m   VO cone radius / min peer separation
 *    voWeight            double 2.0         VO penalty multiplier
 *    launchStaggerS      double 8.0     s   per-UAV launch delay offset
 *    spawnStrideM        double 20.0    m   X-axis stride when per-group spawn
 *                                          is NOT set (all drones share the
 *                                          UAVWaypointMovement.spawn fallback)
 *
 * ── Installation ──────────────────────────────────────────────
 *
 *  1. Copy this file to  <ONE_ROOT>/movement/
 *  2. Recompile ONE normally (ant / gradlew).
 *  3. In settings:  Group1.movementModel = UAVWaypointMovement
 *
 * ============================================================
 */
public class UAVWaypointMovement extends MovementModel {

    /** Namespace for shared / fallback settings keys. */
    public static final String SETTINGS_NS = "UAVWaypointMovement";

    // ================================================================ //
    //  Settings key constants
    // ================================================================ //

    public static final String DWELL_MIN_S          = "dwellMin";
    public static final String DWELL_MAX_S          = "dwellMax";
    public static final String SPEED_MIN_S          = "speedMin";
    public static final String SPEED_MAX_S          = "speedMax";
    public static final String CRUISE_ALT_S         = "cruiseAlt";
    public static final String GRID_CELL_S          = "gridCellM";
    public static final String DWA_STEPS_S          = "dwaSteps";
    public static final String DIST_ALERT_S         = "distAlert";
    public static final String DIST_RISK_S          = "distRisk";
    public static final String OBSTACLE_WKT_S       = "obstacleWktFile";
    public static final String SPAWN_S              = "spawn";
    public static final String TARGET_S             = "target";
    public static final String POINT_RADIUS_S       = "pointObstacleRadius";
    public static final String LINE_HALF_WIDTH_S    = "lineObstacleHalfWidth";
    public static final String POI_GRID_COLS_S      = "poiGridCols";
    public static final String POI_GRID_ROWS_S      = "poiGridRows";
    public static final String SNAP_TO_GRID_S       = "snapLocationsToGrid";
    public static final String SHOW_PLANNING_GRID_S = "showPlanningGrid";
    public static final String UAV_SEP_S            = "uavSeparationM";
    public static final String VO_WEIGHT_S          = "voWeight";
    public static final String LAUNCH_STAGGER_S     = "launchStaggerS";
    public static final String SPAWN_STRIDE_S       = "spawnStrideM";
    public static final String GEO_EXTENTS_S        = "geoExtents";
    public static final String GEO_EXTENT_FILES_S   = "geoExtentFiles";
    public static final String GEO_ORIGIN_LON_S   = "geoOriginLon";
    public static final String GEO_ORIGIN_LAT_S   = "geoOriginLat";
    public static final String GEO_SCALE_FACTOR_S = "geoScaleFactor";

    // ================================================================ //
    //  Defaults
    // ================================================================ //

    private static final double DEF_DWELL_MIN       = 5.0;
    private static final double DEF_DWELL_MAX       = 15.0;
    private static final double DEF_SPEED_MIN       = 0.8;
    private static final double DEF_SPEED_MAX       = 1.0;
    private static final double DEF_CRUISE_ALT      = 30.0;
    private static final double DEF_GRID_CELL       = 5.0;
    private static final int    DEF_DWA_STEPS       = 8;
    private static final double DEF_DIST_ALERT      = 25.0;
    private static final double DEF_DIST_RISK       = 12.0;
    private static final double DEF_UAV_SEP         = 20.0;
    private static final double DEF_VO_WEIGHT       = 3.0;
    private static final double DEF_LAUNCH_STAGGER  = 8.0;
    private static final double DEF_SPAWN_STRIDE    = 20.0;

    // ================================================================ //
    //  DWA adaptive weight sets  (paper Table 4)
    //  index: 0=α(heading) 1=β(vel) 2=λ(obs) 3=η(follow)
    // ================================================================ //

    private static final double[] W_FOLLOW = { 0.30, 0.30, 0.20, 0.20 };
    private static final double[] W_ALERT  = { 0.20, 0.30, 0.40, 0.10 };
    private static final double[] W_DANGER = { 0.20, 0.25, 0.50, 0.05 };

    private static final int STALL_LIMIT = 6;

    // ================================================================ //
    //  Per-instance parameters
    // ================================================================ //

    private double  dwellMin, dwellMax;
    private double  speedMin, speedMax;
    private double  cruiseAlt;
    private double  gridCellM;
    private int     gridW, gridH;
    private int     dwaSteps;
    private double  distAlert, distRisk;
    private double  uavSeparationM;
    private double  voWeight;
    private double  launchStaggerS;
    private double  spawnStrideM;
    private double  spawnX, spawnY;
    private double  targetX, targetY;
    private double  pointObstacleRadius;
    private double  lineObstacleHalfWidth;
    private int     poiGridCols, poiGridRows;
    private boolean snapLocationsToGrid;
    private boolean perGroupSpawnSet = false;
    private List<String> groupObstacleWktFiles = new ArrayList<>();
    private List<String> geoExtentFiles = new ArrayList<>();
    private double[] explicitGeoExtents = null;

    // ================================================================ //
    //  Shared static obstacle grid
    // ================================================================ //

    private static boolean[][]    obstacleGrid   = null;
    private static List<double[]> obstacleDiscs  = null;
    private static List<double[]> obstacleSegBuf = null;

    private static final Set<String> loadedWktFiles
            = Collections.synchronizedSet(new LinkedHashSet<>());

    private static double geoMinX = Double.MAX_VALUE;
    private static double geoMaxX = -Double.MAX_VALUE;
    private static double geoMinY = Double.MAX_VALUE;
    private static double geoMaxY = -Double.MAX_VALUE;
    private static boolean geoScaleInitialised = false;

    private static double geoOriginLon   = 78.300;
    private static double geoOriginLat   = 17.480;
    private static double geoScaleFactor = 100000.0;

    // ================================================================ //
    //  Peer-UAV registry  (Velocity Obstacle)
    // ================================================================ //

    private static final ConcurrentHashMap<Integer, double[]> PEER_REGISTRY
            = new ConcurrentHashMap<>();

    private static final ConcurrentHashMap<Integer, Path> TRAIL_REGISTRY
            = new ConcurrentHashMap<>();

    public static Map<Integer, Path> getTrailRegistry() {
        return Collections.unmodifiableMap(TRAIL_REGISTRY);
    }

    private static final AtomicInteger ID_COUNTER = new AtomicInteger(0);

    private int    uavId = -1;
    private double lastVx = 0.0, lastVy = 0.0;

    private static gui.DTNSimGUI gui = null;

    public static void setGui(gui.DTNSimGUI g) {
        gui = g;
    }

    // ================================================================ //
    //  Planning-grid snapshot  (GUI obstacle overlay)
    // ================================================================ //

    public static final class PlanningGridSnapshot {
        public final double      gridCellM;
        public final int         gridW, gridH;
        public final boolean[][] blocked;

        private PlanningGridSnapshot(double c, int w, int h, boolean[][] b) {
            this.gridCellM = c; this.gridW = w; this.gridH = h; this.blocked = b;
        }
    }

    private static PlanningGridSnapshot planningGridSnapshot = null;

    private static boolean gridRenderingEnabled = true;

    public static PlanningGridSnapshot getPlanningGridSnapshot() {
        return gridRenderingEnabled ? planningGridSnapshot : null;
    }

    public static void setGridRenderingEnabled(boolean enabled) {
        gridRenderingEnabled = enabled;
    }

    public static boolean isGridRenderingEnabled() {
        return gridRenderingEnabled;
    }

    public static final class ObstacleRenderData {
        public enum Type { POINT, LINE, POLYGON }

        public final Type         type;
        public final List<Coord>  coords;
        public final double       radius;

        public ObstacleRenderData(Type type, List<Coord> coords, double radius) {
            this.type   = type;
            this.coords = new ArrayList<>(coords);
            this.radius = radius;
        }
    }

    private static List<ObstacleRenderData> obstacleRenderData = null;

    public static List<ObstacleRenderData> getObstacleRenderData() {
        return obstacleRenderData != null
               ? new ArrayList<>(obstacleRenderData)
               : new ArrayList<>();
    }

    private List<Coord> poiTour;
    private int         poiIndex;
    private List<Coord> keyNodes;
    private int         keyIndex;
    private Coord       uavPos;
    private boolean     hovering   = false;
    private double      hoverUntil = 0;
    private boolean     replanNeeded = false;
    private int         stallCount   = 0;

    private static final double DEFAULT_FINAL_GOAL_TOLERANCE = 2.0;
    public  static final String FINAL_GOAL_TOLERANCE_S = "finalGoalTolerance";
    private double  finalGoalTolerance;

    private boolean reachedFinalGoal = false;

    private Path trailPath = null;

    private static final int HISTORY_SIZE = 16;
    private final int[][] posHistory  = new int[HISTORY_SIZE][2];
    private int           historyHead = 0;
    private int           historyFill = 0;

    // ================================================================ //
    //  FIX: Peer-proximity check threshold for direct-walk bypass
    //
    //  When a peer UAV is closer than this fraction of uavSeparationM,
    //  the direct-walk primary mover hands off to DWA (which includes VO).
    //  This ensures VO fires even when there are no static obstacles
    //  between two converging drones — the original code only invoked
    //  DWA when the direct step landed in obstacleGrid, but peer UAVs
    //  are never written to obstacleGrid.
    // ================================================================ //
    private static final double PEER_CHECK_RATIO = 1.5;

    // ================================================================ //
    //  DEBUG: Stuck-detection logging
    //
    //  Log a detailed status message whenever the drone fires its stall
    //  handler, and track the last position where we logged so we avoid
    //  flooding the console on every single tick.
    // ================================================================ //
    /** Sim time of the last stall-debug log for this drone. */
    private double lastStallLogTime = -1.0;
    /** How often (sim seconds) to emit stall debug messages. */
    private static final double STALL_LOG_INTERVAL = 5.0;

    // ================================================================ //
    //  Two-level settings helpers
    // ================================================================ //

    private static double readDouble(Settings s, Settings cfg,
                                     String key, double def) {
        if (s   != null && s.contains(key))   return s.getDouble(key);
        if (cfg != null && cfg.contains(key)) return cfg.getDouble(key);
        return def;
    }

    private static int readInt(Settings s, Settings cfg,
                               String key, int def) {
        if (s   != null && s.contains(key))   return s.getInt(key);
        if (cfg != null && cfg.contains(key)) return cfg.getInt(key);
        return def;
    }

    private static boolean readBoolean(Settings s, Settings cfg,
                                       String key, boolean def) {
        if (s   != null && s.contains(key))   return s.getBoolean(key, def);
        if (cfg != null && cfg.contains(key)) return cfg.getBoolean(key, def);
        return def;
    }

    private static String readString(Settings s, Settings cfg,
                                     String key, String def) {
        if (s   != null && s.contains(key))   return s.getSetting(key);
        if (cfg != null && cfg.contains(key)) return cfg.getSetting(key);
        return def;
    }

    private static String readRawObstacleFileSetting(Settings s, Settings cfg, String key) {
        try {
            if (s != null && s.contains(key)) {
                String[] tokens = s.getCsvSetting(key);
                if (tokens != null && tokens.length > 0)
                    return String.join(",", tokens);
            }
            if (cfg != null && cfg.contains(key)) {
                String[] tokens = cfg.getCsvSetting(key);
                if (tokens != null && tokens.length > 0)
                    return String.join(",", tokens);
            }
        } catch (Exception ignored) {}
        return readString(s, cfg, key, "");
    }

    private static double[] readCsvDoubles(Settings s, Settings cfg,
                                           String key, int count) {
        if (s   != null && s.contains(key))   return s.getCsvDoubles(key, count);
        if (cfg != null && cfg.contains(key)) return cfg.getCsvDoubles(key, count);
        return null;
    }

    static List<String> parseObstacleFilePaths(String raw) {
        List<String> result = new ArrayList<>();
        if (raw == null || raw.trim().isEmpty()) return result;
        for (String token : raw.split(",")) {
            String path = token.trim();
            if (!path.isEmpty()) result.add(path);
        }
        return result;
    }

    private static List<String> readObstacleFilePaths(Settings s, Settings cfg) {
        String raw = readRawObstacleFileSetting(s, cfg, OBSTACLE_WKT_S);
        return parseObstacleFilePaths(raw);
    }

    // ================================================================ //
    //  Constructors
    // ================================================================ //

    public UAVWaypointMovement(Settings s) {
        super(s);
        Settings cfg = new Settings(SETTINGS_NS);

        double[] sp = readCsvDoubles(s, cfg, SPAWN_S, 2);
        double[] tg = readCsvDoubles(s, cfg, TARGET_S, 2);
        if (sp == null || tg == null) {
            throw new SimError("UAVWaypointMovement requires '"
                + SPAWN_S + "' and '" + TARGET_S
                + "' in GroupN.* or " + SETTINGS_NS + ".*");
        }
        spawnX = sp[0];  spawnY = sp[1];
        targetX = tg[0]; targetY = tg[1];

        perGroupSpawnSet = (s != null && s.contains(SPAWN_S));

        dwellMin   = readDouble(s, cfg, DWELL_MIN_S,     DEF_DWELL_MIN);
        dwellMax   = readDouble(s, cfg, DWELL_MAX_S,     DEF_DWELL_MAX);
        speedMin   = readDouble(s, cfg, SPEED_MIN_S,     DEF_SPEED_MIN);
        speedMax   = readDouble(s, cfg, SPEED_MAX_S,     DEF_SPEED_MAX);
        cruiseAlt  = readDouble(s, cfg, CRUISE_ALT_S,    DEF_CRUISE_ALT);
        gridCellM  = readDouble(s, cfg, GRID_CELL_S,     DEF_GRID_CELL);
        dwaSteps   = readInt   (s, cfg, DWA_STEPS_S,     DEF_DWA_STEPS);
        distAlert  = readDouble(s, cfg, DIST_ALERT_S,    DEF_DIST_ALERT);
        distRisk   = readDouble(s, cfg, DIST_RISK_S,     DEF_DIST_RISK);

        uavSeparationM = readDouble(s, cfg, UAV_SEP_S,        DEF_UAV_SEP);
        voWeight       = readDouble(s, cfg, VO_WEIGHT_S,      DEF_VO_WEIGHT);
        launchStaggerS = readDouble(s, cfg, LAUNCH_STAGGER_S, DEF_LAUNCH_STAGGER);
        spawnStrideM   = readDouble(s, cfg, SPAWN_STRIDE_S,   DEF_SPAWN_STRIDE);

        pointObstacleRadius   = readDouble(s, cfg, POINT_RADIUS_S,    gridCellM);
        lineObstacleHalfWidth = readDouble(s, cfg, LINE_HALF_WIDTH_S, gridCellM);

        poiGridCols         = readInt    (s, cfg, POI_GRID_COLS_S, 0);
        poiGridRows         = readInt    (s, cfg, POI_GRID_ROWS_S, 0);
        snapLocationsToGrid = readBoolean(s, cfg, SNAP_TO_GRID_S,  true);

        finalGoalTolerance  = readDouble (s, cfg, FINAL_GOAL_TOLERANCE_S,
                                          DEFAULT_FINAL_GOAL_TOLERANCE);

        gridW = (int) Math.ceil(worldW() / gridCellM);
        gridH = (int) Math.ceil(worldH() / gridCellM);

        groupObstacleWktFiles = readObstacleFilePaths(s, cfg);

        String geoExtStr = readString(s, cfg, GEO_EXTENTS_S, "");
        if (!geoExtStr.trim().isEmpty()) {
            String[] extParts = geoExtStr.split(",");
            if (extParts.length == 4) {
                try {
                    explicitGeoExtents = new double[]{
                        Double.parseDouble(extParts[0].trim()),
                        Double.parseDouble(extParts[1].trim()),
                        Double.parseDouble(extParts[2].trim()),
                        Double.parseDouble(extParts[3].trim())
                    };
                } catch (NumberFormatException e) {
                    System.err.println("[UAVWaypointMovement] WARNING: could not parse "
                        + GEO_EXTENTS_S + " value '" + geoExtStr + "' — falling back to auto-detect.");
                }
            }
        }

        String geoExtRaw = readRawObstacleFileSetting(s, cfg, GEO_EXTENT_FILES_S);
        geoExtentFiles = parseObstacleFilePaths(geoExtRaw);

        synchronized (UAVWaypointMovement.class) {
            if (!geoScaleInitialised) {
                geoOriginLon   = readDouble(s, cfg, GEO_ORIGIN_LON_S,   78.300);
                geoOriginLat   = readDouble(s, cfg, GEO_ORIGIN_LAT_S,   17.480);
                geoScaleFactor = readDouble(s, cfg, GEO_SCALE_FACTOR_S, 100000.0);
            }
        }
    }

    public UAVWaypointMovement(UAVWaypointMovement proto) {
        super(proto);
        this.dwellMin              = proto.dwellMin;
        this.dwellMax              = proto.dwellMax;
        this.speedMin              = proto.speedMin;
        this.speedMax              = proto.speedMax;
        this.cruiseAlt             = proto.cruiseAlt;
        this.gridCellM             = proto.gridCellM;
        this.gridW                 = proto.gridW;
        this.gridH                 = proto.gridH;
        this.dwaSteps              = proto.dwaSteps;
        this.distAlert             = proto.distAlert;
        this.distRisk              = proto.distRisk;
        this.uavSeparationM        = proto.uavSeparationM;
        this.voWeight              = proto.voWeight;
        this.launchStaggerS        = proto.launchStaggerS;
        this.spawnStrideM          = proto.spawnStrideM;
        this.spawnX                = proto.spawnX;
        this.spawnY                = proto.spawnY;
        this.targetX               = proto.targetX;
        this.targetY               = proto.targetY;
        this.pointObstacleRadius   = proto.pointObstacleRadius;
        this.lineObstacleHalfWidth = proto.lineObstacleHalfWidth;
        this.poiGridCols           = proto.poiGridCols;
        this.poiGridRows           = proto.poiGridRows;
        this.snapLocationsToGrid   = proto.snapLocationsToGrid;
        this.groupObstacleWktFiles  = new ArrayList<>(proto.groupObstacleWktFiles);
        this.perGroupSpawnSet      = proto.perGroupSpawnSet;
        this.geoExtentFiles        = new ArrayList<>(proto.geoExtentFiles);
        this.explicitGeoExtents    = proto.explicitGeoExtents != null
                                     ? proto.explicitGeoExtents.clone() : null;

        this.finalGoalTolerance    = proto.finalGoalTolerance;
        this.reachedFinalGoal      = false;

        this.uavId = ID_COUNTER.getAndIncrement();
    }

    // ================================================================ //
    //  ONE MovementModel interface
    // ================================================================ //

    @Override
    public Coord getInitialLocation() {
        mergeObstaclesIfNeeded();

        double offsetX = (!perGroupSpawnSet && uavId >= 0) ? uavId * spawnStrideM : 0.0;
        uavPos = new Coord(spawnX + offsetX, spawnY);
        if (snapLocationsToGrid) uavPos = snapToNearestFreeCell(uavPos);

        poiTour      = buildMissionPoiTour(uavPos);
        poiIndex     = 0;
        replanNeeded = false;
        stallCount   = 0;
        reachedFinalGoal = false;
        trailPath    = new Path(sampleSpeed());
        trailPath.addWaypoint(uavPos.clone());
        historyHead = 0;
        historyFill = 0;
        lastStallLogTime = -1.0;
        planToNextPoi();
        hovering   = false;
        hoverUntil = 0;

        if (uavId >= 0)
            PEER_REGISTRY.put(uavId,
                new double[]{ uavPos.getX(), uavPos.getY(), 0.0, 0.0 });

        return uavPos.clone();
    }

    @Override
    public Path getPath() {

        // ── FIX 1: Already stopped at goal? Stay put. ───────────────────────
        if (reachedFinalGoal) {
            Path path = new Path(0);
            path.addWaypoint(uavPos.clone());
            path.addWaypoint(uavPos.clone());
            return path;
        }

        // ── FIX 1: Are we close enough to the final goal right now? ─────────
        Coord finalGoal = poiTour.get(poiTour.size() - 1);
        if (poiIndex == poiTour.size() - 1
                && uavPos.distance(finalGoal) <= finalGoalTolerance) {
            reachedFinalGoal = true;
            lastVx = 0.0;
            lastVy = 0.0;
            if (uavId >= 0)
                PEER_REGISTRY.put(uavId,
                    new double[]{ uavPos.getX(), uavPos.getY(), 0.0, 0.0 });
            Path path = new Path(0);
            path.addWaypoint(uavPos.clone());
            path.addWaypoint(uavPos.clone());
            return path;
        }

        // ── Normal planning ─────────────────────────────────────────────────
        if (keyNodes == null || keyNodes.isEmpty()) planToNextPoi();

        Coord subGoal = keyNodes.get(keyIndex);

        if (replanNeeded) {
            replanNeeded = false; stallCount = 0;
            planToNextPoi();
            subGoal = keyNodes.get(keyIndex);
        }

        double prevX = uavPos.getX(), prevY = uavPos.getY();

        // ── FIX: Peer-proximity VO gate for direct-walk primary mover ────────
        //
        // PROBLEM: The direct-walk primary mover only falls back to DWA when
        // the candidate cell is in obstacleGrid.  Peer UAVs are never written
        // to obstacleGrid, so when two drones fly toward each other the direct
        // walk succeeds every tick and they pass through each other.
        //
        // FIX: Before attempting the direct walk, check whether any peer UAV
        // is within PEER_CHECK_RATIO * uavSeparationM.  If so, hand off to
        // DWA immediately so the VO penalty can steer us away.  This is a
        // minimal-change fix: the direct-walk path is unchanged for the
        // common case (no peers nearby); only conflicting geometry triggers
        // the VO-aware DWA path.
        boolean peerThreat = isPeerWithinRange(uavPos, PEER_CHECK_RATIO * uavSeparationM);

        double distToSub = uavPos.distance(subGoal);
        double stepSize  = Math.min(speedMax, distToSub);
        Coord nextPos;
        if (distToSub < 0.1) {
            nextPos = subGoal.clone();
        } else if (peerThreat) {
            // Peer nearby: use DWA so the VO penalty influences direction choice
            nextPos = dwaStep(uavPos, subGoal);
        } else {
            double dx = (subGoal.getX() - uavPos.getX()) / distToSub;
            double dy = (subGoal.getY() - uavPos.getY()) / distToSub;
            Coord direct = clampToWorld(new Coord(
                uavPos.getX() + dx * stepSize,
                uavPos.getY() + dy * stepSize));
            int[] dc = worldToGrid(direct);
            if (obstacleGrid[dc[1]][dc[0]]) {
                nextPos = dwaStep(uavPos, subGoal);
            } else {
                nextPos = direct;
            }
        }

        // ── Record current position in the circular history buffer ───────────
        int[] curCell = worldToGrid(uavPos);
        posHistory[historyHead][0] = curCell[0];
        posHistory[historyHead][1] = curCell[1];
        historyHead = (historyHead + 1) % HISTORY_SIZE;
        if (historyFill < HISTORY_SIZE) historyFill++;

        // ── Stall detection → skip waypoint then replan ──────────────────────
        if (nextPos.distance(uavPos) < speedMax * 0.1) {
            if (++stallCount >= STALL_LIMIT) {

                // ── DEBUG: Log detailed stuck-state information ───────────────
                // Emitted at most every STALL_LOG_INTERVAL sim-seconds so the
                // console is not flooded when the drone oscillates continuously.
                double now = SimClock.getTime();
                if (now - lastStallLogTime >= STALL_LOG_INTERVAL) {
                    lastStallLogTime = now;
                    logStuckState(subGoal, peerThreat);
                }

                stallCount  = 0;
                historyFill = 0;
                historyHead = 0;
                if (keyNodes != null && keyIndex < keyNodes.size() - 1) {
                    System.out.printf(
                        "[UAV%d DEBUG] Stall resolved: skipping to keyNode %d/%d"
                        + " @ (%.1f, %.1f)%n",
                        uavId, keyIndex + 1, keyNodes.size() - 1,
                        keyNodes.get(keyIndex + 1).getX(),
                        keyNodes.get(keyIndex + 1).getY());
                    keyIndex++;
                } else {
                    System.out.printf(
                        "[UAV%d DEBUG] Stall resolved: triggering full A* replan"
                        + " from (%.1f, %.1f)%n",
                        uavId, uavPos.getX(), uavPos.getY());
                    replanNeeded = true;
                }
            }
        } else {
            stallCount = 0;
        }

        if (nextPos.distance(subGoal) < gridCellM * 1.5) {
            uavPos = subGoal.clone();
            historyFill = 0; historyHead = 0;
            if (++keyIndex >= keyNodes.size()) {
                if (++poiIndex >= poiTour.size()) {
                    if (uavPos.distance(finalGoal) <= finalGoalTolerance) {
                        reachedFinalGoal = true;
                        lastVx = 0.0;
                        lastVy = 0.0;
                        if (uavId >= 0)
                            PEER_REGISTRY.put(uavId,
                                new double[]{ uavPos.getX(), uavPos.getY(), 0.0, 0.0 });
                        Path path = new Path(0);
                        path.addWaypoint(uavPos.clone());
                        path.addWaypoint(uavPos.clone());
                        return path;
                    } else {
                        poiIndex = 0;
                        poiTour  = buildMissionPoiTour(uavPos);
                    }
                }
                planToNextPoi();
            }
        } else {
            uavPos = nextPos.clone();
        }

        lastVx = uavPos.getX() - prevX;
        lastVy = uavPos.getY() - prevY;
        if (uavId >= 0)
            PEER_REGISTRY.put(uavId,
                new double[]{ uavPos.getX(), uavPos.getY(), lastVx, lastVy });

        if (trailPath == null) {
            trailPath = new Path(sampleSpeed());
            trailPath.addWaypoint(new Coord(prevX, prevY));
        }
        trailPath.addWaypoint(uavPos.clone());

        Path path = new Path(sampleSpeed());
        path.addWaypoint(new Coord(prevX, prevY));
        path.addWaypoint(uavPos.clone());

        if (uavId >= 0 && trailPath != null) {
            TRAIL_REGISTRY.put(uavId, trailPath);
        }

        return path;
    }

    // ================================================================ //
    //  FIX: Peer proximity check helper
    //
    //  Returns true when any registered peer UAV (excluding self) is
    //  within the given distance threshold of pos.  Called once per
    //  getPath() tick — O(n_peers) which is negligible for typical
    //  small swarms (≤20 drones).
    // ================================================================ //
    /**
     * Returns {@code true} if any peer UAV other than this one is currently
     * within {@code threshold} metres of {@code pos}.
     *
     * <p>This is used as a gate in {@link #getPath()} to decide whether the
     * direct-walk primary mover should hand off to DWA.  When no peer is
     * within range the gate is {@code false} and the fast direct-walk path
     * is taken; when a peer is nearby the gate fires and DWA (with its VO
     * penalty) is used instead, ensuring inter-drone avoidance activates
     * well before the drones reach physical contact distance.</p>
     *
     * @param pos       current world position of this UAV
     * @param threshold distance threshold in metres (typically
     *                  {@code PEER_CHECK_RATIO * uavSeparationM})
     * @return {@code true} if at least one peer is within threshold
     */
    private boolean isPeerWithinRange(Coord pos, double threshold) {
        for (Map.Entry<Integer, double[]> e : PEER_REGISTRY.entrySet()) {
            if (e.getKey() == uavId) continue;
            double[] p = e.getValue();
            double dist = Math.hypot(p[0] - pos.getX(), p[1] - pos.getY());
            if (dist < threshold) return true;
        }
        return false;
    }

    // ================================================================ //
    //  DEBUG: Stuck-state logger
    //
    //  Emits a structured log entry describing everything relevant to
    //  the drone's current failure mode:
    //    • Current world position and grid cell
    //    • Active sub-goal (current keyNode) and distance to it
    //    • Next waypoint if one exists
    //    • Whether a replan was already pending
    //    • Whether a peer UAV triggered the VO-handoff
    //    • Distances to every visible peer
    //    • A* key-node list with reached/pending annotation
    // ================================================================ //
    /**
     * Logs a detailed snapshot of this drone's decision state at the moment
     * it triggers the stall handler.  Called at most every
     * {@link #STALL_LOG_INTERVAL} sim-seconds to avoid console flooding.
     *
     * <p>Output format (one block per stall event):</p>
     * <pre>
     * ── [UAVn STUCK @ T=123.4s] ─────────────────────────────────────────
     *   Position  : (x, y)  grid[col, row]
     *   SubGoal   : (x, y)  dist=d m  (keyNode k/N)
     *   NextNode  : (x, y)  dist=d m   ← if one exists
     *   Flags     : replanPending=false  peerThreat=true
     *   Peers     :
     *     UAV3 @ (x, y)  dist=d m  vel=(vx, vy)
     *   KeyNodes  :
     *     [DONE] 0: (x, y)
     *     [CURR] 1: (x, y)
     *     [NEXT] 2: (x, y)
     * ────────────────────────────────────────────────────────────────────
     * </pre>
     *
     * @param subGoal    the current A* key-node the drone was trying to reach
     * @param peerThreat whether a nearby peer triggered the VO-handoff this tick
     */
    private void logStuckState(Coord subGoal, boolean peerThreat) {
        int[] gc = worldToGrid(uavPos);
        double distToSub = uavPos.distance(subGoal);

        StringBuilder sb = new StringBuilder();
        sb.append(String.format(
            "%n── [UAV%d STUCK @ T=%.1fs] ──────────────────────────────────────%n",
            uavId, SimClock.getTime()));
        sb.append(String.format(
            "  Position  : (%.1f, %.1f)  grid[%d, %d]%n",
            uavPos.getX(), uavPos.getY(), gc[0], gc[1]));
        sb.append(String.format(
            "  SubGoal   : (%.1f, %.1f)  dist=%.1f m  (keyNode %d/%d)%n",
            subGoal.getX(), subGoal.getY(), distToSub,
            keyIndex, keyNodes == null ? 0 : keyNodes.size() - 1));

        // Show next key-node if one exists
        if (keyNodes != null && keyIndex + 1 < keyNodes.size()) {
            Coord next = keyNodes.get(keyIndex + 1);
            sb.append(String.format(
                "  NextNode  : (%.1f, %.1f)  dist=%.1f m%n",
                next.getX(), next.getY(), uavPos.distance(next)));
        }

        // Show current target / POI progress
        Coord currentPoi = (poiTour != null && poiIndex < poiTour.size())
                           ? poiTour.get(poiIndex) : null;
        if (currentPoi != null) {
            sb.append(String.format(
                "  POI Target: (%.1f, %.1f)  dist=%.1f m  (poi %d/%d)%n",
                currentPoi.getX(), currentPoi.getY(),
                uavPos.distance(currentPoi),
                poiIndex, poiTour.size() - 1));
        }

        sb.append(String.format(
            "  Flags     : replanPending=%b  peerThreat=%b  stallCount=%d%n",
            replanNeeded, peerThreat, STALL_LIMIT));

        // List all visible peers and their distances
        sb.append("  Peers     :");
        boolean anyPeer = false;
        for (Map.Entry<Integer, double[]> e : PEER_REGISTRY.entrySet()) {
            if (e.getKey() == uavId) continue;
            double[] p = e.getValue();
            double dist = Math.hypot(p[0] - uavPos.getX(), p[1] - uavPos.getY());
            sb.append(String.format(
                "%n    UAV%d @ (%.1f, %.1f)  dist=%.1f m  vel=(%.2f, %.2f)",
                e.getKey(), p[0], p[1], dist, p[2], p[3]));
            anyPeer = true;
        }
        if (!anyPeer) sb.append(" (none registered)");
        sb.append("\n");

        // Annotate the full key-node list
        if (keyNodes != null && !keyNodes.isEmpty()) {
            sb.append("  KeyNodes  :\n");
            for (int i = 0; i < keyNodes.size(); i++) {
                Coord kn = keyNodes.get(i);
                String tag = i < keyIndex  ? "[DONE]"
                           : i == keyIndex ? "[CURR]"
                                           : "[NEXT]";
                sb.append(String.format(
                    "    %s %d: (%.1f, %.1f)  dist=%.1f m%n",
                    tag, i, kn.getX(), kn.getY(), uavPos.distance(kn)));
            }
        }

        sb.append("─────────────────────────────────────────────────────────────────\n");
        System.out.print(sb.toString());
    }

    @Override
    public double nextPathAvailable() {
        double dwell = dwellMin + this.rng.nextDouble() * (dwellMax - dwellMin);
        if (uavId >= 0 && SimClock.getTime() < 1.0) dwell += uavId * launchStaggerS;
        hoverUntil = SimClock.getTime() + dwell;
        hovering   = true;
        return hoverUntil;
    }

    @Override
    public UAVWaypointMovement replicate() { return new UAVWaypointMovement(this); }

    private double worldW() { return getMaxX(); }
    private double worldH() { return getMaxY(); }

    // ================================================================ //
    //  LAYER 1 — Improved A*
    // ================================================================ //

    private void planToNextPoi() {
        Coord goal = poiTour.get(poiIndex);
        List<int[]> raw = aStarSearch(worldToGrid(uavPos), worldToGrid(goal));
        if (raw == null || raw.isEmpty()) {
            keyNodes = new ArrayList<>();
            keyNodes.add(dwaEscapePoint(uavPos, goal));
            replanNeeded = true;
        } else {
            List<Coord> wp = gridPathToWorld(raw);
            wp.add(goal.clone());
            keyNodes = bresenhamExtractKeyNodes(wp);
        }
        keyIndex = 0;
    }

    private List<int[]> aStarSearch(int[] start, int[] goal) {
        int total = gridW * gridH;
        double[] gCost = new double[total]; Arrays.fill(gCost, Double.MAX_VALUE);
        int[] parentIdx = new int[total];   Arrays.fill(parentIdx, -1);
        int[] parentDir = new int[total];   Arrays.fill(parentDir, -1);
        boolean[] closed = new boolean[total];

        PriorityQueue<double[]> open = new PriorityQueue<>(
            Comparator.comparingDouble(a -> a[0]));
        int si = cellIndex(start[0], start[1]);
        gCost[si] = 0;
        open.offer(new double[]{ adaptiveHeuristic(start, goal), start[0], start[1] });

        int[][] dirs     = { {1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1} };
        double[] moveCost= { 1,1,1,1,
            Math.sqrt(2), Math.sqrt(2), Math.sqrt(2), Math.sqrt(2) };

        while (!open.isEmpty()) {
            double[] cur = open.poll();
            int col = (int)cur[1], row = (int)cur[2], idx = cellIndex(col, row);
            if (closed[idx]) continue;
            closed[idx] = true;
            if (col == goal[0] && row == goal[1])
                return reconstructPath(parentIdx, parentDir, goal, start);

            for (int di = 0; di < 8; di++) {
                int nc = col+dirs[di][0], nr = row+dirs[di][1];
                if (!inBounds(nc,nr) || obstacleGrid[nr][nc]) continue;
                if (di >= 4 && (obstacleGrid[row][nc] || obstacleGrid[nr][col])) continue;
                int ni = cellIndex(nc, nr);
                if (closed[ni]) continue;
                double tg = gCost[idx] + moveCost[di] * gridCellM;
                if (tg < gCost[ni]) {
                    gCost[ni] = tg; parentIdx[ni] = idx; parentDir[ni] = di;
                    double jitter = (this.rng.nextDouble() - 0.5) * gridCellM * 0.01;
                    open.offer(new double[]{ tg + adaptiveHeuristic(
                        new int[]{nc,nr}, goal) + jitter, nc, nr });
                }
            }
        }
        return null;
    }

    private double adaptiveHeuristic(int[] n, int[] g) {
        double manhattan = (Math.abs(n[0]-g[0]) + Math.abs(n[1]-g[1])) * gridCellM;
        double xi = obstacleCoverageRate(n, g);
        double multiplier = 1.0 + 0.3 / (1.0 + Math.exp(-xi * 10.0));
        return multiplier * manhattan;
    }

    private double obstacleCoverageRate(int[] n, int[] g) {
        int x0=n[0], y0=n[1], x1=g[0], y1=g[1];
        int dx=Math.abs(x1-x0), dy=Math.abs(y1-y0);
        int sx=x0<x1?1:-1, sy=y0<y1?1:-1;
        int err=dx-dy, x=x0, y=y0;
        int total=0, obs=0;
        while (true) {
            total++;
            if (inBounds(x,y) && obstacleGrid[y][x]) obs++;
            if (x==x1 && y==y1) break;
            int e2=2*err;
            if (e2>-dy) { err-=dy; x+=sx; }
            if (e2< dx) { err+=dx; y+=sy; }
        }
        return total==0 ? 0.0 : (double)obs/total;
    }

    @SuppressWarnings("unused")
    private int[] getActiveNeighbours(int d) {
        if (d < 0) return new int[]{ 0,1,2,3,4,5,6,7 };
        switch (d) {
            case 0: return new int[]{ 0, 4, 5 };
            case 1: return new int[]{ 1, 6, 7 };
            case 2: return new int[]{ 2, 4, 6 };
            case 3: return new int[]{ 3, 5, 7 };
            case 4: return new int[]{ 4, 0, 2 };
            case 5: return new int[]{ 5, 0, 3 };
            case 6: return new int[]{ 6, 1, 2 };
            case 7: return new int[]{ 7, 1, 3 };
            default: return new int[]{ 0,1,2,3,4,5,6,7 };
        }
    }

    private List<int[]> reconstructPath(int[] pi, int[] pd, int[] goal, int[] start) {
        LinkedList<int[]> path = new LinkedList<>();
        int idx = cellIndex(goal[0], goal[1]);
        while (idx != cellIndex(start[0], start[1])) {
            path.addFirst(new int[]{ idx % gridW, idx / gridW });
            int p = pi[idx]; if (p < 0 || p == idx) break; idx = p;
        }
        path.addFirst(start.clone());
        return new ArrayList<>(path);
    }

    // ================================================================ //
    //  Bresenham key-node extraction
    // ================================================================ //

    private List<Coord> bresenhamExtractKeyNodes(List<Coord> raw) {
        List<Coord> keys = new ArrayList<>();
        if (raw.size() <= 1) { keys.addAll(raw); return keys; }
        int start = 0; keys.add(raw.get(0));
        while (start < raw.size()-1) {
            int lastSafe = start+1;
            for (int j=start+2; j<raw.size(); j++) {
                if (bresenhamLOS(raw.get(start), raw.get(j))) lastSafe = j; else break;
            }
            keys.add(raw.get(lastSafe)); start = lastSafe;
        }
        return keys;
    }

    private boolean bresenhamLOS(Coord a, Coord b) {
        int[] ca = worldToGrid(a), cb = worldToGrid(b);
        int x0=ca[0], y0=ca[1], x1=cb[0], y1=cb[1];
        int dx=Math.abs(x1-x0), dy=Math.abs(y1-y0);
        int sx=x0<x1?1:-1, sy=y0<y1?1:-1, err=dx-dy, x=x0, y=y0;
        while (true) {
            if (!inBounds(x,y) || obstacleGrid[y][x]) return false;
            if (x==x1 && y==y1) break;
            int e2=2*err;
            if (e2>-dy) { err-=dy; x+=sx; }
            if (e2< dx) { err+=dx; y+=sy; }
        }
        return true;
    }

    // ================================================================ //
    //  LAYER 2 — Improved DWA with VO inter-UAV penalty
    // ================================================================ //

    private Coord dwaStep(Coord pos, Coord subGoal) {
        return dwaStepInternal(pos, subGoal, false);
    }

    private Coord dwaStepInternal(Coord pos, Coord subGoal, boolean fullSweep) {
        double d = nearestObstacleDistance(pos);
        double[] w = d<=distRisk ? W_DANGER : d<=distAlert ? W_ALERT : W_FOLLOW;
        double alpha=w[0], beta=w[1], lambda=w[2], eta=w[3];

        double bestScore = Double.NEGATIVE_INFINITY;
        Coord bestPos    = pos.clone();
        double step = Math.min(speedMax, Math.max(speedMin, pos.distance(subGoal)));

        double idBias = (uavId % 2 == 0) ? 0.35 : -0.35;

        double base     = Math.atan2(subGoal.getY()-pos.getY(), subGoal.getX()-pos.getX());
        int    nSamples = fullSweep ? dwaSteps * 2 : dwaSteps;

        List<double[]> peerSnapshots = new ArrayList<>();
        for (Map.Entry<Integer, double[]> e : PEER_REGISTRY.entrySet()) {
            if (e.getKey() == uavId) continue;
            double[] p = e.getValue();
            double px = p[0], py = p[1], pvx = p[2], pvy = p[3];
            double pvm = Math.hypot(pvx, pvy);
            if (pvm > 1e-9) {
                px += (pvx / pvm) * step;
                py += (pvy / pvm) * step;
            }
            peerSnapshots.add(new double[]{ px, py, p[2], p[3] });
        }

        for (int i=0; i<=nSamples; i++) {
            double off   = nSamples==0 ? 0.0
                         : fullSweep   ? ((double)i/nSamples) * 2.0 * Math.PI
                                       : ((double)i/nSamples - 0.5) * Math.PI;
            double angle = base+off;
            double v     = speedMin + (double)i/Math.max(1,nSamples)*(speedMax-speedMin);

            Coord cand = clampToWorld(new Coord(
                pos.getX()+step*Math.cos(angle), pos.getY()+step*Math.sin(angle)));
            int[] cell = worldToGrid(cand);
            if (obstacleGrid[cell[1]][cell[0]]) continue;

            double histPenalty = 0.0;
            for (int h = 0; h < historyFill; h++) {
                int hi = (historyHead - 1 - h + HISTORY_SIZE) % HISTORY_SIZE;
                if (posHistory[hi][0] == cell[0] && posHistory[hi][1] == cell[1]) {
                    histPenalty -= 1.2 * (1.0 - (double)h / HISTORY_SIZE);
                }
            }

            double peerPenalty = 0.0;
            boolean peerTooClose = false;
            double myVx = cand.getX()-pos.getX(), myVy = cand.getY()-pos.getY();
            double myVM = Math.hypot(myVx, myVy);

            for (double[] peer : peerSnapshots) {
                double px=peer[0], py=peer[1], pvx=peer[2], pvy=peer[3];
                double sd = Math.hypot(px - pos.getX(), py - pos.getY());
                if (sd < 0.5 || sd > uavSeparationM * 5) continue;

                double rx = px - pos.getX(), ry = py - pos.getY();

                if (sd < uavSeparationM * 0.7) {
                    double dot = (myVx*rx + myVy*ry) /
                                 Math.max(1e-9, myVM * sd);
                    peerPenalty -= voWeight * 5.0 * Math.max(0, dot);
                    peerTooClose = true;
                    continue;
                }

                double pvm = Math.hypot(pvx, pvy);
                double rvx, rvy;
                if (myVM < 1e-9 || pvm < 1e-9) {
                    rvx = myVx - pvx;
                    rvy = myVy - pvy;
                } else {
                    rvx = myVx/myVM - pvx/pvm;
                    rvy = myVy/myVM - pvy/pvm;
                }
                double rvm = Math.hypot(rvx, rvy);
                if (rvm < 1e-9) continue;

                double sinT = Math.min(1.0, uavSeparationM / sd);
                double cosT = Math.sqrt(Math.max(0, 1 - sinT*sinT));
                double cosA = (rvx*rx + rvy*ry) / (rvm * sd);
                if (cosA > cosT) {
                    double pen  = (cosA - cosT) / Math.max(1e-9, 1 - cosT);
                    double prox = 1 - Math.min(1, sd / (uavSeparationM * 5));
                    peerPenalty -= voWeight * pen * (1 + prox);
                }
            }

            double repulsionBonus = 0.0;
            for (double[] peer : peerSnapshots) {
                double sd = Math.hypot(peer[0]-pos.getX(), peer[1]-pos.getY());
                if (sd < 0.5 || sd > uavSeparationM * 3) continue;
                double rx = peer[0]-pos.getX(), ry = peer[1]-pos.getY();
                double dot = (myVx*rx + myVy*ry) /
                             Math.max(1e-9, myVM * sd);
                if (dot < 0) repulsionBonus += 0.15 * (-dot);
            }

            double tb = (peerPenalty < 0 || peerTooClose)
                        ? idBias * Math.signum(off + 1e-12) : 0;

            double bear    = Math.atan2(subGoal.getY()-cand.getY(),
                                        subGoal.getX()-cand.getX());
            double heading = Math.PI - Math.abs(normaliseAngle(bear - angle));
            double vel     = (v - speedMin) / Math.max(1e-9, speedMax - speedMin);
            double dObsS   = Math.min(1.0, nearestObstacleDistance(cand) / distAlert);
            double dFolS   = 1.0 - Math.min(1.0,
                             pointToSegmentDist(cand, pos, subGoal) / (gridCellM * 5));

            double score = alpha*heading + beta*vel + lambda*dObsS + eta*dFolS
                         + peerPenalty + repulsionBonus + histPenalty + tb;
            if (score > bestScore) { bestScore = score; bestPos = cand; }
        }

        if (!fullSweep && bestPos.distance(pos) < 0.5) {
            return dwaStepInternal(pos, subGoal, true);
        }

        return bestPos;
    }

    private Coord dwaEscapePoint(Coord pos, Coord goal) {
        double best=Double.NEGATIVE_INFINITY; Coord out=pos.clone();
        double step=gridCellM*3;
        double gb=Math.atan2(goal.getY()-pos.getY(), goal.getX()-pos.getX());
        int sw=dwaSteps*2;
        for (int i=0; i<=sw; i++) {
            double angle=((double)i/sw)*2*Math.PI;
            Coord c=clampToWorld(new Coord(
                pos.getX()+step*Math.cos(angle), pos.getY()+step*Math.sin(angle)));
            int[] cell=worldToGrid(c); if(obstacleGrid[cell[1]][cell[0]]) continue;
            double sc=0.75*Math.min(1,nearestObstacleDistance(c)/distAlert)
                    +0.25*0.5*(1+Math.cos(angle-gb));
            if (sc>best) { best=sc; out=c; }
        }
        return out;
    }

    // ================================================================ //
    //  Obstacle world generation — additive merge, never-overwrite
    // ================================================================ //

    private void mergeObstaclesIfNeeded() {
        synchronized (UAVWaypointMovement.class) {
        mergeObstaclesIfNeededLocked();
        }
    }

    private void mergeObstaclesIfNeededLocked() {
        if (obstacleGrid == null) {
            obstacleGrid      = new boolean[gridH][gridW];
            obstacleDiscs     = new ArrayList<>();
            obstacleSegBuf    = new ArrayList<>();
            obstacleRenderData = new ArrayList<>();
            publishPlanningGridSnapshot();
        }

        if (groupObstacleWktFiles == null || groupObstacleWktFiles.isEmpty()) return;

        boolean anyNewFile = false;

        for (String wkt : groupObstacleWktFiles) {
            if (wkt == null || wkt.trim().isEmpty()) continue;
            String trimmed = wkt.trim();

            String canonical;
            try {
                java.nio.file.Path p = java.nio.file.Paths.get(trimmed);
                if (!p.isAbsolute())
                    p = java.nio.file.Paths.get(System.getProperty("user.dir", ".")).resolve(p);
                p = p.normalize();
                canonical = java.nio.file.Files.exists(p) ? p.toRealPath().toString()
                                                           : p.toString();
            } catch (IOException e) {
                canonical = trimmed;
            }

            if (!loadedWktFiles.add(canonical)) continue;

            loadObstaclesFromWkt(trimmed);
            anyNewFile = true;
        }

        if (anyNewFile) {
            publishPlanningGridSnapshot();
        }
    }

    private void publishPlanningGridSnapshot() {
        boolean[][] copy = new boolean[gridH][gridW];
        for (int r=0; r<gridH; r++)
            System.arraycopy(obstacleGrid[r], 0, copy[r], 0, gridW);
        planningGridSnapshot = new PlanningGridSnapshot(gridCellM, gridW, gridH, copy);
    }

    public static synchronized void resetObstacleState() {
        obstacleGrid         = null;
        obstacleDiscs        = null;
        obstacleSegBuf       = null;
        obstacleRenderData   = null;
        planningGridSnapshot = null;
        gridRenderingEnabled = true;
        loadedWktFiles.clear();
        PEER_REGISTRY.clear();
        ID_COUNTER.set(0);
        geoMinX = Double.MAX_VALUE;  geoMaxX = -Double.MAX_VALUE;
        geoMinY = Double.MAX_VALUE;  geoMaxY = -Double.MAX_VALUE;
        geoScaleInitialised = false;
        geoOriginLon   = 78.300;
        geoOriginLat   = 17.480;
        geoScaleFactor = 100000.0;
    }

    // ================================================================ //
    //  Self-contained WKT obstacle loader
    // ================================================================ //

    private static final java.nio.charset.Charset[] WKT_CHARSETS = {
        java.nio.charset.StandardCharsets.UTF_8,
        java.nio.charset.Charset.forName("Windows-1252"),
        java.nio.charset.StandardCharsets.ISO_8859_1
    };

    private void loadObstaclesFromWkt(String rel) {
        java.nio.file.Path path = java.nio.file.Paths.get(rel);
        if (!path.isAbsolute())
            path = java.nio.file.Paths.get(System.getProperty("user.dir", ".")).resolve(path);

        if (!java.nio.file.Files.exists(path)) {
            throw new SimError(
                "UAVWaypointMovement: WKT file not found: " + path
                + "\n  Check that the path in your settings file is correct"
                + " and that the file exists relative to the ONE root directory.");
        }
        if (!java.nio.file.Files.isReadable(path)) {
            throw new SimError(
                "UAVWaypointMovement: WKT file exists but is not readable"
                + " (check OS permissions): " + path);
        }

        List<String> lines = null;
        IOException  lastEx = null;
        for (java.nio.charset.Charset cs : WKT_CHARSETS) {
            try {
                lines = java.nio.file.Files.readAllLines(path, cs);
                break;
            } catch (java.nio.charset.MalformedInputException mie) {
                lastEx = mie;
            } catch (IOException e) {
                throw new SimError(
                    "UAVWaypointMovement: I/O error reading WKT file: " + path
                    + "\n  " + e.getMessage());
            }
        }
        if (lines == null) {
            throw new SimError(
                "UAVWaypointMovement: WKT file could not be decoded with any"
                + " supported charset (UTF-8, Windows-1252, ISO-8859-1): " + path
                + (lastEx != null ? "\n  " + lastEx.getMessage() : ""));
        }

        List<double[]>       points      = new ArrayList<>();
        List<List<double[]>> linestrings = new ArrayList<>();
        List<List<double[]>> polygons    = new ArrayList<>();

        for (int lineNo = 0; lineNo < lines.size(); lineNo++) {
            String line = lines.get(lineNo).trim();
            if (line.isEmpty() || line.startsWith("#") || line.startsWith("//")) continue;
            try {
                wktExtractGeometry(line, points, linestrings, polygons);
            } catch (Exception e) {
                System.err.println("[UAVWaypointMovement] Skipping malformed geometry"
                    + " at " + path.getFileName() + " line " + (lineNo + 1)
                    + ": " + e.getMessage());
            }
        }

        normaliseCoordinatesIfGeographic(points, linestrings, polygons);

        for (double[] pt : points) {
            rasterizeDiscOnGrid(pt[0], pt[1], pointObstacleRadius);
            obstacleDiscs.add(new double[]{ pt[0], pt[1], pointObstacleRadius });
            if (obstacleRenderData != null)
                obstacleRenderData.add(new ObstacleRenderData(
                    ObstacleRenderData.Type.POINT,
                    java.util.Arrays.asList(new Coord(pt[0], pt[1])),
                    pointObstacleRadius));
        }
        for (List<double[]> ls : linestrings) {
            List<Coord> lsCoords = obstacleRenderData != null
                                   ? new ArrayList<>(ls.size()) : null;
            for (int i = 0; i < ls.size() - 1; i++) {
                double[] a = ls.get(i), b = ls.get(i + 1);
                rasterizeStripOnGrid(a[0], a[1], b[0], b[1], lineObstacleHalfWidth);
                obstacleSegBuf.add(new double[]{ a[0], a[1], b[0], b[1], lineObstacleHalfWidth });
            }
            if (lsCoords != null) {
                for (double[] p : ls) lsCoords.add(new Coord(p[0], p[1]));
                if (lsCoords.size() >= 2)
                    obstacleRenderData.add(new ObstacleRenderData(
                        ObstacleRenderData.Type.LINE, lsCoords, lineObstacleHalfWidth));
            }
        }
        for (List<double[]> ring : polygons) {
            rasterizeFilledPolygon(ring);
            for (int i = 0; i < ring.size() - 1; i++) {
                double[] a = ring.get(i), b = ring.get(i + 1);
                obstacleSegBuf.add(new double[]{ a[0], a[1], b[0], b[1], 0.0 });
            }
            if (obstacleRenderData != null) {
                List<Coord> pgCoords = new ArrayList<>(ring.size());
                for (double[] p : ring) pgCoords.add(new Coord(p[0], p[1]));
                if (pgCoords.size() >= 3)
                    obstacleRenderData.add(new ObstacleRenderData(
                        ObstacleRenderData.Type.POLYGON, pgCoords, 0.0));
            }
        }
    }

    private void normaliseCoordinatesIfGeographic(List<double[]>       points,
                                                   List<List<double[]>> linestrings,
                                                   List<List<double[]>> polygons) {
        double fileMinX = Double.MAX_VALUE, fileMaxX = -Double.MAX_VALUE;
        double fileMinY = Double.MAX_VALUE, fileMaxY = -Double.MAX_VALUE;

        for (double[] p : points) {
            fileMinX = Math.min(fileMinX, p[0]); fileMaxX = Math.max(fileMaxX, p[0]);
            fileMinY = Math.min(fileMinY, p[1]); fileMaxY = Math.max(fileMaxY, p[1]);
        }
        for (List<double[]> ls : linestrings)
            for (double[] p : ls) {
                fileMinX = Math.min(fileMinX, p[0]); fileMaxX = Math.max(fileMaxX, p[0]);
                fileMinY = Math.min(fileMinY, p[1]); fileMaxY = Math.max(fileMaxY, p[1]);
            }
        for (List<double[]> pg : polygons)
            for (double[] p : pg) {
                fileMinX = Math.min(fileMinX, p[0]); fileMaxX = Math.max(fileMaxX, p[0]);
                fileMinY = Math.min(fileMinY, p[1]); fileMaxY = Math.max(fileMaxY, p[1]);
            }

        if (fileMinX == Double.MAX_VALUE) return;

        double spanX = fileMaxX - fileMinX, spanY = fileMaxY - fileMinY;

        boolean isGeographic = (spanX <= 5.0 && spanY <= 5.0)
                && (fileMinX >= -180.0 && fileMinX <= 180.0)
                && (fileMinY >= -90.0  && fileMinY <= 90.0);
        if (!isGeographic) return;

        if (!geoScaleInitialised) {
            geoScaleInitialised = true;
            System.out.printf("[UAVWaypointMovement] Geographic transform locked:"
                + " originLon=%.4f  originLat=%.4f  scale=%.1f%n",
                geoOriginLon, geoOriginLat, geoScaleFactor);
        }

        double ox = geoOriginLon, oy = geoOriginLat, sc = geoScaleFactor;

        for (double[] p : points) {
            double lon = p[0], lat = p[1];
            p[0] = (lon - ox) * sc;
            p[1] = (oy - lat) * sc;
        }
        for (List<double[]> ls : linestrings)
            for (double[] p : ls) {
                double lon = p[0], lat = p[1];
                p[0] = (lon - ox) * sc;
                p[1] = (oy - lat) * sc;
            }
        for (List<double[]> pg : polygons)
            for (double[] p : pg) {
                double lon = p[0], lat = p[1];
                p[0] = (lon - ox) * sc;
                p[1] = (oy - lat) * sc;
            }
    }

    private static void wktExtractGeometry(String wkt,
                                           List<double[]>       points,
                                           List<List<double[]>> linestrings,
                                           List<List<double[]>> polygons) {
        String upper = wkt.trim().replaceAll("\\s+", " ").toUpperCase(java.util.Locale.ROOT);
        upper = upper.replaceAll("\\b(Z M|ZM|M|Z)\\s*(?=\\()", "");

        if (upper.startsWith("GEOMETRYCOLLECTION")) {
            String inner = extractOuterBody(wkt);
            if (inner == null) return;
            for (String member : splitTopLevelCommas(inner)) {
                wktExtractGeometry(member.trim(), points, linestrings, polygons);
            }

        } else if (upper.startsWith("MULTIPOLYGON")) {
            String inner = extractOuterBody(wkt);
            if (inner == null) return;
            for (String polyBody : splitTopLevelCommas(inner)) {
                wktExtractGeometry("POLYGON " + polyBody.trim(), points, linestrings, polygons);
            }

        } else if (upper.startsWith("POLYGON")) {
            String inner = extractOuterBody(wkt);
            if (inner == null) return;
            List<String> rings = splitTopLevelCommas(inner);
            for (int ri = 0; ri < rings.size(); ri++) {
                String stripped = rings.get(ri).trim();
                if (stripped.startsWith("(") && stripped.endsWith(")"))
                    stripped = stripped.substring(1, stripped.length() - 1);
                List<double[]> ring = parseCoordSequence(stripped);
                if (ring.size() >= 3) {
                    if (ri == 0) {
                        polygons.add(ring);
                    }
                }
            }

        } else if (upper.startsWith("MULTILINESTRING") || upper.startsWith("MULTISTRING")) {
            String inner = extractOuterBody(wkt);
            if (inner == null) return;
            for (String lsBody : splitTopLevelCommas(inner)) {
                String stripped = lsBody.trim();
                if (stripped.startsWith("(")) {
                    String candidate = extractOuterBody(stripped);
                    if (candidate != null) stripped = candidate;
                }
                List<double[]> ls = parseCoordSequence(stripped);
                if (ls.size() >= 2) linestrings.add(ls);
            }

        } else if (upper.startsWith("LINESTRING")) {
            String inner = extractOuterBody(wkt);
            if (inner == null) return;
            List<double[]> ls = parseCoordSequence(inner);
            if (ls.size() >= 2) linestrings.add(ls);

        } else if (upper.startsWith("MULTIPOINT")) {
            String inner = extractOuterBody(wkt);
            if (inner == null) return;
            for (String ptToken : splitTopLevelCommas(inner)) {
                String stripped = ptToken.trim();
                if (stripped.startsWith("(") && stripped.endsWith(")"))
                    stripped = stripped.substring(1, stripped.length() - 1);
                double[] pt = parseOneCoord(stripped.trim());
                if (pt != null) points.add(pt);
            }

        } else if (upper.startsWith("POINT")) {
            String inner = extractOuterBody(wkt);
            if (inner == null) return;
            double[] pt = parseOneCoord(inner.trim());
            if (pt != null) points.add(pt);

        } else {
            System.err.println("[UAVWaypointMovement] Skipping unsupported WKT type: "
                    + upper.split("\\s+|\\(")[0]);
        }
    }

    private static String extractOuterBody(String wkt) {
        int open = wkt.indexOf('(');
        if (open < 0) return null;
        int depth = 0;
        for (int i = open; i < wkt.length(); i++) {
            char c = wkt.charAt(i);
            if      (c == '(') depth++;
            else if (c == ')') {
                depth--;
                if (depth == 0) return wkt.substring(open + 1, i);
            }
        }
        return null;
    }

    private static List<String> splitTopLevelCommas(String s) {
        List<String> parts = new ArrayList<>();
        int depth = 0, start = 0;
        for (int i = 0; i < s.length(); i++) {
            char c = s.charAt(i);
            if      (c == '(') depth++;
            else if (c == ')') depth--;
            else if (c == ',' && depth == 0) {
                parts.add(s.substring(start, i));
                start = i + 1;
            }
        }
        if (start < s.length()) parts.add(s.substring(start));
        return parts;
    }

    private static List<double[]> parseCoordSequence(String seq) {
        List<double[]> coords = new ArrayList<>();
        if (seq == null || seq.trim().isEmpty()) return coords;
        for (String token : seq.split(",")) {
            double[] c = parseOneCoord(token.trim());
            if (c != null) coords.add(c);
        }
        return coords;
    }

    private static double[] parseOneCoord(String token) {
        if (token == null || token.isEmpty()) return null;
        String[] parts = token.trim().split("\\s+");
        if (parts.length < 2) return null;
        try {
            return new double[]{ Double.parseDouble(parts[0]),
                                 Double.parseDouble(parts[1]) };
        } catch (NumberFormatException e) {
            return null;
        }
    }

    private static double[] computeWktBoundingBox(String relPath) {
        try {
            java.nio.file.Path path = java.nio.file.Paths.get(relPath);
            if (!path.isAbsolute())
                path = java.nio.file.Paths.get(System.getProperty("user.dir", "."))
                           .resolve(path).normalize();
            if (!java.nio.file.Files.exists(path)) return null;

            List<double[]>       pts = new ArrayList<>();
            List<List<double[]>> lss = new ArrayList<>(), pgs = new ArrayList<>();

            List<String> lines;
            try {
                lines = java.nio.file.Files.readAllLines(
                    path, java.nio.charset.StandardCharsets.UTF_8);
            } catch (java.nio.charset.MalformedInputException e) {
                lines = java.nio.file.Files.readAllLines(
                    path, java.nio.charset.StandardCharsets.ISO_8859_1);
            }

            for (String line : lines) {
                line = line.trim();
                if (line.isEmpty() || line.startsWith("#") || line.startsWith("//")) continue;
                try { wktExtractGeometry(line, pts, lss, pgs); }
                catch (Exception ignored) {}
            }

            double minX = Double.MAX_VALUE, maxX = -Double.MAX_VALUE;
            double minY = Double.MAX_VALUE, maxY = -Double.MAX_VALUE;
            for (double[] p : pts) {
                minX = Math.min(minX, p[0]); maxX = Math.max(maxX, p[0]);
                minY = Math.min(minY, p[1]); maxY = Math.max(maxY, p[1]);
            }
            for (List<double[]> ls : lss)
                for (double[] p : ls) {
                    minX = Math.min(minX, p[0]); maxX = Math.max(maxX, p[0]);
                    minY = Math.min(minY, p[1]); maxY = Math.max(maxY, p[1]);
                }
            for (List<double[]> pg : pgs)
                for (double[] p : pg) {
                    minX = Math.min(minX, p[0]); maxX = Math.max(maxX, p[0]);
                    minY = Math.min(minY, p[1]); maxY = Math.max(maxY, p[1]);
                }

            if (minX == Double.MAX_VALUE) return null;

            double sx = maxX - minX, sy = maxY - minY;
            boolean geo = (sx <= 5.0 && sy <= 5.0)
                       && (minX >= -180.0 && minX <= 180.0)
                       && (minY >= -90.0  && minY <= 90.0);
            return geo ? new double[]{ minX, minY, maxX, maxY } : null;

        } catch (Exception e) {
            System.err.println("[UAVWaypointMovement] Could not scan bbox of: "
                + relPath + " (" + e.getMessage() + ")");
            return null;
        }
    }

    private void rasterizeDiscOnGrid(double cx, double cy, double r) {
        double r2=r*r;
        int c0=worldToGridCol(cx-r-gridCellM), c1=worldToGridCol(cx+r+gridCellM);
        int r0=worldToGridRow(cy-r-gridCellM), r1=worldToGridRow(cy+r+gridCellM);
        for (int row=Math.max(0,r0); row<=Math.min(gridH-1,r1); row++)
            for (int col=Math.max(0,c0); col<=Math.min(gridW-1,c1); col++) {
                Coord c=gridToWorld(col,row);
                double dx=c.getX()-cx, dy=c.getY()-cy;
                if (dx*dx+dy*dy<=r2) obstacleGrid[row][col]=true;
            }
    }

    private void rasterizeStripOnGrid(double x0,double y0,double x1,double y1,double hw){
        double pad=hw+gridCellM;
        int c0=worldToGridCol(Math.min(x0,x1)-pad), c1=worldToGridCol(Math.max(x0,x1)+pad);
        int r0=worldToGridRow(Math.min(y0,y1)-pad), r1=worldToGridRow(Math.max(y0,y1)+pad);
        for (int row=Math.max(0,r0); row<=Math.min(gridH-1,r1); row++)
            for (int col=Math.max(0,c0); col<=Math.min(gridW-1,c1); col++) {
                Coord c=gridToWorld(col,row);
                if (pointToSegmentDist(c, new Coord(x0,y0), new Coord(x1,y1))<=hw)
                    obstacleGrid[row][col]=true;
            }
    }

    private void rasterizeFilledPolygon(List<double[]> ring) {
        if (ring == null || ring.size() < 3) return;

        double minX = Double.MAX_VALUE, maxX = -Double.MAX_VALUE;
        double minY = Double.MAX_VALUE, maxY = -Double.MAX_VALUE;
        for (double[] p : ring) {
            minX = Math.min(minX, p[0]); maxX = Math.max(maxX, p[0]);
            minY = Math.min(minY, p[1]); maxY = Math.max(maxY, p[1]);
        }
        int colMin = Math.max(0, worldToGridCol(minX));
        int colMax = Math.min(gridW - 1, worldToGridCol(maxX));
        int rowMin = Math.max(0, worldToGridRow(minY));
        int rowMax = Math.min(gridH - 1, worldToGridRow(maxY));

        int n = ring.size();

        for (int row = rowMin; row <= rowMax; row++) {
            double cy = (row + 0.5) * gridCellM;

            List<Double> xs = new ArrayList<>();
            for (int i = 0, j = n - 1; i < n; j = i++) {
                double y0 = ring.get(i)[1], y1 = ring.get(j)[1];
                double x0 = ring.get(i)[0], x1 = ring.get(j)[0];
                if ((y0 <= cy && y1 > cy) || (y1 <= cy && y0 > cy)) {
                    double t = (cy - y0) / (y1 - y0);
                    xs.add(x0 + t * (x1 - x0));
                }
            }
            if (xs.isEmpty()) continue;
            Collections.sort(xs);

            for (int k = 0; k + 1 < xs.size(); k += 2) {
                double xLeft  = xs.get(k);
                double xRight = xs.get(k + 1);
                int cLeft  = Math.max(colMin, worldToGridCol(xLeft));
                int cRight = Math.min(colMax, worldToGridCol(xRight));
                for (int col = cLeft; col <= cRight; col++)
                    obstacleGrid[row][col] = true;
            }
        }
    }

    // ================================================================ //
    //  Snap / free-cell BFS
    // ================================================================ //

    private Coord snapToNearestFreeCell(Coord c) {
        int[] s=worldToGrid(c);
        s[0]=Math.max(0,Math.min(gridW-1,s[0]));
        s[1]=Math.max(0,Math.min(gridH-1,s[1]));
        if (!obstacleGrid[s[1]][s[0]]) return gridToWorld(s[0],s[1]);
        ArrayDeque<int[]> q=new ArrayDeque<>();
        boolean[][] seen=new boolean[gridH][gridW];
        q.add(s); seen[s[1]][s[0]]=true;
        int[][] dirs={{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};
        while (!q.isEmpty()) {
            int[] cur=q.poll();
            if (!obstacleGrid[cur[1]][cur[0]]) return gridToWorld(cur[0],cur[1]);
            for (int[] d:dirs) {
                int nc=cur[0]+d[0], nr=cur[1]+d[1];
                if (!inBounds(nc,nr)||seen[nr][nc]) continue;
                seen[nr][nc]=true; q.add(new int[]{nc,nr});
            }
        }
        throw new SimError("UAVWaypointMovement: no free cell near "+c);
    }

    private boolean isBlockedWorldXY(double x, double y) {
        int col = worldToGridCol(x);
        int row = worldToGridRow(y);
        return obstacleGrid[row][col];
    }

    // ================================================================ //
    //  POI tour
    // ================================================================ //

    private List<Coord> buildPoiGridWaypoints() {
        if (poiGridCols<=0||poiGridRows<=0) return new ArrayList<>();
        double W=worldW(), H=worldH();
        List<Coord> out=new ArrayList<>();
        for (int ix=0; ix<poiGridCols; ix++)
            for (int iy=0; iy<poiGridRows; iy++) {
                double x=(ix+0.5)*W/poiGridCols, y=(iy+0.5)*H/poiGridRows;
                if (isBlockedWorldXY(x,y)) continue;
                Coord c=new Coord(x,y);
                if (snapLocationsToGrid) c=snapToNearestFreeCell(c);
                out.add(c);
            }
        return out;
    }

    private List<Coord> buildMissionPoiTour(Coord origin) {
        List<Coord> lattice=buildPoiGridWaypoints();
        List<Coord> tour=new ArrayList<>();
        if (!lattice.isEmpty()) tour.addAll(nearestNeighbourTour(origin, new ArrayList<>(lattice)));
        Coord goal=new Coord(targetX, targetY);
        if (snapLocationsToGrid) goal=snapToNearestFreeCell(goal);
        tour.add(goal);
        return tour;
    }

    private List<Coord> nearestNeighbourTour(Coord start, List<Coord> pois) {
        List<Coord> rem=new ArrayList<>(pois), ord=new ArrayList<>(pois.size());
        Coord cur=start;
        while (!rem.isEmpty()) {
            Coord nn=null; double md=Double.MAX_VALUE;
            for (Coord c:rem) { double d=cur.distance(c); if(d<md){md=d;nn=c;} }
            ord.add(nn); rem.remove(nn); cur=nn;
        }
        return ord;
    }

    // ================================================================ //
    //  Geometry utilities
    // ================================================================ //

    private int[] worldToGrid(Coord c) {
        return new int[]{ worldToGridCol(c.getX()), worldToGridRow(c.getY()) };
    }
    private int worldToGridCol(double x) {
        return Math.max(0, Math.min((int)(x / gridCellM), gridW - 1));
    }
    private int worldToGridRow(double y) {
        return Math.max(0, Math.min((int)(y / gridCellM), gridH - 1));
    }
    @Deprecated
    private int worldToGridScalar(double v) {
        return worldToGridCol(v);
    }
    private Coord gridToWorld(int col, int row) {
        return new Coord((col+0.5)*gridCellM, (row+0.5)*gridCellM);
    }
    private List<Coord> gridPathToWorld(List<int[]> cells) {
        List<Coord> wps=new ArrayList<>(cells.size());
        for (int[] c:cells) wps.add(gridToWorld(c[0],c[1]));
        return wps;
    }
    private int     cellIndex(int col, int row) { return row*gridW+col; }
    private boolean inBounds (int col, int row) {
        return col>=0&&col<gridW&&row>=0&&row<gridH;
    }

    private double nearestObstacleDistance(Coord p) {
        double minD=Double.MAX_VALUE, px=p.getX(), py=p.getY();
        if (obstacleDiscs!=null)
            for (double[] d:obstacleDiscs)
                minD=Math.min(minD, Math.max(0, Math.hypot(px-d[0],py-d[1])-d[2]));
        if (obstacleSegBuf!=null)
            for (double[] s:obstacleSegBuf)
                minD=Math.min(minD, Math.max(0,
                    pointToSegmentDist(p,new Coord(s[0],s[1]),new Coord(s[2],s[3]))-s[4]));
        return minD==Double.MAX_VALUE ? distAlert*2 : minD;
    }

    private double pointToSegmentDist(Coord p, Coord a, Coord b) {
        double ax=a.getX(),ay=a.getY(),bx=b.getX(),by=b.getY();
        double px=p.getX(),py=p.getY(),dx=bx-ax,dy=by-ay;
        double ls=dx*dx+dy*dy;
        if (ls<1e-9) return p.distance(a);
        double t=Math.max(0,Math.min(1,((px-ax)*dx+(py-ay)*dy)/ls));
        return Math.hypot(px-(ax+t*dx), py-(ay+t*dy));
    }

    private double normaliseAngle(double a) {
        while (a> Math.PI) a-=2*Math.PI;
        while (a<-Math.PI) a+=2*Math.PI;
        return a;
    }

    private Coord clampToWorld(Coord c) {
        return new Coord(
            Math.max(1, Math.min(Math.max(2,worldW()-1), c.getX())),
            Math.max(1, Math.min(Math.max(2,worldH()-1), c.getY())));
    }

    private double sampleSpeed() {
        return speedMin + this.rng.nextDouble()*(speedMax-speedMin);
    }

    // ================================================================ //
    //  Public accessors
    // ================================================================ //

    public List<Coord> getKeyNodes() {
        return keyNodes==null ? Collections.emptyList()
                              : Collections.unmodifiableList(keyNodes);
    }
    public int     getKeyIndex()     { return keyIndex; }
    public int     getPoiIndex()     { return poiIndex; }
    public List<Coord> getPoiTour()  {
        return poiTour==null ? Collections.emptyList()
                             : Collections.unmodifiableList(poiTour);
    }
    public boolean isHovering()      { return hovering && SimClock.getTime()<hoverUntil; }
    public boolean isReplanPending() { return replanNeeded; }
    public boolean isReachedFinalGoal() { return reachedFinalGoal; }
    public int     getStallCount()   { return stallCount; }
    public Coord   getUavPos()       { return uavPos==null ? null : uavPos.clone(); }
    public int     getUavId()        { return uavId; }

    public Path getTrailPath() {
        return trailPath;
    }

    public static Map<Integer,double[]> getPeerRegistry() {
        return Collections.unmodifiableMap(PEER_REGISTRY);
    }

    public static Set<String> getLoadedWktFiles() {
        return Collections.unmodifiableSet(loadedWktFiles);
    }
}
