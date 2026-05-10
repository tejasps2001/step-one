package movement;

import core.Coord;
import core.Settings;
import core.SimClock;
import core.SimError;

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
 * ── Supporting classes ─────────────────────────────────────────
 *
 *  {@link UavObstacleGrid}  — all WKT loading, rasterization, geo-transform,
 *                              render-data, and grid snapshot management.
 *  {@link UavPathUtils}     — pure stateless geometry: grid↔world conversions,
 *                              distance queries, Bresenham LOS, BFS snap,
 *                              POI tour construction.
 *
 * ── Per-group configuration ────────────────────────────────────
 *
 *  ONE calls new UAVWaypointMovement(Settings s) once per GroupN block.
 *  Every parameter is read via readDouble/readInt/readBoolean/readString
 *  which checks s (per-group) first, then falls back to the shared
 *  UAVWaypointMovement.* namespace (cfg).
 *
 * ── Config keys ───────────────────────────────────────────────
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
 *  1. Copy UAVWaypointMovement.java, UavObstacleGrid.java, UavPathUtils.java
 *     to  <ONE_ROOT>/movement/
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
    public static final String GEO_ORIGIN_LON_S     = "geoOriginLon";
    public static final String GEO_ORIGIN_LAT_S     = "geoOriginLat";
    public static final String GEO_SCALE_FACTOR_S   = "geoScaleFactor";

    public static final String FINAL_GOAL_TOLERANCE_S = "finalGoalTolerance";

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
    private static final double DEFAULT_FINAL_GOAL_TOLERANCE = 2.0;

    // ================================================================ //
    //  DWA adaptive weight sets  (paper Table 4)
    //  index: 0=α(heading) 1=β(vel) 2=λ(obs) 3=η(follow)
    // ================================================================ //

    private static final double[] W_FOLLOW = { 0.30, 0.30, 0.20, 0.20 };
    private static final double[] W_ALERT  = { 0.20, 0.30, 0.40, 0.10 };
    private static final double[] W_DANGER = { 0.20, 0.25, 0.50, 0.05 };

    private static final int    STALL_LIMIT      = 6;
    private static final double PEER_CHECK_RATIO = 1.5;
    private static final int    HISTORY_SIZE     = 16;
    private static final double STALL_LOG_INTERVAL = 5.0;

    // ================================================================ //
    //  Per-instance parameters
    // ================================================================ //

    private double  dwellMin, dwellMax;
    private double  speedMin, speedMax;
    @SuppressWarnings("unused")
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
    private double  finalGoalTolerance;

    private List<String> groupObstacleWktFiles = new ArrayList<>();
    private List<String> geoExtentFiles        = new ArrayList<>();
    private double[]     explicitGeoExtents    = null;

    static {
        // Automatically register this class to be reset between batch runs
        core.DTNSim.registerForReset(UAVWaypointMovement.class.getName());
    }

    // ================================================================ //
    //  Peer-UAV registry  (Velocity Obstacle)
    // ================================================================ //

    private static final ConcurrentHashMap<Integer, double[]> PEER_REGISTRY
            = new ConcurrentHashMap<>();

    private static final ConcurrentHashMap<Integer, Path> TRAIL_REGISTRY
            = new ConcurrentHashMap<>();

    private static final AtomicInteger ID_COUNTER = new AtomicInteger(0);

    private int    uavId  = -1;
    private double lastVx = 0.0, lastVy = 0.0;

    private static gui.DTNSimGUI gui = null;

    public static void setGui(gui.DTNSimGUI g) { gui = g; }

    // ================================================================ //
    //  GUI toggle for planning-grid overlay
    // ================================================================ //

    public static final class PlanningGridSnapshot {
        public final double      gridCellM;
        public final int         gridW, gridH;
        public final boolean[][] blocked;

        PlanningGridSnapshot(double c, int w, int h, boolean[][] b) {
            this.gridCellM = c; this.gridW = w; this.gridH = h; this.blocked = b;
        }
    }

    private static PlanningGridSnapshot planningGridSnapshot = null;

    private static boolean gridRenderingEnabled = true;

    public static void setGridRenderingEnabled(boolean enabled) {
        gridRenderingEnabled = enabled;
    }

    public static boolean isGridRenderingEnabled() {
        return gridRenderingEnabled;
    }

    // ================================================================ //
    //  Inner types — public because the GUI overlay classes reference them
    // ================================================================ //

    /** Immutable snapshot of the A* grid, passed to the GUI renderer. */
    public static final class PlanningGridSnapshot {
        public final double      gridCellM;
        public final int         gridW, gridH;
        public final boolean[][] blocked;

        PlanningGridSnapshot(double c, int w, int h, boolean[][] b) {
            this.gridCellM = c; this.gridW = w; this.gridH = h; this.blocked = b;
        }
    }

    /** Per-obstacle geometry record for the GUI overlay. */
    public static final class ObstacleRenderData {
        public enum Type { POINT, LINE, POLYGON }

        public final Type        type;
        public final List<Coord> coords;
        public final double      radius;

        public ObstacleRenderData(Type type, List<Coord> coords, double radius) {
            this.type   = type;
            this.coords = new ArrayList<>(coords);
            this.radius = radius;
        }
    }

    // ================================================================ //
    //  Per-instance planning state
    // ================================================================ //

    private List<Coord> poiTour;
    private int         poiIndex;
    private List<Coord> keyNodes;
    private int         keyIndex;
    private Coord       uavPos;
    private boolean     hovering   = false;
    private double      hoverUntil = 0;
    private boolean     replanNeeded = false;
    private int         stallCount   = 0;
    private boolean     reachedFinalGoal = false;

    private Path trailPath = null;

    /** Circular position history — used to penalise revisiting cells in DWA. */
    private final int[][] posHistory  = new int[HISTORY_SIZE][2];
    private int           historyHead = 0;
    private int           historyFill = 0;

    private double lastStallLogTime = -1.0;

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
        if (sp == null || tg == null)
            throw new SimError("UAVWaypointMovement requires '"
                + SPAWN_S + "' and '" + TARGET_S
                + "' in GroupN.* or " + SETTINGS_NS + ".*");

        spawnX  = sp[0]; spawnY  = sp[1];
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

        finalGoalTolerance = readDouble(s, cfg, FINAL_GOAL_TOLERANCE_S,
                                        DEFAULT_FINAL_GOAL_TOLERANCE);

        gridW = (int) Math.ceil(worldW() / gridCellM);
        gridH = (int) Math.ceil(worldH() / gridCellM);

        groupObstacleWktFiles = readObstacleFilePaths(s, cfg);

        // ── Optional explicit geo extents ──────────────────────────────
        String geoExtStr = readString(s, cfg, GEO_EXTENTS_S, "");
        if (!geoExtStr.trim().isEmpty()) {
            String[] parts = geoExtStr.split(",");
            if (parts.length == 4) {
                try {
                    explicitGeoExtents = new double[]{
                        Double.parseDouble(parts[0].trim()),
                        Double.parseDouble(parts[1].trim()),
                        Double.parseDouble(parts[2].trim()),
                        Double.parseDouble(parts[3].trim())
                    };
                } catch (NumberFormatException e) {
                    System.err.println("[UAVWaypointMovement] WARNING: could not parse "
                        + GEO_EXTENTS_S + " value '" + geoExtStr + "' — falling back to auto-detect.");
                }
            }
        }

        String geoExtRaw = readRawObstacleFileSetting(s, cfg, GEO_EXTENT_FILES_S);
        geoExtentFiles = parseObstacleFilePaths(geoExtRaw);

        // ── Geo transform parameters — only captured once ──────────────
        synchronized (UAVWaypointMovement.class) {
            if (!UavObstacleGrid.geoScaleInitialised) {
                UavObstacleGrid.geoOriginLon   = readDouble(s, cfg, GEO_ORIGIN_LON_S,   78.300);
                UavObstacleGrid.geoOriginLat   = readDouble(s, cfg, GEO_ORIGIN_LAT_S,   17.480);
                UavObstacleGrid.geoScaleFactor = readDouble(s, cfg, GEO_SCALE_FACTOR_S, 100000.0);
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
        this.groupObstacleWktFiles = new ArrayList<>(proto.groupObstacleWktFiles);
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
        if (snapLocationsToGrid)
            uavPos = UavPathUtils.snapToNearestFreeCell(
                uavPos, UavObstacleGrid.obstacleGrid, gridCellM, gridW, gridH);

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

        // ── FIX 1: Already stopped at goal? Stay put. ───────────────────
        if (reachedFinalGoal) {
            Path path = new Path(0);
            path.addWaypoint(uavPos.clone());
            path.addWaypoint(uavPos.clone());
            return path;
        }

        // ── FIX 1: Are we close enough to the final goal right now? ─────
        Coord finalGoal = poiTour.get(poiTour.size() - 1);
        if (poiIndex == poiTour.size() - 1
                && uavPos.distance(finalGoal) <= finalGoalTolerance) {
            reachedFinalGoal = true;
            lastVx = 0.0; lastVy = 0.0;
            if (uavId >= 0)
                PEER_REGISTRY.put(uavId,
                    new double[]{ uavPos.getX(), uavPos.getY(), 0.0, 0.0 });
            Path path = new Path(0);
            path.addWaypoint(uavPos.clone());
            path.addWaypoint(uavPos.clone());
            return path;
        }

        // ── Normal planning ─────────────────────────────────────────────
        if (keyNodes == null || keyNodes.isEmpty()) planToNextPoi();

        Coord subGoal = keyNodes.get(keyIndex);

        if (replanNeeded) {
            replanNeeded = false; stallCount = 0;
            planToNextPoi();
            subGoal = keyNodes.get(keyIndex);
        }

        double prevX = uavPos.getX(), prevY = uavPos.getY();

        // ── Peer-proximity VO gate for direct-walk primary mover ─────────
        boolean peerThreat = isPeerWithinRange(uavPos, PEER_CHECK_RATIO * uavSeparationM);

        double distToSub = uavPos.distance(subGoal);
        double stepSize  = Math.min(speedMax, distToSub);
        Coord nextPos;
        if (distToSub < 0.1) {
            nextPos = subGoal.clone();
        } else if (peerThreat) {
            nextPos = dwaStep(uavPos, subGoal);
        } else {
            double dx = (subGoal.getX() - uavPos.getX()) / distToSub;
            double dy = (subGoal.getY() - uavPos.getY()) / distToSub;
            Coord direct = UavPathUtils.clampToWorld(
                new Coord(uavPos.getX() + dx * stepSize,
                          uavPos.getY() + dy * stepSize),
                worldW(), worldH());
            int[] dc = worldToGrid(direct);
            nextPos = UavObstacleGrid.obstacleGrid[dc[1]][dc[0]]
                      ? dwaStep(uavPos, subGoal)
                      : direct;
        }

        // ── Record current position in the circular history buffer ───────
        int[] curCell = worldToGrid(uavPos);
        posHistory[historyHead][0] = curCell[0];
        posHistory[historyHead][1] = curCell[1];
        historyHead = (historyHead + 1) % HISTORY_SIZE;
        if (historyFill < HISTORY_SIZE) historyFill++;

        // ── Stall detection → skip waypoint then replan ──────────────────
        if (nextPos.distance(uavPos) < speedMax * 0.1) {
            if (++stallCount >= STALL_LIMIT) {
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
                        "[UAV%d DEBUG] Stall resolved: skipping to keyNode %d/%d @ (%.1f, %.1f)%n",
                        uavId, keyIndex + 1, keyNodes.size() - 1,
                        keyNodes.get(keyIndex + 1).getX(),
                        keyNodes.get(keyIndex + 1).getY());
                    keyIndex++;
                } else {
                    System.out.printf(
                        "[UAV%d DEBUG] Stall resolved: triggering full A* replan from (%.1f, %.1f)%n",
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
                        lastVx = 0.0; lastVy = 0.0;
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

        if (uavId >= 0 && trailPath != null)
            TRAIL_REGISTRY.put(uavId, trailPath);

        return path;
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

    // ================================================================ //
    //  Peer proximity check helper
    // ================================================================ //

    /**
     * Returns {@code true} if any peer UAV other than this one is currently
     * within {@code threshold} metres of {@code pos}.
     */
    private boolean isPeerWithinRange(Coord pos, double threshold) {
        for (Map.Entry<Integer, double[]> e : PEER_REGISTRY.entrySet()) {
            if (e.getKey() == uavId) continue;
            double[] p = e.getValue();
            if (Math.hypot(p[0] - pos.getX(), p[1] - pos.getY()) < threshold) return true;
        }
        return false;
    }

    // ================================================================ //
    //  DEBUG: Stuck-state logger
    // ================================================================ //

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

        if (keyNodes != null && keyIndex + 1 < keyNodes.size()) {
            Coord next = keyNodes.get(keyIndex + 1);
            sb.append(String.format(
                "  NextNode  : (%.1f, %.1f)  dist=%.1f m%n",
                next.getX(), next.getY(), uavPos.distance(next)));
        }

        Coord currentPoi = (poiTour != null && poiIndex < poiTour.size())
                           ? poiTour.get(poiIndex) : null;
        if (currentPoi != null)
            sb.append(String.format(
                "  POI Target: (%.1f, %.1f)  dist=%.1f m  (poi %d/%d)%n",
                currentPoi.getX(), currentPoi.getY(),
                uavPos.distance(currentPoi), poiIndex, poiTour.size() - 1));

        sb.append(String.format(
            "  Flags     : replanPending=%b  peerThreat=%b  stallCount=%d%n",
            replanNeeded, peerThreat, STALL_LIMIT));

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

    // ================================================================ //
    //  LAYER 1 — Improved A*
    // ================================================================ //

    private void planToNextPoi() {
        Coord goal = poiTour.get(poiIndex);
        List<int[]> raw = aStarSearch(worldToGrid(uavPos), worldToGrid(goal));
        if (raw == null || raw.isEmpty()) {
            keyNodes = new ArrayList<>();
            keyNodes.add(dwaEscapePoint(uavPos, goal));
            // Fix: Do not instantly trigger replanNeeded = true here, otherwise 
            // the A* algorithm will continuously run every single tick until it escapes, 
            // freezing the simulation. Let the drone physically move to the escape point first.
        } else {
            List<Coord> wp = UavPathUtils.gridPathToWorld(raw, gridCellM);
            wp.add(goal.clone());
            keyNodes = UavPathUtils.bresenhamExtractKeyNodes(
                wp, UavObstacleGrid.obstacleGrid, gridCellM, gridW, gridH);
        }
        keyIndex = 0;
    }

    private List<int[]> aStarSearch(int[] start, int[] goal) {
        int total = gridW * gridH;
        double[] gCost  = new double[total]; Arrays.fill(gCost, Double.MAX_VALUE);
        int[]  parentIdx = new int[total];   Arrays.fill(parentIdx, -1);
        int[]  parentDir = new int[total];   Arrays.fill(parentDir, -1);
        boolean[] closed = new boolean[total];

        PriorityQueue<double[]> open =
            new PriorityQueue<>(Comparator.comparingDouble(a -> a[0]));
        int si = cellIndex(start[0], start[1]);
        gCost[si] = 0;
        open.offer(new double[]{ adaptiveHeuristic(start, goal), start[0], start[1] });

        int[][] dirs      = { {1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1} };
        double[] moveCost = { 1, 1, 1, 1,
            Math.sqrt(2), Math.sqrt(2), Math.sqrt(2), Math.sqrt(2) };

        while (!open.isEmpty()) {
            double[] cur = open.poll();
            int col = (int)cur[1], row = (int)cur[2], idx = cellIndex(col, row);
            if (closed[idx]) continue;
            closed[idx] = true;
            if (col == goal[0] && row == goal[1])
                return reconstructPath(parentIdx, goal, start);

            for (int di = 0; di < 8; di++) {
                int nc = col + dirs[di][0], nr = row + dirs[di][1];
                if (!inBounds(nc, nr) || UavObstacleGrid.obstacleGrid[nr][nc]) continue;
                if (di >= 4 && (UavObstacleGrid.obstacleGrid[row][nc]
                             || UavObstacleGrid.obstacleGrid[nr][col])) continue;
                int ni = cellIndex(nc, nr);
                if (closed[ni]) continue;
                double tg = gCost[idx] + moveCost[di] * gridCellM;
                if (tg < gCost[ni]) {
                    gCost[ni] = tg; parentIdx[ni] = idx; parentDir[ni] = di;
                    double jitter = (this.rng.nextDouble() - 0.5) * gridCellM * 0.01;
                    open.offer(new double[]{
                        tg + adaptiveHeuristic(new int[]{nc, nr}, goal) + jitter, nc, nr });
                }
            }
        }
        return null;
    }

    private double adaptiveHeuristic(int[] n, int[] g) {
        double manhattan = (Math.abs(n[0] - g[0]) + Math.abs(n[1] - g[1])) * gridCellM;
        double xi = obstacleCoverageRate(n, g);
        double multiplier = 1.0 + 0.3 / (1.0 + Math.exp(-xi * 10.0));
        return multiplier * manhattan;
    }

    private double obstacleCoverageRate(int[] n, int[] g) {
        int x0 = n[0], y0 = n[1], x1 = g[0], y1 = g[1];
        int dx = Math.abs(x1 - x0), dy = Math.abs(y1 - y0);
        int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
        int err = dx - dy, x = x0, y = y0;
        int total = 0, obs = 0;
        while (true) {
            total++;
            if (inBounds(x, y) && UavObstacleGrid.obstacleGrid[y][x]) obs++;
            if (x == x1 && y == y1) break;
            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x += sx; }
            if (e2 <  dx) { err += dx; y += sy; }
        }
        return total == 0 ? 0.0 : (double) obs / total;
    }

    private List<int[]> reconstructPath(int[] pi, int[] goal, int[] start) {
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
    //  LAYER 2 — Improved DWA with VO inter-UAV penalty
    // ================================================================ //

    private Coord dwaStep(Coord pos, Coord subGoal) {
        return dwaStepInternal(pos, subGoal, false);
    }

    private Coord dwaStepInternal(Coord pos, Coord subGoal, boolean fullSweep) {
        double d = UavPathUtils.nearestObstacleDistance(
            pos, UavObstacleGrid.obstacleDiscs, UavObstacleGrid.obstacleSegBuf,
            distAlert * 2);
        double[] w = d <= distRisk ? W_DANGER : d <= distAlert ? W_ALERT : W_FOLLOW;
        double alpha = w[0], beta = w[1], lambda = w[2], eta = w[3];

        double bestScore = Double.NEGATIVE_INFINITY;
        Coord  bestPos   = pos.clone();
        double step = Math.min(speedMax, Math.max(speedMin, pos.distance(subGoal)));

        double idBias  = (uavId % 2 == 0) ? 0.35 : -0.35;
        double base    = Math.atan2(subGoal.getY() - pos.getY(),
                                    subGoal.getX() - pos.getX());
        int nSamples   = fullSweep ? dwaSteps * 2 : dwaSteps;

        // ── Build peer snapshots (predict one step forward) ──────────────
        List<double[]> peerSnapshots = new ArrayList<>();
        for (Map.Entry<Integer, double[]> e : PEER_REGISTRY.entrySet()) {
            if (e.getKey() == uavId) continue;
            double[] p = e.getValue();
            double px = p[0], py = p[1], pvx = p[2], pvy = p[3];
            double pvm = Math.hypot(pvx, pvy);
            if (pvm > 1e-9) { px += (pvx / pvm) * step; py += (pvy / pvm) * step; }
            peerSnapshots.add(new double[]{ px, py, p[2], p[3] });
        }

        for (int i = 0; i <= nSamples; i++) {
            double off = nSamples == 0 ? 0.0
                       : fullSweep    ? ((double) i / nSamples) * 2.0 * Math.PI
                                      : ((double) i / nSamples - 0.5) * Math.PI;
            double angle = base + off;
            double v     = speedMin + (double) i / Math.max(1, nSamples) * (speedMax - speedMin);

            Coord cand = UavPathUtils.clampToWorld(
                new Coord(pos.getX() + step * Math.cos(angle),
                          pos.getY() + step * Math.sin(angle)),
                worldW(), worldH());
            int[] cell = worldToGrid(cand);
            if (UavObstacleGrid.obstacleGrid[cell[1]][cell[0]]) continue;

            // ── History penalty ──────────────────────────────────────────
            double histPenalty = 0.0;
            for (int h = 0; h < historyFill; h++) {
                int hi = (historyHead - 1 - h + HISTORY_SIZE) % HISTORY_SIZE;
                if (posHistory[hi][0] == cell[0] && posHistory[hi][1] == cell[1])
                    histPenalty -= 1.2 * (1.0 - (double) h / HISTORY_SIZE);
            }

            // ── VO peer penalties ────────────────────────────────────────
            double peerPenalty  = 0.0;
            boolean peerTooClose = false;
            double myVx = cand.getX() - pos.getX();
            double myVy = cand.getY() - pos.getY();
            double myVM = Math.hypot(myVx, myVy);

            for (double[] peer : peerSnapshots) {
                double px = peer[0], py = peer[1], pvx = peer[2], pvy = peer[3];
                double sd = Math.hypot(px - pos.getX(), py - pos.getY());
                if (sd < 0.5 || sd > uavSeparationM * 5) continue;
                double rx = px - pos.getX(), ry = py - pos.getY();

                if (sd < uavSeparationM * 0.7) {
                    double dot = (myVx * rx + myVy * ry)
                                 / Math.max(1e-9, myVM * sd);
                    peerPenalty -= voWeight * 5.0 * Math.max(0, dot);
                    peerTooClose = true;
                    continue;
                }

                double pvm = Math.hypot(pvx, pvy);
                double rvx, rvy;
                if (myVM < 1e-9 || pvm < 1e-9) { rvx = myVx - pvx; rvy = myVy - pvy; }
                else { rvx = myVx / myVM - pvx / pvm; rvy = myVy / myVM - pvy / pvm; }
                double rvm = Math.hypot(rvx, rvy);
                if (rvm < 1e-9) continue;

                double sinT = Math.min(1.0, uavSeparationM / sd);
                double cosT = Math.sqrt(Math.max(0, 1 - sinT * sinT));
                double cosA = (rvx * rx + rvy * ry) / (rvm * sd);
                if (cosA > cosT) {
                    double pen  = (cosA - cosT) / Math.max(1e-9, 1 - cosT);
                    double prox = 1 - Math.min(1, sd / (uavSeparationM * 5));
                    peerPenalty -= voWeight * pen * (1 + prox);
                }
            }

            // ── Repulsion bonus ──────────────────────────────────────────
            double repulsionBonus = 0.0;
            for (double[] peer : peerSnapshots) {
                double sd = Math.hypot(peer[0] - pos.getX(), peer[1] - pos.getY());
                if (sd < 0.5 || sd > uavSeparationM * 3) continue;
                double rx = peer[0] - pos.getX(), ry = peer[1] - pos.getY();
                double dot = (myVx * rx + myVy * ry) / Math.max(1e-9, myVM * sd);
                if (dot < 0) repulsionBonus += 0.15 * (-dot);
            }

            double tb = (peerPenalty < 0 || peerTooClose)
                        ? idBias * Math.signum(off + 1e-12) : 0;

            double bear    = Math.atan2(subGoal.getY() - cand.getY(),
                                        subGoal.getX() - cand.getX());
            double heading = Math.PI - Math.abs(UavPathUtils.normaliseAngle(bear - angle));
            double vel     = (v - speedMin) / Math.max(1e-9, speedMax - speedMin);
            double dObsS   = Math.min(1.0,
                UavPathUtils.nearestObstacleDistance(cand,
                    UavObstacleGrid.obstacleDiscs, UavObstacleGrid.obstacleSegBuf,
                    distAlert * 2) / distAlert);
            double dFolS   = 1.0 - Math.min(1.0,
                UavPathUtils.pointToSegmentDist(cand, pos, subGoal) / (gridCellM * 5));

            double score = alpha * heading + beta * vel
                         + lambda * dObsS  + eta * dFolS
                         + peerPenalty + repulsionBonus + histPenalty + tb;
            if (score > bestScore) { bestScore = score; bestPos = cand; }
        }

        if (!fullSweep && bestPos.distance(pos) < 0.5)
            return dwaStepInternal(pos, subGoal, true);

        return bestPos;
    }

    private Coord dwaEscapePoint(Coord pos, Coord goal) {
        double best = Double.NEGATIVE_INFINITY;
        Coord  out  = pos.clone();
        double step = gridCellM * 3;
        double gb   = Math.atan2(goal.getY() - pos.getY(), goal.getX() - pos.getX());
        int sw = dwaSteps * 2;
        for (int i = 0; i <= sw; i++) {
            double angle = ((double) i / sw) * 2 * Math.PI;
            Coord c = UavPathUtils.clampToWorld(
                new Coord(pos.getX() + step * Math.cos(angle),
                          pos.getY() + step * Math.sin(angle)),
                worldW(), worldH());
            int[] cell = worldToGrid(c);
            if (UavObstacleGrid.obstacleGrid[cell[1]][cell[0]]) continue;
            double sc = 0.75 * Math.min(1, UavPathUtils.nearestObstacleDistance(
                                c, UavObstacleGrid.obstacleDiscs,
                                UavObstacleGrid.obstacleSegBuf, distAlert * 2) / distAlert)
                      + 0.25 * 0.5 * (1 + Math.cos(angle - gb));
            if (sc > best) { best = sc; out = c; }
        }
        return out;
    }

    // ================================================================ //
    //  Obstacle world generation — additive merge, never-overwrite
    // ================================================================ //

    private void mergeObstaclesIfNeeded() {
        synchronized (UAVWaypointMovement.class) {
            UavObstacleGrid.mergeIfNeeded(
                groupObstacleWktFiles,
                gridW, gridH, gridCellM,
                pointObstacleRadius, lineObstacleHalfWidth);
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

    public static synchronized void reset() {
        obstacleGrid         = null;
        obstacleDiscs        = null;
        obstacleSegBuf       = null;
        obstacleRenderData   = null;
        planningGridSnapshot = null;
        gridRenderingEnabled = true;
        PEER_REGISTRY.clear();
        TRAIL_REGISTRY.clear();
        ID_COUNTER.set(0);
    }

    // ================================================================ //
    //  POI tour
    // ================================================================ //

    private List<Coord> buildMissionPoiTour(Coord origin) {
        List<Coord> tour = new ArrayList<>(
            UavPathUtils.buildPoiGridWaypoints(
                origin, poiGridCols, poiGridRows,
                worldW(), worldH(),
                UavObstacleGrid.obstacleGrid,
                gridCellM, gridW, gridH, snapLocationsToGrid));

        Coord goal = new Coord(targetX, targetY);
        if (snapLocationsToGrid)
            goal = UavPathUtils.snapToNearestFreeCell(
                goal, UavObstacleGrid.obstacleGrid, gridCellM, gridW, gridH);
        tour.add(goal);
        return tour;
    }

    // ================================================================ //
    //  Grid coordinate wrappers (thin instance-level convenience methods)
    // ================================================================ //

    private int[] worldToGrid(Coord c) {
        return UavPathUtils.worldToGrid(c, gridCellM, gridW, gridH);
    }

    private int cellIndex(int col, int row) {
        return UavPathUtils.cellIndex(col, row, gridW);
    }

    private boolean inBounds(int col, int row) {
        return UavPathUtils.inBounds(col, row, gridW, gridH);
    }

    private double worldW() { return getMaxX(); }
    private double worldH() { return getMaxY(); }

    private double sampleSpeed() {
        return speedMin + this.rng.nextDouble() * (speedMax - speedMin);
    }

    // ================================================================ //
    //  Public accessors
    // ================================================================ //

    /** Returns the current A* key-node list (unmodifiable). */
    public List<Coord> getKeyNodes() {
        return keyNodes == null ? Collections.emptyList()
                                : Collections.unmodifiableList(keyNodes);
    }

    public int         getKeyIndex()         { return keyIndex; }
    public int         getPoiIndex()         { return poiIndex; }
    public List<Coord> getPoiTour()          {
        return poiTour == null ? Collections.emptyList()
                               : Collections.unmodifiableList(poiTour);
    }
    public boolean isHovering()              { return hovering && SimClock.getTime() < hoverUntil; }
    public boolean isReplanPending()         { return replanNeeded; }
    public boolean isReachedFinalGoal()      { return reachedFinalGoal; }
    public int     getStallCount()           { return stallCount; }
    public Coord   getUavPos()               { return uavPos == null ? null : uavPos.clone(); }
    public int     getUavId()                { return uavId; }

    public Path    getTrailPath()            { return trailPath; }

    public static Map<Integer, double[]> getPeerRegistry() {
        return Collections.unmodifiableMap(PEER_REGISTRY);
    }

    public static Map<Integer, Path> getTrailRegistry() {
        return Collections.unmodifiableMap(TRAIL_REGISTRY);
    }

    public static Set<String> getLoadedWktFiles() {
        return Collections.unmodifiableSet(UavObstacleGrid.loadedWktFiles);
    }

    // ── GUI overlay accessors ─────────────────────────────────────────

    public static PlanningGridSnapshot getPlanningGridSnapshot() {
        return gridRenderingEnabled ? UavObstacleGrid.planningGridSnapshot : null;
    }

    public static List<ObstacleRenderData> getObstacleRenderData() {
        return UavObstacleGrid.getRenderData();
    }
}
