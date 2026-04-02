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
    /**
     * Optional explicit geographic bounding box: "minLon,minLat,maxLon,maxLat".
     * When set, this overrides the auto-detected bbox from the first WKT file and
     * ensures obstacle coordinates are scaled with the SAME reference frame that
     * ONE's MapBasedMovement uses when it renders the base map layer.
     *
     * How to find the right values: run the simulation once, note the campus
     * boundary extents from uoh_map.wkt (or from QGIS), and set:
     *   UAVWaypointMovement.geoExtents = minLon,minLat,maxLon,maxLat
     *
     * Example for UoH campus (Hyderabad):
     *   UAVWaypointMovement.geoExtents = 78.3300,17.4400,78.3700,17.4700
     */
    public static final String GEO_EXTENTS_S        = "geoExtents";

    /**
     * Comma-separated list of extra WKT files whose bounding boxes should be
     * included when computing the shared geographic reference frame.  This is
     * needed when ONE's MapBasedMovement map files (roads, highways, etc.) extend
     * beyond the obstacle WKT files, which would make ONE's map bbox larger than
     * the obstacle bbox and cause alignment shift.
     *
     * Set to the same files listed in MapBasedMovement.mapFile* that are
     * geographic (lon/lat).  The bbox is computed from ALL of these files plus
     * the obstacle files, then LOCKED — matching ONE's own map extent calculation.
     *
     * Example:
     *   UAVWaypointMovement.geoExtentFiles = samples/gdrrt/highwaysbarriers.wkt,
     *                                        samples/gdrrt/waterway.wkt
     */
    public static final String GEO_EXTENT_FILES_S   = "geoExtentFiles";

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
    private static final double DEF_VO_WEIGHT       = 2.0;
    private static final double DEF_LAUNCH_STAGGER  = 8.0;
    private static final double DEF_SPAWN_STRIDE    = 20.0;

    // ================================================================ //
    //  DWA adaptive weight sets  (paper Table 4)
    //  index: 0=α(heading) 1=β(vel) 2=λ(obs) 3=η(follow)
    // ================================================================ //

    private static final double[] W_FOLLOW = { 0.30, 0.30, 0.20, 0.20 };
    private static final double[] W_ALERT  = { 0.20, 0.30, 0.40, 0.10 };
    private static final double[] W_DANGER = { 0.20, 0.25, 0.50, 0.05 };

    private static final int STALL_LIMIT = 12;

    // ================================================================ //
    //  Per-instance parameters
    //  All read in the constructor via readDouble/Int/Boolean/String
    //  which apply the per-group → shared-namespace fallback chain.
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

    /**
     * True when this prototype's spawn was read from a per-group key
     * (GroupN.spawn) rather than the shared UAVWaypointMovement.spawn.
     * When true, the spawnStrideM offset is NOT applied in
     * getInitialLocation() — the config already placed the drone precisely.
     */
    private boolean perGroupSpawnSet = false;

    /**
     * WKT obstacle files resolved for this group.
     * May contain one or more paths parsed from a comma-separated config value.
     * All entries are union-merged into the shared obstacleGrid by
     * mergeObstaclesIfNeeded().
     */
    private List<String> groupObstacleWktFiles = new ArrayList<>();

    /**
     * Extra WKT files (e.g. road/map files) whose geographic extents are included
     * in the shared geo bbox calculation so that it matches ONE's MapBasedMovement bbox.
     * Set via UAVWaypointMovement.geoExtentFiles.
     */
    private List<String> geoExtentFiles = new ArrayList<>();

    /**
     * Explicitly-configured geographic bounding box [minLon, minLat, maxLon, maxLat].
     * When non-null, overrides auto-detection. Set via UAVWaypointMovement.geoExtents.
     */
    private double[] explicitGeoExtents = null;

    // ================================================================ //
    //  Shared static obstacle grid
    //  Allocated once; extended additively as each group's WKT loads.
    // ================================================================ //

    /** obstacleGrid[row][col] == true → cell is blocked. */
    private static boolean[][]    obstacleGrid   = null;
    /** POINT obstacles for DWA clearance queries: {cx, cy, radius} */
    private static List<double[]> obstacleDiscs  = null;
    /** LINESTRING obstacles for DWA: {x0, y0, x1, y1, halfWidth} */
    private static List<double[]> obstacleSegBuf = null;

    /**
     * Canonical paths of WKT files already rasterised.
     * Prevents double-loading when multiple groups share the same file.
     */
    private static final Set<String> loadedWktFiles
            = Collections.synchronizedSet(new LinkedHashSet<>());

    // ── Shared geographic bounding-box (Bug 1 & 2 fix) ────────────────────
    //
    // All WKT files that use geographic (lon/lat) coordinates must be
    // rescaled with the SAME affine transform so that their relative
    // spatial positions are preserved.  We compute one global bounding
    // box across every coordinate seen in any geographic file, store it
    // here on the first detection, and reuse it for every subsequent file.
    //
    // Fields are null/NaN until the first geographic file is loaded.
    // Once set they are never overwritten (subsequent files only expand the
    // box if their coordinates lie outside — but for a single-campus
    // scenario the first file (uoh_map.wkt) already captures the full
    // extents and later files (buildings, facilities) lie within it).
    //
    // Access is always inside the synchronized(UAVWaypointMovement.class)
    // block in mergeObstaclesIfNeededLocked(), so no extra lock needed.
    private static double geoMinX = Double.MAX_VALUE;
    private static double geoMaxX = -Double.MAX_VALUE;
    private static double geoMinY = Double.MAX_VALUE;
    private static double geoMaxY = -Double.MAX_VALUE;
    /** True once at least one geographic WKT file has been detected. */
    private static boolean geoScaleInitialised = false;

    // ================================================================ //
    //  Peer-UAV registry  (Velocity Obstacle)
    // ================================================================ //

    /**
     * Shared registry: key = uavId, value = {x, y, vx, vy}.
     * Written at the end of every getPath(); read during dwaStep().
     */
    private static final ConcurrentHashMap<Integer, double[]> PEER_REGISTRY
            = new ConcurrentHashMap<>();

    /**
     * Shared registry of per-UAV trail paths for GUI rendering.
     * Key = uavId, Value = the live accumulated trail Path object.
     *
     * PlayField clears its internal path list on every updateField() call,
     * so a path pushed from getPath() (sim thread, between GUI frames) is
     * wiped before it is ever rendered.  DTNSimGUI.updateView() polls this
     * registry every repaint and re-pushes all trails to PlayField, matching
     * the frequency at which PlayField actually renders them.
     */
    private static final ConcurrentHashMap<Integer, Path> TRAIL_REGISTRY
            = new ConcurrentHashMap<>();

    /**
     * Returns an unmodifiable view of all live UAV trail paths.
     * Called by DTNSimGUI.updateView() on every repaint cycle.
     */
    public static Map<Integer, Path> getTrailRegistry() {
        return Collections.unmodifiableMap(TRAIL_REGISTRY);
    }

    private static final AtomicInteger ID_COUNTER = new AtomicInteger(0);

    private int    uavId = -1;  // -1 = prototype; live IDs start at 0
    private double lastVx = 0.0, lastVy = 0.0;

    // ── GUI reference for path rendering — mirrors GDRRTPlanner pattern ──────
    /**
     * Set once by DTNSimGUI.initGUI() via {@link #setGui(gui.DTNSimGUI)}.
     * {@code null} when running headless (batch mode) — all gui.showPath()
     * calls are guarded with a null-check so headless runs are unaffected.
     */
    private static gui.DTNSimGUI gui = null;

    /**
     * Called by DTNSimGUI.initGUI() to register the GUI instance so that
     * getPath() can push the live trail to the PlayField renderer.
     * Identical pattern to GDRRTPlanner.setGui().
     *
     * @param g the running DTNSimGUI instance; {@code null} to detach
     */
    public static void setGui(gui.DTNSimGUI g) {
        gui = g;
    }

    // ================================================================ //
    //  Planning-grid snapshot  (GUI obstacle overlay)
    // ================================================================ //

    public static final class PlanningGridSnapshot {
        public final double      gridCellM;
        public final int         gridW, gridH;
        public final boolean[][] blocked;   // blocked[row][col]

        private PlanningGridSnapshot(double c, int w, int h, boolean[][] b) {
            this.gridCellM = c; this.gridW = w; this.gridH = h; this.blocked = b;
        }
    }

    /**
     * Written ONLY inside mergeObstaclesIfNeeded() and resetObstacleState().
     * NEVER written from the prototype constructor — doing so would blank the
     * GUI overlay each time a new group prototype is constructed, erasing
     * obstacles loaded by earlier groups.
     */
    private static PlanningGridSnapshot planningGridSnapshot = null;

    // ── FIX 2: Runtime grid-rendering toggle ─────────────────────────────────
    /**
     * Controls whether the planning grid is rendered in the GUI.
     * Can be toggled at runtime without restarting the simulation.
     * Initialised to true so the grid is visible by default.
     */
    private static boolean gridRenderingEnabled = true;

    /**
     * FIX 2 — Returns the planning-grid snapshot for GUI rendering, or
     * {@code null} when grid rendering has been disabled via
     * {@link #setGridRenderingEnabled(boolean)}.
     *
     * <p>The internal {@code obstacleGrid} used by A* and DWA is completely
     * unaffected — only the GUI overlay is hidden.
     */
    public static PlanningGridSnapshot getPlanningGridSnapshot() {
        return gridRenderingEnabled ? planningGridSnapshot : null;
    }

    /**
     * FIX 2 — Enable or disable the planning-grid GUI overlay at runtime.
     *
     * @param enabled {@code true} to show the grid, {@code false} to hide it.
     *                Planning and obstacle-avoidance are never affected.
     */
    public static void setGridRenderingEnabled(boolean enabled) {
        gridRenderingEnabled = enabled;
    }

    /**
     * FIX 2 — Returns the current state of the grid-rendering toggle.
     *
     * @return {@code true} if the grid overlay is currently visible.
     */
    public static boolean isGridRenderingEnabled() {
        return gridRenderingEnabled;
    }

    // ── FIX 3: Coordinate-based obstacle data for GUI rendering ──────────────
    /**
     * FIX 3 — Stores the original WKT coordinates for every obstacle so the
     * GUI can render smooth, accurate shapes independently of the planning grid.
     *
     * <p>Dual-storage design:
     * <ul>
     *   <li>{@code obstacleGrid}      — boolean[][] used by A* and DWA (unchanged)</li>
     *   <li>{@code obstacleRenderData} — coordinate list used by the GUI renderer</li>
     * </ul>
     *
     * <p>The render data is populated inside {@link #loadObstaclesFromWkt} at the
     * same time as grid rasterisation, so it is always consistent with the grid.
     * Unlike the grid, it is never cleared when {@link #setGridRenderingEnabled}
     * is called — obstacles remain visible even when the grid overlay is hidden.
     */
    public static final class ObstacleRenderData {
        /** Geometry type of this obstacle. */
        public enum Type { POINT, LINE, POLYGON }

        public final Type         type;
        /** Original WKT world coordinates (metres after geo-normalisation). */
        public final List<Coord>  coords;
        /**
         * For POINT obstacles: the buffer radius (= {@code pointObstacleRadius}).
         * For LINE obstacles : the half-width (= {@code lineObstacleHalfWidth}).
         * For POLYGON        : unused (0.0).
         */
        public final double       radius;

        public ObstacleRenderData(Type type, List<Coord> coords, double radius) {
            this.type   = type;
            this.coords = new ArrayList<>(coords);
            this.radius = radius;
        }
    }

    /**
     * FIX 3 — Shared list of all obstacles in their original coordinate form.
     * Populated by {@link #loadObstaclesFromWkt}; never cleared between groups.
     * {@code null} until the first WKT file is loaded.
     */
    private static List<ObstacleRenderData> obstacleRenderData = null;

    /**
     * FIX 3 — Returns a snapshot of all obstacle render data for GUI use.
     * Returns an empty list (never {@code null}) if no obstacles have been loaded.
     *
     * <p>This list is independent of {@link #getPlanningGridSnapshot()} — it
     * remains populated even when grid rendering is disabled.
     */
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

    // ── FIX 1: Final-goal arrival state ──────────────────────────────────────
    /**
     * FIX 1 — Distance threshold within which the drone is considered to have
     * reached the final goal and stops permanently.
     * Smaller than {@code gridCellM} for precise stopping; configurable via
     * {@code UAVWaypointMovement.finalGoalTolerance} or per-group override.
     */
    private static final double DEFAULT_FINAL_GOAL_TOLERANCE = 2.0;
    /** Config key for the final-goal tolerance (metres). */
    public  static final String FINAL_GOAL_TOLERANCE_S = "finalGoalTolerance";
    /** Per-instance tolerance value, read from settings in the constructor. */
    private double  finalGoalTolerance;

    /**
     * FIX 1 — Set to {@code true} once the drone arrives within
     * {@link #finalGoalTolerance} of its final mission goal.
     * When {@code true}, {@link #getPath()} immediately returns a zero-speed
     * path at the current position and skips all planning, preventing the
     * tour-restart jitter present in the original code.
     */
    private boolean reachedFinalGoal = false;

    /**
     * Accumulates every position the drone visits so the full traversed
     * trail can be rendered in the GUI as a single connected path.
     * Reset in getInitialLocation() on each new run.
     */
    private Path trailPath = null;

    // ================================================================ //
    //  Two-level settings helpers
    //  Check per-group Settings s first; fall back to shared cfg.
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

    /**
     * Reads a raw (non-CSV-split) setting value for keys whose value is itself
     * a comma-separated list of file paths.
     *
     * <p>ONE's {@code Settings.getSetting()} is a thin wrapper around
     * {@code getCsvSetting(key)[0]}, which silently discards everything after
     * the first comma.  That is exactly the wrong behaviour for
     * {@code obstacleWktFile} when multiple paths are listed on one line.
     *
     * <p>Fix: call {@code getCsvSetting(key)} — which returns ALL tokens —
     * and join them back with commas to reconstruct the full original value.
     * The subsequent {@link #parseObstacleFilePaths(String)} split then sees
     * the complete list and produces the correct per-path entries.
     *
     * <p>Falls back gracefully to {@code readString} if {@code getCsvSetting}
     * is unavailable (older ONE builds), preserving the single-file case.
     */
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
        } catch (Exception ignored) {
            // getCsvSetting not available — fall back to getSetting (single-file only)
        }
        return readString(s, cfg, key, "");
    }

    private static double[] readCsvDoubles(Settings s, Settings cfg,
                                           String key, int count) {
        if (s   != null && s.contains(key))   return s.getCsvDoubles(key, count);
        if (cfg != null && cfg.contains(key)) return cfg.getCsvDoubles(key, count);
        return null;
    }

    /**
     * Parses a comma-separated obstacle-file config value into an ordered list
     * of trimmed, non-empty path strings.
     *
     * <p>Handles all common formatting artefacts found in ONE settings files:
     * <ul>
     *   <li>Extra whitespace or tabs around each path</li>
     *   <li>Trailing/leading commas (produces no phantom empty entry)</li>
     *   <li>Multi-line values already concatenated into one long string</li>
     * </ul>
     *
     * <p>Example — the following config value:
     * <pre>
     *   Group1.obstacleWktFile = samples/obs.wkt, samples/buildings/b1.wkt,
     *                            samples/buildings/b2.wkt
     * </pre>
     * becomes {@code ["samples/obs.wkt", "samples/buildings/b1.wkt",
     * "samples/buildings/b2.wkt"]}.
     *
     * @param raw  raw value returned by {@code Settings.getSetting()} —
     *             may be {@code null} or empty
     * @return     ordered list of individual path strings; never {@code null}
     */
    static List<String> parseObstacleFilePaths(String raw) {
        List<String> result = new ArrayList<>();
        if (raw == null || raw.trim().isEmpty()) return result;
        for (String token : raw.split(",")) {
            String path = token.trim();
            if (!path.isEmpty()) result.add(path);
        }
        return result;
    }

    /**
     * Reads the {@value OBSTACLE_WKT_S} config key from {@code s}
     * (per-group) or {@code cfg} (shared namespace) and returns every path
     * it contains.  A single file and a comma-separated list are both
     * handled transparently.
     *
     * @return list of file-path strings; empty list when the key is absent
     */
    private static List<String> readObstacleFilePaths(Settings s, Settings cfg) {
        // Use readRawObstacleFileSetting instead of readString: ONE's getSetting()
        // internally returns getCsvSetting()[0] which silently drops all paths
        // after the first comma in a multi-file list.  readRawObstacleFileSetting
        // calls getCsvSetting() directly and re-joins all tokens so that
        // parseObstacleFilePaths receives the complete comma-separated value.
        String raw = readRawObstacleFileSetting(s, cfg, OBSTACLE_WKT_S);
        return parseObstacleFilePaths(raw);
    }

    // ================================================================ //
    //  Constructors
    // ================================================================ //

    /**
     * Called by ONE via reflection once per GroupN block.
     * {@code s} is the GroupN-scoped Settings object.
     *
     * Bug fixes applied here:
     *
     *  FIX 1 — Per-group config ignored.
     *    Previously every parameter was read exclusively from the shared
     *    UAVWaypointMovement.* namespace, so Group2.spawn, Group3.target,
     *    etc. were silently ignored.  All parameters now use readDouble/
     *    readInt/readBoolean/readCsvDoubles which check s (per-group) first
     *    and fall back to cfg (shared namespace) — giving each drone its own
     *    spawn/target/speed/dwell while still inheriting unset shared defaults.
     *
     *  FIX 2 — Snapshot overwrite blanks obstacle GUI.
     *    Previously the constructor called publishEmptyPlanningGridSnapshot()
     *    unconditionally.  With 3 groups, the constructors run in order:
     *    Group1 ctor → Group1.getInitialLocation() loads WKT, publishes
     *    snapshot → Group2 ctor calls publishEmptyPlanningGridSnapshot() →
     *    snapshot now shows NO obstacles even though obstacleGrid is intact.
     *    Fix: constructor does NOT touch planningGridSnapshot at all.
     *    The snapshot is written only by mergeObstaclesIfNeeded() and
     *    resetObstacleState().
     */
    public UAVWaypointMovement(Settings s) {
        super(s);
        Settings cfg = new Settings(SETTINGS_NS);

        // ── spawn / target — per-group first, then shared namespace ────
        double[] sp = readCsvDoubles(s, cfg, SPAWN_S, 2);
        double[] tg = readCsvDoubles(s, cfg, TARGET_S, 2);
        if (sp == null || tg == null) {
            throw new SimError("UAVWaypointMovement requires '"
                + SPAWN_S + "' and '" + TARGET_S
                + "' in GroupN.* or " + SETTINGS_NS + ".*");
        }
        spawnX = sp[0];  spawnY = sp[1];
        targetX = tg[0]; targetY = tg[1];

        // Track whether spawn came from the per-group block.
        // Used in getInitialLocation() to decide whether to apply spawnStrideM.
        perGroupSpawnSet = (s != null && s.contains(SPAWN_S));

        // ── movement parameters ──────────────────────────────────────
        dwellMin   = readDouble(s, cfg, DWELL_MIN_S,     DEF_DWELL_MIN);
        dwellMax   = readDouble(s, cfg, DWELL_MAX_S,     DEF_DWELL_MAX);
        speedMin   = readDouble(s, cfg, SPEED_MIN_S,     DEF_SPEED_MIN);
        speedMax   = readDouble(s, cfg, SPEED_MAX_S,     DEF_SPEED_MAX);
        cruiseAlt  = readDouble(s, cfg, CRUISE_ALT_S,    DEF_CRUISE_ALT);
        gridCellM  = readDouble(s, cfg, GRID_CELL_S,     DEF_GRID_CELL);
        dwaSteps   = readInt   (s, cfg, DWA_STEPS_S,     DEF_DWA_STEPS);
        distAlert  = readDouble(s, cfg, DIST_ALERT_S,    DEF_DIST_ALERT);
        distRisk   = readDouble(s, cfg, DIST_RISK_S,     DEF_DIST_RISK);

        // ── VO / multi-UAV parameters ────────────────────────────────
        uavSeparationM = readDouble(s, cfg, UAV_SEP_S,        DEF_UAV_SEP);
        voWeight       = readDouble(s, cfg, VO_WEIGHT_S,      DEF_VO_WEIGHT);
        launchStaggerS = readDouble(s, cfg, LAUNCH_STAGGER_S, DEF_LAUNCH_STAGGER);
        spawnStrideM   = readDouble(s, cfg, SPAWN_STRIDE_S,   DEF_SPAWN_STRIDE);

        // ── obstacle geometry ────────────────────────────────────────
        pointObstacleRadius   = readDouble(s, cfg, POINT_RADIUS_S,    gridCellM);
        lineObstacleHalfWidth = readDouble(s, cfg, LINE_HALF_WIDTH_S, gridCellM);

        // ── POI lattice ──────────────────────────────────────────────
        poiGridCols         = readInt    (s, cfg, POI_GRID_COLS_S, 0);
        poiGridRows         = readInt    (s, cfg, POI_GRID_ROWS_S, 0);
        snapLocationsToGrid = readBoolean(s, cfg, SNAP_TO_GRID_S,  true);

        // ── FIX 1: final-goal stopping tolerance ─────────────────────
        finalGoalTolerance  = readDouble (s, cfg, FINAL_GOAL_TOLERANCE_S,
                                          DEFAULT_FINAL_GOAL_TOLERANCE);

        // ── grid dimensions ──────────────────────────────────────────
        gridW = (int) Math.ceil(worldW() / gridCellM);
        gridH = (int) Math.ceil(worldH() / gridCellM);

        // ── obstacle WKT files for this group (single path or comma-separated list)
        groupObstacleWktFiles = readObstacleFilePaths(s, cfg);

        // ── optional explicit geographic bounding box ─────────────────────────
        // UAVWaypointMovement.geoExtents = minLon,minLat,maxLon,maxLat
        // Overrides auto-detection so obstacles scale with the same bbox that
        // ONE's MapBasedMovement uses for the base map layer.
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

        // ── extra WKT files for bbox computation (e.g. MapBasedMovement map files)
        String geoExtRaw = readRawObstacleFileSetting(s, cfg, GEO_EXTENT_FILES_S);
        geoExtentFiles = parseObstacleFilePaths(geoExtRaw);

        // ── MapBasedMovement-identical transform constants ─────────────────
        // These must match the hard-coded values in your MapBasedMovement.java:
        //   world_x = (lon  - geoOriginLon) * geoScale
        //   world_y = (geoOriginLat - lat)  * geoScale
        // Defaults mirror the provided MapBasedMovement.java (78.300, 17.480, 100000).
        geoOriginLon = readDouble(s, cfg, GEO_ORIGIN_LON_S, DEF_GEO_ORIGIN_LON);
        geoOriginLat = readDouble(s, cfg, GEO_ORIGIN_LAT_S, DEF_GEO_ORIGIN_LAT);
        geoScale     = readDouble(s, cfg, GEO_SCALE_S,       DEF_GEO_SCALE);

        // NOTE: do NOT write planningGridSnapshot here (see FIX 2 above).
    }

    /**
     * Copy constructor — called by ONE for every host in the group via
     * replicate().  Copies ALL per-instance parameters so each drone retains
     * its own spawn/target/speed values.
     */
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
        this.geoOriginLon          = proto.geoOriginLon;
        this.geoOriginLat          = proto.geoOriginLat;
        this.geoScale              = proto.geoScale;

        // FIX 1: each replicated host gets its own independent stopped-flag
        this.finalGoalTolerance    = proto.finalGoalTolerance;
        this.reachedFinalGoal      = false;   // fresh per-host state, not copied

        this.uavId = ID_COUNTER.getAndIncrement();
    }

    // ================================================================ //
    //  ONE MovementModel interface
    // ================================================================ //

    /**
     * Called once per host at simulation start.
     *
     * 1. mergeObstaclesIfNeeded():
     *      • Allocates obstacleGrid/Discs/SegBuf on the very first call.
     *      • Rasterises this group's WKT into the shared grid if not already done.
     *      • Publishes planningGridSnapshot after each new WKT load.
     *    After all groups have called getInitialLocation() the grid holds
     *    the UNION of every group's obstacles and the snapshot is correct.
     *
     * 2. Spawn position:
     *      • If GroupN.spawn was explicitly set → use spawnX/spawnY as-is.
     *      • Otherwise → apply uavId * spawnStrideM offset so drones sharing
     *        the UAVWaypointMovement.spawn fallback don't all start at the
     *        same point.
     */
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
        reachedFinalGoal = false;   // FIX 1: reset per-run so re-runs work
        trailPath    = new Path(sampleSpeed()); // reset trail on each new run
        trailPath.addWaypoint(uavPos.clone());
        planToNextPoi();
        hovering   = false;
        hoverUntil = 0;

        if (uavId >= 0)
            PEER_REGISTRY.put(uavId,
                new double[]{ uavPos.getX(), uavPos.getY(), 0.0, 0.0 });

        return uavPos.clone();
    }

    /**
     * Called by ONE each time the UAV completes a movement segment.
     *  1. FIX 1 — Early-exit if already at final goal (prevents jitter/looping).
     *  2. FIX 1 — Pre-move check: stop immediately if within finalGoalTolerance.
     *  3. Bresenham LOS check → replanNeeded if sub-goal blocked.
     *  4. Stall detection → force re-plan after STALL_LIMIT idle steps.
     *  5. DWA step with VO peer-avoidance penalty.
     *  6. Sub-goal / POI advancement — with FIX 1 guard replacing the original
     *     unconditional tour-reset that caused destination jitter.
     *  7. Publish position + velocity to PEER_REGISTRY.
     */
    @Override
    public Path getPath() {

        // ── FIX 1: CHECK 1 — Already stopped at goal? Stay put. ─────────────
        // Once reachedFinalGoal is true we return a zero-speed path immediately
        // on every subsequent call — no planning, no movement, no jitter.
        if (reachedFinalGoal) {
            Path path = new Path(0);
            path.addWaypoint(uavPos.clone());
            path.addWaypoint(uavPos.clone());
            return path;
        }

        // ── FIX 1: CHECK 2 — Are we close enough to the final goal right now?
        // This fires BEFORE the DWA step so the drone stops cleanly rather than
        // overshooting.  We guard by poiIndex so it only triggers on the last POI.
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

        // ── Normal planning ──────────────────────────────────────────────────
        if (keyNodes == null || keyNodes.isEmpty()) planToNextPoi();

        Coord subGoal = keyNodes.get(keyIndex);
        if (!bresenhamLOS(uavPos, subGoal)) replanNeeded = true;

        if (replanNeeded) {
            replanNeeded = false; stallCount = 0;
            planToNextPoi();
            subGoal = keyNodes.get(keyIndex);
        }

        double prevX = uavPos.getX(), prevY = uavPos.getY();
        Coord nextPos = dwaStep(uavPos, subGoal);

        if (nextPos.distance(uavPos) < gridCellM * 0.1) {
            if (++stallCount >= STALL_LIMIT) {
                stallCount = 0; planToNextPoi();
                subGoal  = keyNodes.get(keyIndex);
                nextPos  = dwaStep(uavPos, subGoal);
            }
        } else {
            stallCount = 0;
        }

        if (nextPos.distance(subGoal) < gridCellM) {
            uavPos = subGoal.clone();
            if (++keyIndex >= keyNodes.size()) {
                if (++poiIndex >= poiTour.size()) {
                    // ── FIX 1: CHECK 3 — Reached the last POI in the tour. ──
                    // ORIGINAL CODE: unconditionally reset poiIndex=0 and
                    //   rebuild the tour, which caused the drone to loop back
                    //   to its first waypoint and jitter around the goal.
                    // FIX: only restart the tour if we are NOT within
                    //   finalGoalTolerance — otherwise stop permanently.
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
                        // Intermediate goal reached but not at final destination —
                        // restart the tour from the current position (original behaviour).
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

        // Accumulate the traversed trail for GUI rendering.
        if (trailPath == null) {
            trailPath = new Path(sampleSpeed());
            trailPath.addWaypoint(new Coord(prevX, prevY));
        }
        trailPath.addWaypoint(uavPos.clone());

        // Return a two-waypoint segment: previous → current.
        // ONE's PlayField renderer draws a line between consecutive waypoints,
        // so a single-waypoint path (the old code) produced nothing visible.
        Path path = new Path(sampleSpeed());
        path.addWaypoint(new Coord(prevX, prevY));
        path.addWaypoint(uavPos.clone());

        // Register trail in shared registry.
        // DTNSimGUI.updateView() re-pushes all trails to PlayField on every
        // repaint — necessary because PlayField clears its path list each
        // updateField() call, so a one-shot push from the sim thread (here)
        // is wiped before it is ever rendered.
        if (uavId >= 0 && trailPath != null) {
            TRAIL_REGISTRY.put(uavId, trailPath);
        }

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

            // Build the natural pruned neighbour set for this expansion direction.
            java.util.Set<Integer> toExpand = new java.util.LinkedHashSet<>();
            for (int di : getActiveNeighbours(parentDir[idx])) toExpand.add(di);

            // ── Forced-neighbour detection (paper Fig. 1c/d) ────────────────
            // If a cell lateral to the current expansion direction is blocked, the
            // diagonal beyond it cannot be reached more cheaply without passing
            // through the current node — so it becomes a "forced" successor and
            // must be added to the expansion set even if pruning would skip it.
            int pd = parentDir[idx];
            if (pd == 0) { // East: check N and S laterals
                if (inBounds(col, row+1) && obstacleGrid[row+1][col]) toExpand.add(4); // force NE
                if (inBounds(col, row-1) && obstacleGrid[row-1][col]) toExpand.add(5); // force SE
            } else if (pd == 1) { // West: check N and S
                if (inBounds(col, row+1) && obstacleGrid[row+1][col]) toExpand.add(6); // force NW
                if (inBounds(col, row-1) && obstacleGrid[row-1][col]) toExpand.add(7); // force SW
            } else if (pd == 2) { // North: check E and W
                if (inBounds(col+1, row) && obstacleGrid[row][col+1]) toExpand.add(4); // force NE
                if (inBounds(col-1, row) && obstacleGrid[row][col-1]) toExpand.add(6); // force NW
            } else if (pd == 3) { // South: check E and W
                if (inBounds(col+1, row) && obstacleGrid[row][col+1]) toExpand.add(5); // force SE
                if (inBounds(col-1, row) && obstacleGrid[row][col-1]) toExpand.add(7); // force SW
            } else if (pd == 4) { // NE: check S and W laterals
                if (inBounds(col-1, row) && obstacleGrid[row][col-1]) toExpand.add(6); // force NW
                if (inBounds(col, row-1) && obstacleGrid[row-1][col]) toExpand.add(5); // force SE
            } else if (pd == 5) { // SE: check N and W
                if (inBounds(col-1, row) && obstacleGrid[row][col-1]) toExpand.add(7); // force SW
                if (inBounds(col, row+1) && obstacleGrid[row+1][col]) toExpand.add(4); // force NE
            } else if (pd == 6) { // NW: check S and E
                if (inBounds(col+1, row) && obstacleGrid[row][col+1]) toExpand.add(4); // force NE
                if (inBounds(col, row-1) && obstacleGrid[row-1][col]) toExpand.add(7); // force SW
            } else if (pd == 7) { // SW: check N and E
                if (inBounds(col+1, row) && obstacleGrid[row][col+1]) toExpand.add(5); // force SE
                if (inBounds(col, row+1) && obstacleGrid[row+1][col]) toExpand.add(6); // force NW
            }

            for (int di : toExpand) {
                int nc = col+dirs[di][0], nr = row+dirs[di][1];
                if (!inBounds(nc,nr) || obstacleGrid[nr][nc]) continue;
                // Block diagonal moves that cut through a corner obstacle.
                if (di >= 4 && (obstacleGrid[row][nc] || obstacleGrid[nr][col])) continue;
                int ni = cellIndex(nc, nr);
                if (closed[ni]) continue;
                double tg = gCost[idx] + moveCost[di] * gridCellM;
                if (tg < gCost[ni]) {
                    gCost[ni] = tg; parentIdx[ni] = idx; parentDir[ni] = di;
                    open.offer(new double[]{ tg + adaptiveHeuristic(
                        new int[]{nc,nr}, goal), nc, nr });
                }
            }
        }
        return null;
    }

    private double adaptiveHeuristic(int[] n, int[] g) {
        double manhattan = (Math.abs(n[0]-g[0]) + Math.abs(n[1]-g[1])) * gridCellM;
        double xi = obstacleCoverageRate(n, g);
        return (1.0 + 1.0 / (1.0 + Math.exp(-xi))) * manhattan;
    }

    private double obstacleCoverageRate(int[] n, int[] g) {
        int c0=Math.min(n[0],g[0]), c1=Math.max(n[0],g[0]);
        int r0=Math.min(n[1],g[1]), r1=Math.max(n[1],g[1]);
        int tot = Math.max(1,(c1-c0+1)*(r1-r0+1)), obs = 0;
        for (int r=r0; r<=r1; r++)
            for (int c=c0; c<=c1; c++)
                if (inBounds(c,r) && obstacleGrid[r][c]) obs++;
        return (double)obs/tot;
    }

    /**
     * Returns the set of neighbour direction indices to expand from the
     * current node, given the direction {@code d} from which the current
     * node was reached (paper Fig. 1 — adjacency node clipping rule).
     *
     * Direction encoding (matches dirs[][] in aStarSearch):
     *   0:{+1, 0}  East      4:{+1,+1} NE
     *   1:{-1, 0}  West      5:{+1,-1} SE
     *   2:{ 0,+1}  North     6:{-1,+1} NW
     *   3:{ 0,-1}  South     7:{-1,-1} SW
     *
     * Cardinal step (d=0..3): forced successor is direction d itself;
     *   obstacle-forced extra neighbours are identified in the main loop
     *   by checking obstacleGrid, so here we return only {d} plus the
     *   two flanking diagonals that cannot be reached more cheaply
     *   without passing through the current node.
     *
     * Diagonal step (d=4..7): keep the diagonal itself plus the two
     *   adjacent cardinals.  The other diagonal and the two "back"
     *   cardinals are always reachable more cheaply via the parent.
     *
     * d < 0 (start node): expand all 8 directions.
     */
    private int[] getActiveNeighbours(int d) {
        // Start node — no parent direction, expand all 8.
        if (d < 0) return new int[]{ 0,1,2,3,4,5,6,7 };
        switch (d) {
            // ── Cardinal directions ──────────────────────────────────────
            // Forward cardinal + the two diagonals that flank it.
            case 0: return new int[]{ 0, 4, 5 };   // E  → E, NE, SE
            case 1: return new int[]{ 1, 6, 7 };   // W  → W, NW, SW
            case 2: return new int[]{ 2, 4, 6 };   // N  → N, NE, NW
            case 3: return new int[]{ 3, 5, 7 };   // S  → S, SE, SW
            // ── Diagonal directions ──────────────────────────────────────
            // Diagonal + the two adjacent cardinals.
            case 4: return new int[]{ 4, 0, 2 };   // NE → NE, E, N
            case 5: return new int[]{ 5, 0, 3 };   // SE → SE, E, S
            case 6: return new int[]{ 6, 1, 2 };   // NW → NW, W, N
            case 7: return new int[]{ 7, 1, 3 };   // SW → SW, W, S
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
        double d = nearestObstacleDistance(pos);
        double[] w = d<=distRisk ? W_DANGER : d<=distAlert ? W_ALERT : W_FOLLOW;
        double alpha=w[0], beta=w[1], lambda=w[2], eta=w[3];

        double bestScore = Double.NEGATIVE_INFINITY;
        Coord bestPos    = pos.clone();
        double step      = Math.min(gridCellM*1.5, pos.distance(subGoal));
        if (step < 0.1) step = 0.1;
        double idBias    = (uavId%2==0) ? 0.04 : -0.04;

        for (int i=0; i<=dwaSteps; i++) {
            double base  = Math.atan2(subGoal.getY()-pos.getY(), subGoal.getX()-pos.getX());
            double off   = dwaSteps==0 ? 0.0 : ((double)i/dwaSteps-0.5)*Math.PI;
            double angle = base+off;
            double v     = speedMin + (double)i/Math.max(1,dwaSteps)*(speedMax-speedMin);

            Coord cand = clampToWorld(new Coord(
                pos.getX()+step*Math.cos(angle), pos.getY()+step*Math.sin(angle)));
            int[] cell = worldToGrid(cand);
            if (obstacleGrid[cell[1]][cell[0]]) continue;

            double bear   = Math.atan2(subGoal.getY()-cand.getY(), subGoal.getX()-cand.getX());
            double heading= Math.PI - Math.abs(normaliseAngle(bear-angle));
            double vel    = (v-speedMin)/Math.max(1e-9, speedMax-speedMin);
            double dObsS  = Math.min(1.0, nearestObstacleDistance(cand)/distAlert);
            double dFolS  = 1.0 - Math.min(1.0,
                pointToSegmentDist(cand,pos,subGoal)/(gridCellM*5));

            double myVx=cand.getX()-pos.getX(), myVy=cand.getY()-pos.getY();
            double voScore=0; boolean voFired=false;
            for (double[] peer : PEER_REGISTRY.values()) {
                double px=peer[0], py=peer[1], pvx=peer[2], pvy=peer[3];
                double sd=Math.hypot(px-pos.getX(), py-pos.getY());
                if (sd<0.5 || sd>uavSeparationM*4) continue;
                double rx=px-pos.getX(), ry=py-pos.getY();
                if (sd<=uavSeparationM) { voScore-=voWeight*2; voFired=true; continue; }
                double rvx=myVx-pvx, rvy=myVy-pvy, rvm=Math.hypot(rvx,rvy);
                if (rvm<1e-9) continue;
                double sinT=uavSeparationM/sd, cosT=Math.sqrt(Math.max(0,1-sinT*sinT));
                double cosA=(rvx*rx+rvy*ry)/(rvm*sd);
                if (cosA>cosT) {
                    double pen=(cosA-cosT)/Math.max(1e-9,1-cosT);
                    double prox=1-Math.min(1,sd/(uavSeparationM*4));
                    voScore-=voWeight*pen*(1+prox); voFired=true;
                }
            }
            double tb = voFired ? idBias*Math.signum(off+1e-12) : 0;
            double score = alpha*heading+beta*vel+lambda*dObsS+eta*dFolS+voScore+tb;
            if (score>bestScore) { bestScore=score; bestPos=cand; }
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

    /**
     * Initialises shared obstacle structures (once) and then rasterises
     * this group's WKT file into them if not already done.
     *
     * Synchronised — safe for parallel getInitialLocation() calls.
     *
     * Fixes:
     *  • obstacleGrid is zeroed ONLY when null (first call). Later calls
     *    only ADD cells; they never clear what previous groups loaded.
     *  • planningGridSnapshot is published here (and only here), so it
     *    always reflects the current union of all loaded obstacles —
     *    never blanked by a constructor call.
     *  • loadedWktFiles.add() returns false for already-loaded paths →
     *    no double-rasterisation when groups share the same file.
     */
    private void mergeObstaclesIfNeeded() {
        // Synchronize on the CLASS (not `this`) so that concurrent calls from
        // different group prototype instances (which hold different instance locks)
        // are still serialized.  Using `synchronized(this)` above was a bug:
        // each group prototype is a separate object, so they never contended and
        // two threads could both see obstacleGrid==null and double-initialize.
        synchronized (UAVWaypointMovement.class) {
        mergeObstaclesIfNeededLocked();
        }
    }

    private void mergeObstaclesIfNeededLocked() {
        // Allocate shared structures exactly once (first call).
        if (obstacleGrid == null) {
            obstacleGrid      = new boolean[gridH][gridW];
            obstacleDiscs     = new ArrayList<>();
            obstacleSegBuf    = new ArrayList<>();
            obstacleRenderData = new ArrayList<>();   // FIX 3
            publishPlanningGridSnapshot();   // valid empty snapshot from frame 0
        }

        if (groupObstacleWktFiles == null || groupObstacleWktFiles.isEmpty()) return;

        boolean anyNewFile = false;

        for (String wkt : groupObstacleWktFiles) {
            if (wkt == null || wkt.trim().isEmpty()) continue;
            String trimmed = wkt.trim();

            // Canonicalise path to de-duplicate across groups.
            // NOTE: toRealPath() throws NoSuchFileException when the file does
            // not exist, so we resolve to an absolute normalised path first and
            // only follow symlinks (toRealPath) when the file is present.
            String canonical;
            try {
                java.nio.file.Path p = java.nio.file.Paths.get(trimmed);
                if (!p.isAbsolute())
                    p = java.nio.file.Paths.get(System.getProperty("user.dir", ".")).resolve(p);
                p = p.normalize();   // collapse ./ ../ without touching the FS
                // Resolve symlinks only when the file already exists; otherwise
                // the normalised absolute path is a good-enough de-dup key and
                // the missing-file error will be reported by loadObstaclesFromWkt.
                canonical = java.nio.file.Files.exists(p) ? p.toRealPath().toString()
                                                           : p.toString();
            } catch (IOException e) {
                canonical = trimmed;   // fallback: use raw path as de-dup key
            }

            if (!loadedWktFiles.add(canonical)) continue;  // already rasterised — skip

            loadObstaclesFromWkt(trimmed);
            anyNewFile = true;
        }

        if (anyNewFile) {
            publishPlanningGridSnapshot();   // update GUI once after all new files are loaded
        }
    } // end mergeObstaclesIfNeededLocked

    /** Publishes a deep-copy snapshot of the current obstacleGrid for the GUI. */
    private void publishPlanningGridSnapshot() {
        boolean[][] copy = new boolean[gridH][gridW];
        for (int r=0; r<gridH; r++)
            System.arraycopy(obstacleGrid[r], 0, copy[r], 0, gridW);
        planningGridSnapshot = new PlanningGridSnapshot(gridCellM, gridW, gridH, copy);
    }

    /**
     * Resets all shared static state between scenario runs (without JVM restart).
     * Must be called before re-running a scenario; otherwise loadedWktFiles
     * retains stale entries and mergeObstaclesIfNeeded() skips every file.
     * Also resets the explicit and auto-detected geographic bounding box so the
     * new run re-computes it from geoExtents / geoExtentFiles as configured.
     */
    public static synchronized void resetObstacleState() {
        obstacleGrid         = null;
        obstacleDiscs        = null;
        obstacleSegBuf       = null;
        obstacleRenderData   = null;    // FIX 3
        planningGridSnapshot = null;
        gridRenderingEnabled = true;    // FIX 2: restore default on re-run
        loadedWktFiles.clear();
        PEER_REGISTRY.clear();
        ID_COUNTER.set(0);
        // Reset shared geographic bounding box so re-runs start clean.
        geoMinX = Double.MAX_VALUE;  geoMaxX = -Double.MAX_VALUE;
        geoMinY = Double.MAX_VALUE;  geoMaxY = -Double.MAX_VALUE;
        geoScaleInitialised = false;
    }

    // ================================================================ //
    //  Self-contained WKT obstacle loader
    //
    //  Replaces the external WktObstacleParser dependency.
    //
    //  Supported geometry types (case-insensitive):
    //    POINT, MULTIPOINT
    //    LINESTRING, MULTILINESTRING
    //    POLYGON, MULTIPOLYGON
    //    GEOMETRYCOLLECTION
    //
    //  All compound types are decomposed to their primitive
    //  POINT / LINESTRING components before rasterisation.
    //  POLYGON rings are treated as closed LINESTRING obstacles
    //  (exterior ring + all interior rings), which is the correct
    //  treatment for building footprints in a 2-D grid planner.
    //
    //  Geometry tags with a Z or M suffix (e.g. LINESTRING Z,
    //  POINTM) are accepted; the extra ordinate is silently dropped.
    //
    //  Each non-blank, non-comment line in the file is expected to
    //  contain exactly one WKT geometry.  Lines beginning with '#'
    //  or '//' are treated as comments and skipped.
    // ================================================================ //

    /**
     * Loads one WKT file and rasterises all contained obstacles into the
     * shared grid.  Handles every geometry type encountered in real-world
     * OpenStreetMap exports (MULTILINESTRING, POLYGON, MULTIPOLYGON, …).
     *
     * @param rel  path as it appears in the settings file (relative or absolute)
     * @throws SimError on I/O failure or a truly un-parseable geometry token
     */
    /**
     * Charsets tried in order when reading a WKT file.
     * ISO-8859-1 never throws MalformedInputException, so it acts as a
     * guaranteed fallback for any byte sequence including Windows-1252 files.
     */
    private static final java.nio.charset.Charset[] WKT_CHARSETS = {
        java.nio.charset.StandardCharsets.UTF_8,
        java.nio.charset.Charset.forName("Windows-1252"),
        java.nio.charset.StandardCharsets.ISO_8859_1
    };

    private void loadObstaclesFromWkt(String rel) {
        // ── resolve path ────────────────────────────────────────────────────
        java.nio.file.Path path = java.nio.file.Paths.get(rel);
        if (!path.isAbsolute())
            path = java.nio.file.Paths.get(System.getProperty("user.dir", ".")).resolve(path);

        // ── existence / readability pre-check ──────────────────────────────
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

        // ── read lines — try each charset until one succeeds ───────────────
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

        // ── parse geometry ─────────────────────────────────────────────────
        List<double[]>       points      = new ArrayList<>();
        List<List<double[]>> linestrings = new ArrayList<>();
        // polygons: exterior rings of POLYGON/MULTIPOLYGON geometries.
        // Stored separately so they can be solid-filled on the grid rather
        // than rasterised as thin edge strips (the bug: buildings appeared as
        // scattered border dots instead of filled solid rectangles).
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

        // ── coordinate-space normalisation ─────────────────────────────────
        // WKT files exported from GIS tools (QGIS, PostGIS, OSM) use geographic
        // lon/lat coordinates (e.g. 78.33 17.45 for Hyderabad).  The A* grid is
        // in simulation metres (0..worldW × 0..worldH).  If the raw coordinates
        // fall in a typical lon/lat range we rescale them linearly onto the world
        // bounding box so obstacles appear correctly distributed across the grid.
        //
        // Detection heuristic: if the bounding box of all extracted coordinates
        // has both dimensions ≤ 1.0 AND min-x is in [−180, 180] we treat the
        // data as geographic and scale it.  Pure-metre files always span hundreds
        // of metres (>> 1.0 in at least one axis), so false-positives are
        // extremely unlikely.
        normaliseCoordinatesIfGeographic(points, linestrings, polygons);

        // ── rasterise ──────────────────────────────────────────────────────
        for (double[] pt : points) {
            rasterizeDiscOnGrid(pt[0], pt[1], pointObstacleRadius);
            obstacleDiscs.add(new double[]{ pt[0], pt[1], pointObstacleRadius });
            // FIX 3: store original coord for smooth GUI rendering
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
            // FIX 3: store whole linestring as one render entry
            if (lsCoords != null) {
                for (double[] p : ls) lsCoords.add(new Coord(p[0], p[1]));
                if (lsCoords.size() >= 2)
                    obstacleRenderData.add(new ObstacleRenderData(
                        ObstacleRenderData.Type.LINE, lsCoords, lineObstacleHalfWidth));
            }
        }
        // Polygon exterior rings: solid-fill every grid cell whose centre lies
        // inside the ring.  This is the fix for buildings appearing as scattered
        // border dots — they now render as solid filled rectangles on the grid,
        // matching the building footprints visible in the ONE GUI map overlay.
        for (List<double[]> ring : polygons) {
            rasterizeFilledPolygon(ring);
            // Also store edges in obstacleSegBuf so DWA clearance queries work.
            for (int i = 0; i < ring.size() - 1; i++) {
                double[] a = ring.get(i), b = ring.get(i + 1);
                obstacleSegBuf.add(new double[]{ a[0], a[1], b[0], b[1], 0.0 });
            }
            // FIX 3: store polygon for smooth GUI rendering
            if (obstacleRenderData != null) {
                List<Coord> pgCoords = new ArrayList<>(ring.size());
                for (double[] p : ring) pgCoords.add(new Coord(p[0], p[1]));
                if (pgCoords.size() >= 3)
                    obstacleRenderData.add(new ObstacleRenderData(
                        ObstacleRenderData.Type.POLYGON, pgCoords, 0.0));
            }
        }
    }

    /**
     * Detects whether the extracted coordinates appear to be geographic (lon/lat)
     * and transforms them to world simulation metres using the IDENTICAL formula
     * that ONE's MapBasedMovement uses in its {@code readMap()} method:
     *
     * <pre>
     *   world_x = (lon  - GEO_ORIGIN_LON) * GEO_SCALE
     *   world_y = (GEO_ORIGIN_LAT - lat)  * GEO_SCALE    // Y-axis flipped
     * </pre>
     *
     * <h3>Root cause of the previous misalignment</h3>
     * <p>MapBasedMovement hard-codes a fixed geographic origin and a single
     * uniform scale factor, and it <strong>flips the Y axis</strong>
     * (latitude is subtracted FROM the origin constant, not added to it).
     * The previous implementation used two independent scale factors
     * ({@code worldW/lonSpan} and {@code worldH/latSpan}) with no Y flip,
     * producing three simultaneous errors:
     * <ol>
     *   <li>Wrong origin — obstacles shifted relative to the base map.</li>
     *   <li>Wrong / non-uniform scale — obstacles stretched or squashed.</li>
     *   <li>Missing Y flip — obstacles mirrored vertically.</li>
     * </ol>
     *
     * <h3>Fix</h3>
     * <p>Mirror MapBasedMovement exactly.  The two constants below
     * ({@link #GEO_ORIGIN_LON}, {@link #GEO_ORIGIN_LAT}, {@link #GEO_SCALE})
     * must match the values hard-coded in your project's MapBasedMovement.java:
     * <pre>
     *   world_x = (lon  - 78.300) * 100000
     *   world_y = (17.480 - lat)  * 100000
     * </pre>
     * Override them via the config keys {@code UAVWaypointMovement.geoOriginLon},
     * {@code UAVWaypointMovement.geoOriginLat}, and {@code UAVWaypointMovement.geoScale}
     * if your MapBasedMovement uses different constants.
     */

    // ── Constants that mirror MapBasedMovement.readMap() ─────────────────────
    // Default values match the hard-coded transform in the provided
    // MapBasedMovement.java:
    //   world_x = (lon  - 78.300) * 100000.0
    //   world_y = (17.480 - lat)  * 100000.0
    //
    // If your MapBasedMovement uses different values, override via config:
    //   UAVWaypointMovement.geoOriginLon = 78.300
    //   UAVWaypointMovement.geoOriginLat = 17.480
    //   UAVWaypointMovement.geoScale     = 100000.0
    private static final double DEF_GEO_ORIGIN_LON = 78.300;
    private static final double DEF_GEO_ORIGIN_LAT = 17.480;
    private static final double DEF_GEO_SCALE       = 100000.0;

    /** Config key for the longitude subtracted in MapBasedMovement (default 78.300). */
    public static final String GEO_ORIGIN_LON_S = "geoOriginLon";
    /** Config key for the latitude subtracted FROM in MapBasedMovement (default 17.480). */
    public static final String GEO_ORIGIN_LAT_S = "geoOriginLat";
    /** Config key for the uniform scale multiplier in MapBasedMovement (default 100000). */
    public static final String GEO_SCALE_S       = "geoScale";

    /**
     * Per-instance values read from config in the constructor (or set to defaults).
     * Stored as instance fields so the copy-constructor propagates them correctly.
     */
    private double geoOriginLon = DEF_GEO_ORIGIN_LON;
    private double geoOriginLat = DEF_GEO_ORIGIN_LAT;
    private double geoScale     = DEF_GEO_SCALE;

    private void normaliseCoordinatesIfGeographic(List<double[]>       points,
                                                   List<List<double[]>> linestrings,
                                                   List<List<double[]>> polygons) {
        // ── Compute bounding box of coordinates in THIS file ──────────────
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

        if (fileMinX == Double.MAX_VALUE) return;  // no coordinates at all

        double spanX = fileMaxX - fileMinX, spanY = fileMaxY - fileMinY;

        // ── Geographic detection heuristic ────────────────────────────────
        // Both span dimensions ≤ 5 degrees AND coordinates are in valid lon/lat
        // ranges.  Pure-metre simulation files always span hundreds of metres
        // in at least one axis, so false positives are essentially impossible.
        boolean isGeographic = (spanX <= 5.0 && spanY <= 5.0)
                && (fileMinX >= -180.0 && fileMinX <= 180.0)
                && (fileMinY >= -90.0  && fileMinY <= 90.0);
        if (!isGeographic) return;

        if (!geoScaleInitialised) {
            geoScaleInitialised = true;
            System.out.printf("[UAVWaypointMovement] Using MapBasedMovement-identical transform:"
                + " world_x=(lon-%.3f)*%.0f  world_y=(%.3f-lat)*%.0f%n",
                geoOriginLon, geoScale, geoOriginLat, geoScale);
        }

        // ── Apply the SAME transform as MapBasedMovement.readMap() ────────
        //   world_x = (lon  - geoOriginLon) * geoScale
        //   world_y = (geoOriginLat - lat)  * geoScale   ← Y flipped, matches MapBasedMovement
        for (double[] p : points) {
            double lon = p[0], lat = p[1];
            p[0] = (lon - geoOriginLon) * geoScale;
            p[1] = (geoOriginLat - lat) * geoScale;
        }
        for (List<double[]> ls : linestrings)
            for (double[] p : ls) {
                double lon = p[0], lat = p[1];
                p[0] = (lon - geoOriginLon) * geoScale;
                p[1] = (geoOriginLat - lat) * geoScale;
            }
        for (List<double[]> pg : polygons)
            for (double[] p : pg) {
                double lon = p[0], lat = p[1];
                p[0] = (lon - geoOriginLon) * geoScale;
                p[1] = (geoOriginLat - lat) * geoScale;
            }
    }

    /**
     * Parses one WKT geometry string and appends extracted primitives to the
     * supplied accumulator lists.  Recursively handles collection types.
     *
     * @param wkt         a single WKT geometry string (may contain nested geometries)
     * @param points      accumulator for POINT coordinates
     * @param linestrings accumulator for LINESTRING / ring coordinate sequences
     */
    private static void wktExtractGeometry(String wkt,
                                           List<double[]>       points,
                                           List<List<double[]>> linestrings,
                                           List<List<double[]>> polygons) {
        // ── normalise ──────────────────────────────────────────────────────
        // Upper-case for type matching; collapse runs of whitespace.
        String upper = wkt.trim().replaceAll("\\s+", " ").toUpperCase(java.util.Locale.ROOT);

        // Strip optional dimension qualifiers: LINESTRING Z, POINT M, etc.
        // These appear between the type keyword and the opening parenthesis.
        upper = upper.replaceAll("\\b(Z M|ZM|M|Z)\\s*(?=\\()", "");

        // ── dispatch by geometry type ──────────────────────────────────────
        if (upper.startsWith("GEOMETRYCOLLECTION")) {
            // Peel off the outer GEOMETRYCOLLECTION(...) wrapper and recurse
            // on each comma-separated member geometry.
            String inner = extractOuterBody(wkt);
            if (inner == null) return;
            for (String member : splitTopLevelCommas(inner)) {
                wktExtractGeometry(member.trim(), points, linestrings, polygons);
            }

        } else if (upper.startsWith("MULTIPOLYGON")) {
            // MULTIPOLYGON (((ring),(ring)),((ring)))
            // Split at the boundary between adjacent polygon blocks: )),((
            String inner = extractOuterBody(wkt);
            if (inner == null) return;
            for (String polyBody : splitTopLevelCommas(inner)) {
                wktExtractGeometry("POLYGON " + polyBody.trim(), points, linestrings, polygons);
            }

        } else if (upper.startsWith("POLYGON")) {
            // POLYGON ((exterior),(hole1),(hole2),...)
            // Exterior ring → solid-filled polygon obstacle.
            // Hole rings are ignored (holes are passable interior space).
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
                        // Exterior ring: solid fill
                        polygons.add(ring);
                    }
                    // Hole rings (ri > 0) are skipped — the solid fill already
                    // covers them but holes in building footprints are rare in
                    // OSM exports and small enough to ignore for path planning.
                }
            }

        } else if (upper.startsWith("MULTILINESTRING") || upper.startsWith("MULTISTRING")) {
            // MULTILINESTRING ((p0 p1,...),(p0 p1,...),...)
            // Also handles the non-standard "MULTISTRING" alias emitted by some WKT exporters.
            //
            // extractOuterBody() returns the content between the OUTERMOST parens, e.g.:
            //   "(x0 y0, x1 y1,...),(x0 y0,...)"   ← each ring still wrapped in ()
            //
            // splitTopLevelCommas() then splits at depth-0 commas to give one token
            // per ring.  Each token is "(x0 y0, x1 y1,...)" — we strip the wrapping
            // parens before passing to parseCoordSequence().
            //
            // Edge case: if the WKT has no space between rings — ")(" — the comma-splitter
            // still works because it only fires at depth 0.  The per-ring strip below
            // handles both "( x y, ... )" and " x y, ... " (already stripped upstream).
            String inner = extractOuterBody(wkt);
            if (inner == null) return;
            for (String lsBody : splitTopLevelCommas(inner)) {
                String stripped = lsBody.trim();
                // Strip exactly one layer of outer parens if present.
                // Do NOT use startsWith+endsWith alone — ensure the parens are balanced
                // so that "(a)(b)" (malformed input) doesn't silently eat a closing paren.
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
            // MULTIPOINT ((x y),(x y),...) or MULTIPOINT (x y, x y, ...)
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
            // Unknown / unsupported type — log a warning and continue rather
            // than aborting the whole simulation.
            System.err.println("[UAVWaypointMovement] Skipping unsupported WKT type: "
                    + upper.split("\\s+|\\(")[0]);
        }
    }

    /**
     * Returns the content inside the outermost parentheses of a WKT string,
     * preserving all inner nesting.
     * <p>Example: {@code "LINESTRING (1 2, 3 4)"} → {@code "1 2, 3 4"}
     *
     * <p>Implementation uses a balanced-parenthesis scan rather than
     * {@code lastIndexOf(')')} so that trailing whitespace or any characters
     * after the closing paren of the outermost group are not mis-included and
     * multi-ring geometries like POLYGON are split correctly.
     *
     * @return the inner string, or {@code null} if no balanced parentheses are found
     */
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
        return null;  // unbalanced parentheses — treat as unparseable
    }

    /**
     * Splits a string on commas that are NOT inside any parentheses.
     * This correctly separates ring / member-geometry lists in compound types
     * without breaking coordinate pairs that also use commas.
     */
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

    /**
     * Parses a flat comma-separated coordinate sequence such as
     * {@code "1.0 2.0, 3.0 4.0, 5.0 6.0"} into a list of {@code [x, y]} pairs.
     * Z (and M) ordinates beyond the second are silently discarded.
     */
    private static List<double[]> parseCoordSequence(String seq) {
        List<double[]> coords = new ArrayList<>();
        if (seq == null || seq.trim().isEmpty()) return coords;
        for (String token : seq.split(",")) {
            double[] c = parseOneCoord(token.trim());
            if (c != null) coords.add(c);
        }
        return coords;
    }

    /**
     * Parses a single whitespace-separated coordinate token like {@code "78.33 17.46"}
     * or {@code "78.33 17.46 0.0"} into a two-element {@code [x, y]} array.
     *
     * @return {@code [x, y]} or {@code null} if the token is malformed
     */
    private static double[] parseOneCoord(String token) {
        if (token == null || token.isEmpty()) return null;
        String[] parts = token.trim().split("\\s+");
        if (parts.length < 2) return null;
        try {
            return new double[]{ Double.parseDouble(parts[0]),
                                 Double.parseDouble(parts[1]) };
        } catch (NumberFormatException e) {
            return null;   // silently skip malformed coordinate
        }
    }

    /**
     * Reads a WKT file and returns its geographic bounding box as
     * [minX, minY, maxX, maxY], or null if the file cannot be read,
     * does not exist, or its coordinates do not appear to be geographic.
     *
     * Used by normaliseCoordinatesIfGeographic() to pre-expand the shared
     * geo reference bbox using geoExtentFiles (road/map files) so that it
     * matches the bbox ONE's MapBasedMovement computes from its map layers.
     *
     * @param relPath  path to the WKT file (relative to ONE working directory)
     * @return [minLon, minLat, maxLon, maxLat] or null
     */
    private static double[] computeWktBoundingBox(String relPath) {
        try {
            java.nio.file.Path path = java.nio.file.Paths.get(relPath);
            if (!path.isAbsolute())
                path = java.nio.file.Paths.get(System.getProperty("user.dir", "."))
                           .resolve(path).normalize();
            if (!java.nio.file.Files.exists(path)) return null;

            List<double[]>       pts = new ArrayList<>();
            List<List<double[]>> lss = new ArrayList<>(), pgs = new ArrayList<>();

            // Try UTF-8 first, fall back to ISO-8859-1 (never throws).
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

            // Validate: only return if coordinates look geographic (lon/lat).
            double sx = maxX - minX, sy = maxY - minY;
            boolean geo = (sx <= 5.0 && sy <= 5.0)
                       && (minX >= -180.0 && minX <= 180.0)
                       && (minY >= -90.0  && minY <= 90.0);
            return geo ? new double[]{ minX, minY, maxX, maxY } : null;

        } catch (Exception e) {
            // Non-fatal: bbox contribution simply not included.
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

    /**
     * Solid-fills all grid cells whose centre lies inside the given polygon ring
     * using a scanline even-odd ray-casting test.
     *
     * This is the fix for building footprints (POLYGON / MULTIPOLYGON geometries)
     * appearing as scattered border dots on the planning grid instead of solid
     * filled rectangles.  Previously every polygon ring was handed to
     * rasterizeStripOnGrid() which only marks cells within lineObstacleHalfWidth
     * of each edge — correct for road barriers but wrong for building volumes.
     *
     * Algorithm: for each grid row, cast a horizontal ray from x=-∞ and count
     * edge crossings.  Cells to the right of an odd number of crossings are
     * inside the polygon and marked as blocked.
     */
    private void rasterizeFilledPolygon(List<double[]> ring) {
        if (ring == null || ring.size() < 3) return;

        // Bounding box of the ring in world coords → limit row/col scan range.
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
            // Y coordinate of the centre of this grid row.
            double cy = (row + 0.5) * gridCellM;

            // Collect X intersections of the horizontal ray y=cy with each edge.
            List<Double> xs = new ArrayList<>();
            for (int i = 0, j = n - 1; i < n; j = i++) {
                double y0 = ring.get(i)[1], y1 = ring.get(j)[1];
                double x0 = ring.get(i)[0], x1 = ring.get(j)[0];
                // Edge crosses the scanline?
                if ((y0 <= cy && y1 > cy) || (y1 <= cy && y0 > cy)) {
                    // X coordinate of the intersection.
                    double t = (cy - y0) / (y1 - y0);
                    xs.add(x0 + t * (x1 - x0));
                }
            }
            if (xs.isEmpty()) continue;
            Collections.sort(xs);

            // Fill between pairs of intersections (even-odd rule).
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
    /** Maps a world X value to a grid column index, clamped to [0, gridW-1]. */
    private int worldToGridCol(double x) {
        return Math.max(0, Math.min((int)(x / gridCellM), gridW - 1));
    }
    /** Maps a world Y value to a grid row index, clamped to [0, gridH-1]. */
    private int worldToGridRow(double y) {
        return Math.max(0, Math.min((int)(y / gridCellM), gridH - 1));
    }
    /**
     * @deprecated Use {@link #worldToGridCol} or {@link #worldToGridRow} instead.
     *             This method capped both axes at gridW-1, which silently
     *             corrupted row indices whenever gridH != gridW.
     */
    @Deprecated
    private int worldToGridScalar(double v) {
        return worldToGridCol(v);   // kept for any external callers; X-safe only
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
    public boolean isReachedFinalGoal() { return reachedFinalGoal; }   // FIX 1
    public int     getStallCount()   { return stallCount; }
    public Coord   getUavPos()       { return uavPos==null ? null : uavPos.clone(); }
    public int     getUavId()        { return uavId; }

    /**
     * Returns the full accumulated traversal trail as a Path object.
     * Every position visited since getInitialLocation() is included.
     * Useful for GUI renderers that want to draw the complete flight path.
     * Returns null if the drone has not yet started moving.
     */
    public Path getTrailPath() {
        return trailPath;
    }

    public static Map<Integer,double[]> getPeerRegistry() {
        return Collections.unmodifiableMap(PEER_REGISTRY);
    }

    /**
     * Returns the set of WKT canonical paths merged into the obstacle grid.
     * Use for debugging when obstacles appear missing for some groups:
     *   System.out.println(UAVWaypointMovement.getLoadedWktFiles());
     */
    public static Set<String> getLoadedWktFiles() {
        return Collections.unmodifiableSet(loadedWktFiles);
    }
}
