
package movement;

import core.Coord;
import core.SimScenario;
import core.Settings;
import core.SimClock;
import core.SimError;

import java.util.*;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicInteger;

/**
 
 * Implements the double-layer path planning algorithm from:
 *
 *   He, Hou &amp; Wang (2024). "A new method for unmanned aerial
 *   vehicle path planning in complex environments."
 
 */
public class UAVWaypointMovement extends MovementModel {

    

    /** Shared settings namespace; used as a fallback when a key is absent
     *  from the per-group block. */
    public static final String SETTINGS_NS = "UAVWaypointMovement";


    /** Settings key for the minimum hover (dwell) duration at a POI . */
    public static final String DWELL_MIN_S          = "dwellMin";
    /** Settings key for the maximum hover (dwell) duration at a POI . */
    public static final String DWELL_MAX_S          = "dwellMax";
    /** Settings key for the minimum cruise speed in m/s . */
    public static final String SPEED_MIN_S          = "speedMin";
    /** Settings key for the maximum cruise speed in m/s . */
    public static final String SPEED_MAX_S          = "speedMax";
    /** Settings key for the cruise altitude in metres . */
    public static final String CRUISE_ALT_S         = "cruiseAlt";
    /** Settings key for the A* grid cell side length in metres . */
    public static final String GRID_CELL_S          = "gridCellM";
    /** Settings key for the number of DWA velocity samples . */
    public static final String DWA_STEPS_S          = "dwaSteps";
    /** Settings key for the DWA obstacle alert distance in metres . */
    public static final String DIST_ALERT_S         = "distAlert";
    /** Settings key for the DWA obstacle danger (risk) distance in metres . */
    public static final String DIST_RISK_S          = "distRisk";
    /** Settings key for the path to the WKT obstacle file . */
    public static final String OBSTACLE_WKT_S       = "obstacleWktFile";
    /** Settings key for the drone spawn position as a csv pair (x, y) . */
    public static final String SPAWN_S              = "spawn";
    /** Settings key for the drone target position as a csv pair (x, y) . */
    public static final String TARGET_S             = "target";
    /** Settings key for the buffer radius around POINT obstacles in metres . */
    public static final String POINT_RADIUS_S       = "pointObstacleRadius";
    /** Settings key for the half-width of LINESTRING obstacle rasterisation . */
    public static final String LINE_HALF_WIDTH_S    = "lineObstacleHalfWidth";
    /** Settings key for the number of POI lattice columns (0 disables the grid) . */
    public static final String POI_GRID_COLS_S      = "poiGridCols";
    /** Settings key for the number of POI lattice rows (0 disables the grid) . */
    public static final String POI_GRID_ROWS_S      = "poiGridRows";
    /** Settings key controlling whether spawn/goal/POI coords are snapped to
     *  the nearest free grid cell . */
    public static final String SNAP_TO_GRID_S       = "snapLocationsToGrid";
    /** Settings key controlling whether the A* grid overlay is drawn in the GUI . */
    public static final String SHOW_PLANNING_GRID_S = "showPlanningGrid";
    /** Settings key for the minimum inter-UAV separation used by the VO cone
     *  and hard-collision guard, in metres . */
    public static final String UAV_SEP_S            = "uavSeparationM";
    /** Settings key for the Velocity Obstacle penalty multiplier . */
    public static final String VO_WEIGHT_S          = "voWeight";
    /** Settings key for the per-UAV staggered launch delay in seconds . */
    public static final String LAUNCH_STAGGER_S     = "launchStaggerS";
    /** Settings key for the X-axis stride (metres) used to offset spawn positions
     *  when all drones fall back to the shared UAVWaypointMovement.spawn value . */
    public static final String SPAWN_STRIDE_S       = "spawnStrideM";
    /** Settings key for explicit geographic extents as a csv quad
     *  (minLon, minLat, maxLon, maxLat) . */
    public static final String GEO_EXTENTS_S        = "geoExtents";
    /** Settings key for one or more WKT files that define geographic extents . */
    public static final String GEO_EXTENT_FILES_S   = "geoExtentFiles";
    /** Settings key for the geographic origin longitude used for coordinate
     *  projection . */
    public static final String GEO_ORIGIN_LON_S     = "geoOriginLon";
    /** Settings key for the geographic origin latitude used for coordinate
     *  projection . */
    public static final String GEO_ORIGIN_LAT_S     = "geoOriginLat";
    /** Settings key for the metres-per-degree scale factor applied during
     *  geographic coordinate projection . */
    public static final String GEO_SCALE_FACTOR_S   = "geoScaleFactor";

    
    //  Defaults values 

    /** Default minimum hover time at a POI in seconds. */
    private static final double DEF_DWELL_MIN       = 5.0;
    /** Default maximum hover time at a POI in seconds. */
    private static final double DEF_DWELL_MAX       = 15.0;
    /** Default minimum cruise speed in m/s. */
    private static final double DEF_SPEED_MIN       = 0.8;
    /** Default maximum cruise speed in m/s. */
    private static final double DEF_SPEED_MAX       = 1.0;
    /** Default cruise altitude in metres (metadata only; not used in 2-D planning). */
    private static final double DEF_CRUISE_ALT      = 30.0;
    /** Default A* grid cell side length in metres. */
    private static final double DEF_GRID_CELL       = 5.0;
    /** Default number of DWA velocity samples per step. */
    private static final int    DEF_DWA_STEPS       = 8;
    /** Default obstacle alert distance in metres (transitions to W_ALERT weights). */
    private static final double DEF_DIST_ALERT      = 25.0;
    /** Default obstacle danger distance in metres (transitions to W_DANGER weights). */
    private static final double DEF_DIST_RISK       = 12.0;
    /** Default minimum inter-UAV separation for the VO cone, in metres. */
    private static final double DEF_UAV_SEP         = 20.0;
    /** Default Velocity Obstacle penalty multiplier. */
    private static final double DEF_VO_WEIGHT       = 3.0;
    /** Default per-UAV staggered launch delay in seconds. */
    private static final double DEF_LAUNCH_STAGGER  = 8.0;
    /** Default X-axis spawn stride in metres when drones share a fallback spawn point. */
    private static final double DEF_SPAWN_STRIDE    = 20.0;
    private static final double DEFAULT_FINAL_GOAL_TOLERANCE = 2.0;

    

    /** DWA weights used in open space: prioritises heading and velocity. */
    private static final double[] W_FOLLOW = { 0.30, 0.30, 0.20, 0.20 };
    /** DWA weights used when within distAlert of an obstacle: increases obstacle
     *  avoidance term. */
    private static final double[] W_ALERT  = { 0.20, 0.30, 0.40, 0.10 };
    /** DWA weights used when within distRisk of an obstacle: maximises obstacle
     *  avoidance at the expense of path-following. */
    private static final double[] W_DANGER = { 0.20, 0.25, 0.50, 0.05 };

    /** Number of consecutive idle DWA steps before a stall is declared and
     *  an escape manoeuvre is attempted. */
    private static final int STALL_LIMIT = 6;

    

    /** Minimum and maximum hover time at a POI in seconds. */
    private double  dwellMin, dwellMax;
    /** Minimum and maximum cruise speed in m/s. */
    private double  speedMin, speedMax;
    /** Cruise altitude in metres (stored as metadata; planning is 2-D). */
    private double  cruiseAlt;
    /** A* grid cell side length in metres. */
    private double  gridCellM;
    /** Grid dimensions in cells derived from worldSize / gridCellM. */
    private int     gridW, gridH;
    /** Number of candidate velocities sampled per DWA step. */
    private int     dwaSteps;
    /** Distance threshold in metres at which DWA switches from W_FOLLOW
     *  to W_ALERT obstacle-avoidance weights. */
    private double  distAlert;
    /** Distance threshold in metres at which DWA switches from W_ALERT
     *  to W_DANGER obstacle-avoidance weights. */
    private double  distRisk;
    /** Minimum inter-UAV separation in metres; defines the VO cone radius. */
    private double  uavSeparationM;
    /** Scaling multiplier applied to the VO cone penalty score. */
    private double  voWeight;
    /** Per-UAV launch delay offset in seconds, indexed by UAV ID. */
    private double  launchStaggerS;
    /** X-axis offset in metres applied between drones that share the fallback
     *  UAVWaypointMovement.spawn position. */
    private double  spawnStrideM;
    /** World-space spawn coordinates (metres). */
    private double  spawnX, spawnY;
    /** World-space goal coordinates (metres). */
    private double  targetX, targetY;
    /** Buffer radius in metres used when rasterising POINT obstacles. */
    private double  pointObstacleRadius;
    /** Half-width in metres used when rasterising LINESTRING obstacles. */
    private double  lineObstacleHalfWidth;
    /** Number of POI lattice columns; 0 disables the POI grid. */
    private int     poiGridCols;
    /** Number of POI lattice rows; 0 disables the POI grid. */
    private int     poiGridRows;
    /** It is true, spawn, goal, and POI coordinates are snapped to the
     *  nearest obstacle-free grid cell before use. */
    private boolean snapLocationsToGrid;
    /**It is  true , when a per-group spawn coordinate was explicitly set in the
     *  GroupN config block (as opposed to inheriting the shared fallback). */
    private boolean perGroupSpawnSet = false;
    /** WKT obstacle file paths loaded for this group; merged into the shared
     *  grid at initialisation time. */
    private List<String> groupObstacleWktFiles = new ArrayList<>();
    /** WKT files that define geographic extent polygons for coordinate clamping. */
    private List<String> geoExtentFiles = new ArrayList<>();
    /** Explicit four-element geographic bounding box [minLon, minLat, maxLon, maxLat],
     *  or  null when not specified in the settings file. */
    private double[] explicitGeoExtents = null;

    

    /** Grid-coordinate conversion helper for this instance's grid parameters. */
    private UavPathUtils    pathUtils;

    /**
     * Obstacle-grid manager.  Wraps the shared static grid held in
     *  UavObstacleGrid and exposes obstacle queries and rasterisation.
     */
    private UavObstacleGrid obstacleGridHelper;

    static {
        
        core.DTNSim.registerForReset(UAVWaypointMovement.class.getName());
    }

    

    /**
     * Shared registry of all active UAVs' positions and velocities.
     * Each entry maps a UAV ID to a  double[4] array
     * e {x, y, vx, vy} in world coordinates.  Updated at the end
     * of every getPath() call so that peers see current state
     * during the next DWA step.
     */
    private static final ConcurrentHashMap<Integer, double[]> PEER_REGISTRY
            = new ConcurrentHashMap<>();

    /**
     * Shared registry of each UAV's accumulated flight trail, keyed by UAV ID.
     * Updated every getPath() call and exposed to the GUI via
     * getTrailRegistry().
     */
    private static final ConcurrentHashMap<Integer, Path> TRAIL_REGISTRY
            = new ConcurrentHashMap<>();

    /**
     * It return  , a map from UAV ID to its accumulated flight-trail Path
     */
    public static Map<Integer, Path> getTrailRegistry() {
        return Collections.unmodifiableMap(TRAIL_REGISTRY);
    }

    /** Monotonically increasing counter used to assign unique IDs to replicated
     *  UAV instances. */
    private static final AtomicInteger ID_COUNTER = new AtomicInteger(0);

    /** Unique identifier for this UAV instance, assigned during replication. */
    private int    uavId = -1;
    /** Last step's velocity components, published for VO calculations. */
    private double lastVx = 0.0, lastVy = 0.0;

    /** Reference to the simulator GUI;  */
    private static gui.DTNSimGUI gui = null;

    /**
     * Injects a reference to the simulator GUI so that movement-model code
     * can trigger overlay redraws. 
     */
    public static void setGui(gui.DTNSimGUI g) {
        gui = g;
    }


    /**
     * Returns the current planning-grid snapshot for the GUI obstacle overlay,
     */
    public static UavObstacleGrid.PlanningGridSnapshot getPlanningGridSnapshot() {
        return UavObstacleGrid.getPlanningGridSnapshot();
    }

    /**
     * Enables or disables rendering of the A* planning grid in the GUI.
     
     */
    public static void setGridRenderingEnabled(boolean enabled) {
        UavObstacleGrid.setGridRenderingEnabled(enabled);
    }

    /**
     * Returns whether the A* planning grid overlay is currently enabled.
     */
    public static boolean isGridRenderingEnabled() {
        return UavObstacleGrid.isGridRenderingEnabled();
    }

    /**
     * Returns render data for all obstacles currently loaded into the shared
     * grid.  Used by the GUI to draw the obstacle overlay.
     */
    public static List<UavObstacleGrid.ObstacleRenderData> getObstacleRenderData() {
        return UavObstacleGrid.getObstacleRenderData();
    }

    

    /** Ordered sequence of waypoints (POIs + final goal) that this UAV
     *  must visit during a mission tour. */
    private List<Coord> poiTour;
    /** Index into poiTour  identifying the current target POI. */
    private int         poiIndex;
    /** Key nodes extracted by Bresenham LOS compression from the raw A* path
     *  to the current POI; these become sequential DWA sub-goals. */
    private List<Coord> keyNodes;
    /** Index into keyNodes identifying the current DWA sub-goal. */
    private int         keyIndex;
    /** Current world-space position of this UAV, updated every
     *  getPath() call. */
    private Coord       uavPos;
    /** It is  true ,  while the UAV is hovering at a POI between movements. */
    private boolean     hovering   = false;
    /** Simulation time at which the current hover ends and movement resumes. */
    private double      hoverUntil = 0;
    /** Flag set when the current key-node sequence is invalid and a full
     *  Layer-1 re-plan is required at the start of the next getPath()
     *  call. */
    private boolean     replanNeeded = false;
    /** Number of consecutive getPath() calls during which the UAV
     *  has made no discernible forward progress (physical stall counter). */
    private int         stallCount   = 0;
    /** Closest distance to the current sub-goal observed so far; used to
     *  detect net-progress oscillations that are too slow to trip the
     *  physical stall counter. */
    private double      minSubGoalDist = Double.MAX_VALUE;
    /** Number of consecutive ticks during which the UAV has not beaten
     * minSubGoalDist triggers oscillation-based re-planning. */
    private int         progressStallCount = 0;

    /** Default arrival tolerance in metres used when checking whether the UAV
     *  has reached its final goal. */
    private static final double DEFAULT_FINAL_GOAL_TOLERANCE = 2.0;
    /** Settings key for the final-goal arrival tolerance in metres . */
    public  static final String FINAL_GOAL_TOLERANCE_S = "finalGoalTolerance";
    /** Configured arrival tolerance in metres */
    private double  finalGoalTolerance;

    /** It is true, when once the UAV has arrived within finalGoalTolerance
     *  of the last POI (the mission goal). */
    private boolean reachedFinalGoal = false;

    /** Accumulated flight-trail path for this UAV; appended every tick */
    private Path trailPath = null;

    /** Capacity of the circular grid-cell position history used to penalise
     *  recently visited cells in DWA candidate scoring. */
    private static final int HISTORY_SIZE = 16;
    /** Circular buffer storing the last HISTORY_SIZE grid cells visited
     *  by this UAV; each entry is a  [col, row] pair. */
    private final int[][] posHistory  = new int[HISTORY_SIZE][2];
    /** Write head for the circular posHistory buffer. */
    private int           historyHead = 0;
    /** Number of valid entries currently stored in posHistory. */
    private int           historyFill = 0;

    /** Cumulative nanoseconds spent inside getInitialLocation() and
     *  getPath() ; exposed via getComputeTimeSeconds(). */
    private long          totalComputeTimeNs = 0;
    /** Cumulative sum of absolute heading-change angles (radians) across all
     *  movement steps; a lower value indicates a smoother path. */
    private double        totalTurnCost = 0.0;
    /** Previous step's heading in radians, or null initially. */
    private Double        lastHeading = null;
    /** IT is true ,when after the mission-failure message has been printed for
     *  this UAV; prevents duplicate log lines at simulation end. */
    private boolean       failureLogPrinted = false;

 

    /** Multiplier applied to uavSeparationM to form the peer-proximity
     *  check radius; when any peer falls within this radius the primary mover
     *  delegates to DWA so that VO avoidance activates before physical contact. */
    private static final double PEER_CHECK_RATIO = 1.5;

    

    /**
     * Reads a  double setting, preferring the per-group scope  s
     * over the shared namespace  cfg and returning  def} when
     * the key is absent from both.
     
     */
    private static double readDouble(Settings s, Settings cfg,
                                     String key, double def) {
        if (s   != null && s.contains(key))   return s.getDouble(key);
        if (cfg != null && cfg.contains(key)) return cfg.getDouble(key);
        return def;
    }

    /**
     * Reads an  int setting, preferring the per-group scope s
     * over the shared namespace  cfg, and returning  def when
     * the key is absent from both.
     
     */
    private static int readInt(Settings s, Settings cfg,
                               String key, int def) {
        if (s   != null && s.contains(key))   return s.getInt(key);
        if (cfg != null && cfg.contains(key)) return cfg.getInt(key);
        return def;
    }

    /**
     * Reads a  boolean setting, preferring the per-group scope s
     * over the shared namespace  cfg ,  and returning def when
     * the key is absent from both.
     */
    private static boolean readBoolean(Settings s, Settings cfg,
                                       String key, boolean def) {
        if (s   != null && s.contains(key))   return s.getBoolean(key, def);
        if (cfg != null && cfg.contains(key)) return cfg.getBoolean(key, def);
        return def;
    }

    /**
     * Reads a String setting, preferring the per-group scope  s
     * over the shared namespace  cfg, and returning  def when
     * the key is absent from both.
     
     */
    private static String readString(Settings s, Settings cfg,
                                     String key, String def) {
        if (s   != null && s.contains(key))   return s.getSetting(key);
        if (cfg != null && cfg.contains(key)) return cfg.getSetting(key);
        return def;
    }

    /**
     * Reads a raw comma-separated obstacle file setting, attempting
     *  getCsvSetting() first (which honours all tokens in a
     * comma-separated value) and falling back to readString if
     * the CSV call throws.  This is necessary because
     *  getSetting() returns only the first token.
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
        } catch (Exception ignored) {}
        return readString(s, cfg, key, "");
    }

    /**
     * Reads a csv-double array setting, preferring the per-group scope
     *  s over the shared namespace cfg.
     
     */
    private static double[] readCsvDoubles(Settings s, Settings cfg,
                                           String key, int count) {
        if (s   != null && s.contains(key))   return s.getCsvDoubles(key, count);
        if (cfg != null && cfg.contains(key)) return cfg.getCsvDoubles(key, count);
        return null;
    }

    /**
     * Reads and de-duplicates all obstacle WKT file paths from the two-level
     */
    private static List<String> readObstacleFilePaths(Settings s, Settings cfg) {
        String raw = readRawObstacleFileSetting(s, cfg, OBSTACLE_WKT_S);
        return WktObstacleParser.parseObstacleFilePaths(raw);
    }


    /**
     * Prototype constructor called by ONE once per GroupN block.
     * Reads all per-group and shared settings, computes the grid dimensions,
     * and initialises the path-utility and obstacle-grid helper instances.     
     */
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

        // Track whether a per-group spawn was given to avoid applying spawnStrideM
        // to drones that already have explicit independent start positions.
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

        // Derive grid dimensions from world size and cell resolution.
        gridW = (int) Math.ceil(worldW() / gridCellM);
        gridH = (int) Math.ceil(worldH() / gridCellM);

        // Initialise sub-module helpers with this group's grid parameters.
        pathUtils          = new UavPathUtils(gridCellM, gridW, gridH, worldW(), worldH());
        obstacleGridHelper = new UavObstacleGrid(pathUtils, pointObstacleRadius,
                                                  lineObstacleHalfWidth, distAlert);

        groupObstacleWktFiles = readObstacleFilePaths(s, cfg);

        // Parse optional explicit geographic extents.
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
        geoExtentFiles = WktObstacleParser.parseObstacleFilePaths(geoExtRaw);

        // Initialise geo-scale in UavObstacleGrid (no-op after the first call).
        UavObstacleGrid.initGeoScaleIfNeeded(
            readDouble(s, cfg, GEO_ORIGIN_LON_S,   78.300),
            readDouble(s, cfg, GEO_ORIGIN_LAT_S,   17.480),
            readDouble(s, cfg, GEO_SCALE_FACTOR_S, 100000.0));
    }

    /**
     * Copy constructor called by ONE via replicate() to create one
     * instance per host node in the group.  Copies all per-group parameters
     * from the prototype, assigns a new unique UAV ID from ID_COUNTER,
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
        this.groupObstacleWktFiles = new ArrayList<>(proto.groupObstacleWktFiles);
        this.perGroupSpawnSet      = proto.perGroupSpawnSet;
        this.geoExtentFiles        = new ArrayList<>(proto.geoExtentFiles);
        this.explicitGeoExtents    = proto.explicitGeoExtents != null
                                     ? proto.explicitGeoExtents.clone() : null;
        this.finalGoalTolerance    = proto.finalGoalTolerance;

        // Per-instance planning state is not inherited from the prototype.
        this.reachedFinalGoal      = false;
        this.failureLogPrinted     = false;
        this.totalComputeTimeNs    = 0;
        this.totalTurnCost         = 0.0;
        this.lastHeading           = null;

        this.uavId = ID_COUNTER.getAndIncrement();

        // Each replicated instance gets its own helper pair; both delegate to the
        // same shared static obstacle grid held by UavObstacleGrid.
        this.pathUtils          = new UavPathUtils(this.gridCellM, this.gridW, this.gridH,
                                                    worldW(), worldH());
        this.obstacleGridHelper = new UavObstacleGrid(this.pathUtils, this.pointObstacleRadius,
                                                       this.lineObstacleHalfWidth, this.distAlert);
    }

    

    /**
     * Returns the initial world-space position of this UAV and completes all
     * one-time initialisation that depends on having a concrete UAV ID.
     */
    @Override
    public Coord getInitialLocation() {
        long _startNs = System.nanoTime();
        try {
            obstacleGridHelper.mergeObstaclesIfNeeded(groupObstacleWktFiles);

            // Apply the stride offset only when drones share the fallback spawn.
            double offsetX = (!perGroupSpawnSet && uavId >= 0) ? uavId * spawnStrideM : 0.0;
            uavPos = new Coord(spawnX + offsetX, spawnY);
            if (snapLocationsToGrid) uavPos = obstacleGridHelper.snapToNearestFreeCell(uavPos);

            poiTour      = buildMissionPoiTour(uavPos);
            poiIndex     = 0;
            replanNeeded = false;
            stallCount   = 0;
            minSubGoalDist = Double.MAX_VALUE;
            progressStallCount = 0;
            reachedFinalGoal = false;
            failureLogPrinted = false;
            this.totalTurnCost = 0.0;
            this.lastHeading = null;
            trailPath    = new Path(sampleSpeed());
            trailPath.addWaypoint(uavPos.clone());
            historyHead = 0;
            historyFill = 0;
            planToNextPoi();
            hovering   = false;
            hoverUntil = 0;

            if (uavId >= 0)
                PEER_REGISTRY.put(uavId,
                    new double[]{ uavPos.getX(), uavPos.getY(), 0.0, 0.0 });

            return uavPos.clone();
        } finally {
            this.totalComputeTimeNs += (System.nanoTime() - _startNs);
        }
    }

    /**
     * Computes and returns the next one-step movement path for this UAV.
     */
    @Override
    public Path getPath() {
        long _startNs = System.nanoTime();
        try {
            // ── End-of-simulation failure log ───────────────────────────────────
            if (SimClock.getTime() >= SimScenario.getInstance().getEndTime() - 1.0) {
                if (!reachedFinalGoal && !failureLogPrinted) {
                    System.out.printf("[GUI/LOG] FAILURE at T=%.1fs: UAV%d did not reach final goal (%.1f, %.1f)%n",
                            SimClock.getTime(), uavId, targetX, targetY);
                    failureLogPrinted = true;
                }
            }

            // ── Already stopped at goal: stay put indefinitely. ─────────────────
            if (reachedFinalGoal) {
                Path path = new Path(0);
                path.addWaypoint(uavPos.clone());
                path.addWaypoint(uavPos.clone());
                return path;
            }

            // ── Early arrival check: are we already close enough to the final goal? ─
            Coord finalGoal = poiTour.get(poiTour.size() - 1);
            if (poiIndex == poiTour.size() - 1
                    && uavPos.distance(finalGoal) <= finalGoalTolerance) {
                reachedFinalGoal = true;
                if (getHost() != null) {
                    System.out.println("Drone " + getHost().getAddress() + " successfully reached its goal at " + core.SimClock.getTime() + "s!");
                }
                lastVx = 0.0;
                lastVy = 0.0;
                if (uavId >= 0)
                    PEER_REGISTRY.put(uavId, new double[]{ uavPos.getX(), uavPos.getY(), 0.0, 0.0 });
                Path path = new Path(0);
                path.addWaypoint(uavPos.clone());
                path.addWaypoint(uavPos.clone());
                return path;
            }

            // ── Normal planning ─────────────────────────────────────────────────
            if (keyNodes == null || keyNodes.isEmpty()) planToNextPoi();

            Coord subGoal = keyNodes.get(keyIndex);

            if (replanNeeded) {
                // Clear flags and run a fresh Layer-1 plan before proceeding.
                replanNeeded = false; stallCount = 0;
                progressStallCount = 0;
                minSubGoalDist = Double.MAX_VALUE;
                planToNextPoi();
                subGoal = keyNodes.get(keyIndex);
            }

            double prevX = uavPos.getX(), prevY = uavPos.getY();
            double distToSub = uavPos.distance(subGoal);

            // Track net progress toward the sub-goal to detect oscillations.
            if (distToSub < minSubGoalDist - 0.1) {
                minSubGoalDist = distToSub;
                progressStallCount = 0;
            } else {
                progressStallCount++;
            }

            // ── Peer-proximity VO gate for direct-walk primary mover ─────────────
            boolean peerThreat = isPeerWithinRange(uavPos, PEER_CHECK_RATIO * uavSeparationM);

            double stepSize  = Math.min(speedMax, distToSub);
            Coord nextPos;
            if (distToSub < 0.1) {
                // Already at the sub-goal; snap exactly.
                nextPos = subGoal.clone();
            } else if (peerThreat) {
                // Peer nearby: use DWA so the VO penalty influences direction choice.
                nextPos = dwaStep(uavPos, subGoal);
            } else {
                // Attempt a direct step; fall back to DWA if it lands in an obstacle.
                double dx = (subGoal.getX() - uavPos.getX()) / distToSub;
                double dy = (subGoal.getY() - uavPos.getY()) / distToSub;
                Coord direct = pathUtils.clampToWorld(new Coord(
                    uavPos.getX() + dx * stepSize,
                    uavPos.getY() + dy * stepSize));
                int[] dc = pathUtils.worldToGrid(direct);
                if (UavObstacleGrid.obstacleGrid[dc[1]][dc[0]]) {
                    nextPos = dwaStep(uavPos, subGoal);
                } else {
                    nextPos = direct;
                }
            }

            // ── Record current position in the circular history buffer ───────────
            int[] curCell = pathUtils.worldToGrid(uavPos);
            posHistory[historyHead][0] = curCell[0];
            posHistory[historyHead][1] = curCell[1];
            historyHead = (historyHead + 1) % HISTORY_SIZE;
            if (historyFill < HISTORY_SIZE) historyFill++;

            // ── Stall and oscillation detection → escape then re-plan ────────────
            boolean isPhysicalStall = (nextPos.distance(uavPos) < speedMax * 0.1);
            boolean isOscillating = (progressStallCount >= STALL_LIMIT * 4);

            if (isPhysicalStall) stallCount++;
            else stallCount = 0;

            if (stallCount >= STALL_LIMIT || isOscillating) {
                // Reset counters and attempt a DWA escape manoeuvre.
                stallCount  = 0;
                progressStallCount = 0;
                minSubGoalDist = Double.MAX_VALUE;
                historyFill = 0;
                historyHead = 0;

                Coord escape = dwaEscapePoint(uavPos, subGoal);
                if (escape.distance(uavPos) < 1.0) {
                    // No usable escape point; schedule a full re-plan.
                    replanNeeded = true;
                } else {
                    if (keyNodes != null) {
                        // Insert the escape point immediately before the blocked sub-goal.
                        keyNodes.add(keyIndex, escape);
                    } else {
                        replanNeeded = true;
                    }
                }
            }

            boolean reachedSubGoal = false;
            boolean isFinalNode = (keyIndex == keyNodes.size() - 1);

            if (isFinalNode) {
                // For the final goal, require exact arrival without premature teleportation.
                if (nextPos.distance(subGoal) <= Math.max(speedMax, finalGoalTolerance)) {
                    uavPos = subGoal.clone();
                    reachedSubGoal = true;
                } else {
                    uavPos = nextPos.clone();
                }
            } else {
                uavPos = nextPos.clone();
                // For intermediate waypoints, advance smoothly when within 1.5 cells.
                if (nextPos.distance(subGoal) < gridCellM * 1.5) {
                    reachedSubGoal = true;
                }
            }

            if (reachedSubGoal) {
                // Reset per-sub-goal tracking state and advance indices.
                historyFill = 0; historyHead = 0;
                minSubGoalDist = Double.MAX_VALUE;
                progressStallCount = 0;
                if (++keyIndex >= keyNodes.size()) {
                    if (++poiIndex >= poiTour.size()) {
                        if (uavPos.distance(finalGoal) <= finalGoalTolerance) {
                            reachedFinalGoal = true;
                            if (getHost() != null) {
                                System.out.println("Drone " + getHost().getAddress() + " successfully reached its goal at " + core.SimClock.getTime() + "s!");
                            }
                            lastVx = 0.0;
                            lastVy = 0.0;
                            if (uavId >= 0)
                                PEER_REGISTRY.put(uavId, new double[]{ uavPos.getX(), uavPos.getY(), 0.0, 0.0 });
                            Path path = new Path(0);
                            path.addWaypoint(uavPos.clone());
                            path.addWaypoint(uavPos.clone());
                            return path;
                        } else {
                            // Rebuild the tour from the current position and try again.
                            poiIndex = 0;
                            poiTour  = buildMissionPoiTour(uavPos);
                        }
                    }
                    planToNextPoi();
                }
            }

            // Derive velocity from the positional delta and publish to the peer registry.
            lastVx = uavPos.getX() - prevX;
            lastVy = uavPos.getY() - prevY;

            // Accumulate heading-change cost for path-smoothness metric.
            if (Math.hypot(lastVx, lastVy) > 1e-3) {
                double heading = Math.atan2(lastVy, lastVx);
                if (lastHeading != null) {
                    double diff = Math.abs(heading - lastHeading);
                    while (diff > Math.PI) diff -= 2 * Math.PI;
                    totalTurnCost += Math.abs(diff);
                }
                lastHeading = heading;
            }

            if (uavId >= 0)
                PEER_REGISTRY.put(uavId,
                    new double[]{ uavPos.getX(), uavPos.getY(), lastVx, lastVy });

            // Append waypoint to the accumulated trail path.
            if (trailPath == null) {
                trailPath = new Path(sampleSpeed());
                trailPath.addWaypoint(new Coord(prevX, prevY));
            }
            trailPath.addWaypoint(uavPos.clone());

            // Build and return the single-segment path for this tick.
            Path path = new Path(sampleSpeed());
            path.addWaypoint(new Coord(prevX, prevY));
            path.addWaypoint(uavPos.clone());

            if (uavId >= 0 && trailPath != null) {
                TRAIL_REGISTRY.put(uavId, trailPath);
            }

            return path;
        } finally {
            this.totalComputeTimeNs += (System.nanoTime() - _startNs);
        }
    }


    /**
     * Returns true if any peer UAV other than this one is currently
     * within  threshold} metres of  pos.
     
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

    /**
     * Returns the simulation time at which the next path will be available,
     * implementing the POI dwell (hover) behaviour.
     */
    @Override
    public double nextPathAvailable() {
        double dwell = dwellMin + this.rng.nextDouble() * (dwellMax - dwellMin);
        // Stagger launches so drones do not depart simultaneously at T=0.
        if (uavId >= 0 && SimClock.getTime() < 1.0) dwell += uavId * launchStaggerS;
        hoverUntil = SimClock.getTime() + dwell;
        hovering   = true;
        return hoverUntil;
    }

    /**
     * Creates a per-host copy of this movement model.
     * @return a new {@link UAVWaypointMovement} initialised from this instance
     */
    @Override
    public UAVWaypointMovement replicate() { return new UAVWaypointMovement(this); }

    /** @return world width in metres, as reported by the parent {@link MovementModel}. */
    private double worldW() { return getMaxX(); }
    /** @return world height in metres, as reported by the parent {@link MovementModel}. */
    private double worldH() { return getMaxY(); }


    /**
     * Runs the Layer-1 A* planner from uavPos to the current POI
     * and stores the resulting key nodes in keyNodes, resetting
     * keyIndex to 0.
     
     */
    private void planToNextPoi() {
        Coord goal = poiTour.get(poiIndex);
        List<int[]> raw = aStarSearch(pathUtils.worldToGrid(uavPos), pathUtils.worldToGrid(goal));
        if (raw == null || raw.isEmpty()) {
            // No path found: insert an escape point and let the drone move first.
            keyNodes = new ArrayList<>();
            keyNodes.add(dwaEscapePoint(uavPos, goal));
        } else {
            List<Coord> wp = pathUtils.gridPathToWorld(raw);
            wp.add(goal.clone());
            keyNodes = UavPathUtils.bresenhamExtractKeyNodes(
                wp, UavObstacleGrid.obstacleGrid, gridCellM, gridW, gridH);
        }
        keyIndex = 0;
    }

    /**
     * Executes the improved A* search (Layer 1) from start to
     * goal on the shared obstacle grid.
     * The heuristic is adaptive: f(n) = g(n) + (1 + sigmoid(ξ)) * h(n)
     * where ξ is the obstacle coverage rate along the straight line from
     *  n to  goal
     *
     
     */
    private List<int[]> aStarSearch(int[] start, int[] goal) {
        int total = gridW * gridH;
        double[] gCost  = new double[total]; Arrays.fill(gCost, Double.MAX_VALUE);
        int[]  parentIdx = new int[total];   Arrays.fill(parentIdx, -1);
        int[]  parentDir = new int[total];   Arrays.fill(parentDir, -1);
        boolean[] closed = new boolean[total];

        PriorityQueue<double[]> open = new PriorityQueue<>(
            Comparator.comparingDouble(a -> a[0]));
        int si = pathUtils.cellIndex(start[0], start[1]);
        gCost[si] = 0;
        open.offer(new double[]{ adaptiveHeuristic(start, goal), start[0], start[1] });

        // Direction vectors: [E, W, N, S, NE, SE, NW, SW] with corresponding move costs.
        int[][] dirs     = { {1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1} };
        double[] moveCost= { 1,1,1,1,
            Math.sqrt(2), Math.sqrt(2), Math.sqrt(2), Math.sqrt(2) };

        while (!open.isEmpty()) {
            double[] cur = open.poll();
            int col = (int)cur[1], row = (int)cur[2], idx = pathUtils.cellIndex(col, row);
            if (closed[idx]) continue;
            closed[idx] = true;
            if (col == goal[0] && row == goal[1])
                return reconstructPath(parentIdx, goal, start);

            // Apply neighbour clipping: only expand the subset of directions
            // that cannot be reached more cheaply via the current node's parent.
            int pDir = parentDir[idx];
            int[] activeDirs = getActiveNeighbours(pDir);
            for (int di : activeDirs) {
                int nc = col+dirs[di][0], nr = row+dirs[di][1];
                if (!pathUtils.inBounds(nc,nr) || UavObstacleGrid.obstacleGrid[nr][nc]) continue;
                // Block diagonal moves that cut through obstacle corners.
                if (di >= 4 && (UavObstacleGrid.obstacleGrid[row][nc]
                             || UavObstacleGrid.obstacleGrid[nr][col])) continue;
                int ni = pathUtils.cellIndex(nc, nr);
                if (closed[ni]) continue;
                double tg = gCost[idx] + moveCost[di] * gridCellM;
                if (tg < gCost[ni]) {
                    gCost[ni] = tg; parentIdx[ni] = idx; parentDir[ni] = di;
                    // Small jitter breaks f-value ties stochastically.
                    double jitter = (this.rng.nextDouble() - 0.5) * gridCellM * 0.01;
                    open.offer(new double[]{
                        tg + adaptiveHeuristic(new int[]{nc, nr}, goal) + jitter, nc, nr });
                }
            }
        }
        return null; // No path found.
    }

    /**
     * Computes the adaptive A* heuristic for node  n toward code g.
     * Uses Manhattan distance scaled by (1 + sigmoid(ξ)) where ξ is the
     * fraction of obstacle-occupied cells along the Bresenham line from
     *  n to  g .  This inflates the heuristic
     * in obstacle-dense regions, guiding the search away from blocked areas
     * faster than a plain Manhattan heuristic.
     */
    private double adaptiveHeuristic(int[] n, int[] g) {
        double manhattan = (Math.abs(n[0] - g[0]) + Math.abs(n[1] - g[1])) * gridCellM;
        double xi = obstacleCoverageRate(n, g);
        double multiplier = 1.0 + 0.3 / (1.0 + Math.exp(-xi * 10.0));
        return multiplier * manhattan;
    }

    /**
     * Computes the obstacle coverage rate ξ along the Bresenham line from
     * grid cell  n to grid cell g.
     *
     * ξ = (number of obstacle cells intersected) / (total cells traversed).
     * Used as the obstacle-density input to the sigmoid in
     * adaptiveHeuristic(int[], int[])
     */
    private double obstacleCoverageRate(int[] n, int[] g) {
        int x0 = n[0], y0 = n[1], x1 = g[0], y1 = g[1];
        int dx = Math.abs(x1 - x0), dy = Math.abs(y1 - y0);
        int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
        int err = dx - dy, x = x0, y = y0;
        int total = 0, obs = 0;
        while (true) {
            total++;
            if (pathUtils.inBounds(x,y) && UavObstacleGrid.obstacleGrid[y][x]) obs++;
            if (x==x1 && y==y1) break;
            int e2=2*err;
            if (e2>-dy) { err-=dy; x+=sx; }
            if (e2< dx) { err+=dx; y+=sy; }
        }
        return total == 0 ? 0.0 : (double) obs / total;
    }

    /**
     * Returns the subset of the eight movement directions that should be
     * expanded from a node whose parent arrived from direction d.
     *
     * This implements the neighbour-clipping pruning rule from paper Fig. 1:
     * when a parent direction is known, only the directions that cannot be
     * reached via a cheaper path bypassing the current node are considered.
     * All eight directions are returned for the start node (d &lt; 0).
     
     */
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

    /**
     * Reconstructs the A* path from the parent-index and parent-direction
     * arrays by walking backward from the goal to the start.
     */
    private List<int[]> reconstructPath(int[] pi, int[] pd, int[] goal, int[] start) {
        LinkedList<int[]> path = new LinkedList<>();
        int idx = pathUtils.cellIndex(goal[0], goal[1]);
        while (idx != pathUtils.cellIndex(start[0], start[1])) {
            path.addFirst(new int[]{ idx % gridW, idx / gridW });
            int p = pi[idx]; if (p < 0 || p == idx) break; idx = p;
        }
        path.addFirst(start.clone());
        return new ArrayList<>(path);
    }

    

    /**
     * Compresses a raw A* waypoint list to only the critical turning-point
     * (key) nodes using iterative Bresenham line-of-sight checks 
     *
     * Starting from each confirmed key node, the method walks forward
     * through the raw waypoints and retains the furthest reachable waypoint
     * that has an unobstructed line-of-sight.  The result is a minimal set
     * of waypoints that captures every necessary turn while eliminating
     * redundant intermediate nodes.
     
     */
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

    /**
     * It returns  true , if there is an unobstructed line of sight between
     * world-space coordinates a  and  b, tested on the obstacle
     * grid using the Bresenham line algorithm.
     */
    private boolean bresenhamLOS(Coord a, Coord b) {
        int[] ca = pathUtils.worldToGrid(a), cb = pathUtils.worldToGrid(b);
        int x0=ca[0], y0=ca[1], x1=cb[0], y1=cb[1];
        int dx=Math.abs(x1-x0), dy=Math.abs(y1-y0);
        int sx=x0<x1?1:-1, sy=y0<y1?1:-1, err=dx-dy, x=x0, y=y0;
        while (true) {
            if (!pathUtils.inBounds(x,y) || UavObstacleGrid.obstacleGrid[y][x]) return false;
            if (x==x1 && y==y1) break;
            int e2=2*err;
            if (e2>-dy) { err-=dy; x+=sx; }
            if (e2< dx) { err+=dx; y+=sy; }
        }
        return true;
    }


    /**
     * Executes a single Layer-2 DWA step from  pos toward  subGoal,
     * using the restricted angular sweep (±π/2 around the goal bearing).
     * Falls back to a full 2π sweep via waStepInternal(Coord, Coord, boolean
     * when the restricted sweep produces no net movement.
     
     */
    private Coord dwaStep(Coord pos, Coord subGoal) {
        return dwaStepInternal(pos, subGoal, false);
    }

    /**
     * Evaluates and selects the optimal immediate movement step.
          * input ---     the drone's current position , 
     *    where the drone is trying to go right now , 
     *  to look in a full circle instead of just forward
     * return --- best, safest position to move to next
     */
    private Coord dwaStepInternal(Coord pos, Coord subGoal, boolean fullSweep) {
        double d = obstacleGridHelper.nearestObstacleDistance(pos);
        // Select adaptive weight set based on distance to nearest obstacle.
        double[] w = d<=distRisk ? W_DANGER : d<=distAlert ? W_ALERT : W_FOLLOW;
        double alpha=w[0], beta=w[1], lambda=w[2], eta=w[3];

        double bestScore = Double.NEGATIVE_INFINITY;
        Coord  bestPos   = pos.clone();
        double step = Math.min(speedMax, Math.max(speedMin, pos.distance(subGoal)));

        // ID-based tiebreaker: even-ID drones yield slightly left, odd-ID right,
        // preventing symmetric oscillation in head-on encounters.
        double idBias = (uavId % 2 == 0) ? 0.35 : -0.35;

        double base     = Math.atan2(subGoal.getY()-pos.getY(), subGoal.getX()-pos.getX());
        int    nSamples = fullSweep ? dwaSteps * 2 : dwaSteps;

        // Forward-extrapolate peer positions by one step along their last velocity.
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

            Coord cand = pathUtils.clampToWorld(new Coord(
                pos.getX()+step*Math.cos(angle), pos.getY()+step*Math.sin(angle)));
            int[] cell = pathUtils.worldToGrid(cand);
            if (UavObstacleGrid.obstacleGrid[cell[1]][cell[0]]) continue;

            // ── HARD COLLISION AVOIDANCE ─────────────────────────────────────────
            // Skip candidates that bring this drone closer to a peer than 5.5 m.
            boolean hardCollision = false;
            for (double[] peer : peerSnapshots) {
                double curDist = Math.hypot(peer[0] - pos.getX(), peer[1] - pos.getY());
                double nxtDist = Math.hypot(peer[0] - cand.getX(), peer[1] - cand.getY());
                if (nxtDist < 5.5 && nxtDist <= curDist + 1e-4) {
                    hardCollision = true;
                    break;
                }
            }
            if (hardCollision) continue;

            // ── History penalty ──────────────────────────────────────────────────
            // Penalise recently visited grid cells to discourage re-traversal.
            double histPenalty = 0.0;
            for (int h = 0; h < historyFill; h++) {
                int hi = (historyHead - 1 - h + HISTORY_SIZE) % HISTORY_SIZE;
                if (posHistory[hi][0] == cell[0] && posHistory[hi][1] == cell[1])
                    histPenalty -= 1.2 * (1.0 - (double) h / HISTORY_SIZE);
            }

            // ── Velocity Obstacle penalty ────────────────────────────────────────
            double peerPenalty = 0.0;
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
                    // Within the danger zone: apply a strong repulsion penalty.
                    double dot = (myVx*rx + myVy*ry) /
                                 Math.max(1e-9, myVM * sd);
                    peerPenalty -= voWeight * 5.0 * Math.max(0, dot);
                    peerTooClose = true;
                    continue;
                }

                // Compute the VO cone: sin θ = uavSeparationM / dist(self, peer).
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
                    // Relative velocity is inside the VO cone; scale penalty by
                    // penetration depth and proximity.
                    double pen  = (cosA - cosT) / Math.max(1e-9, 1 - cosT);
                    double prox = 1 - Math.min(1, sd / (uavSeparationM * 5));
                    peerPenalty -= voWeight * pen * (1 + prox);
                }
            }

            // ── Repulsion bonus for moving away from nearby peers ────────────────
            double repulsionBonus = 0.0;
            for (double[] peer : peerSnapshots) {
                double sd = Math.hypot(peer[0] - pos.getX(), peer[1] - pos.getY());
                if (sd < 0.5 || sd > uavSeparationM * 3) continue;
                double rx = peer[0]-pos.getX(), ry = peer[1]-pos.getY();
                double dot = (myVx*rx + myVy*ry) /
                             Math.max(1e-9, myVM * sd);
                // Reward velocity components that point away from the peer.
                if (dot < 0) repulsionBonus += 0.15 * (-dot);
            }

            // ID tiebreaker is only applied when peer interactions are active.
            double tb = (peerPenalty < 0 || peerTooClose)
                        ? idBias * Math.signum(off + 1e-12) : 0;

            // ── Evaluate the candidate ───────────────────────────────────────────
            double bear    = Math.atan2(subGoal.getY()-cand.getY(),
                                        subGoal.getX()-cand.getX());
            double heading = Math.PI - Math.abs(UavPathUtils.normaliseAngle(bear - angle));
            double vel     = (v - speedMin) / Math.max(1e-9, speedMax - speedMin);
            double dObsS   = Math.min(1.0, obstacleGridHelper.nearestObstacleDistance(cand) / distAlert);
            double dFolS   = 1.0 - Math.min(1.0,
                             UavPathUtils.pointToSegmentDist(cand, pos, subGoal) / (gridCellM * 5));

            double score = alpha * heading + beta * vel
                         + lambda * dObsS  + eta * dFolS
                         + peerPenalty + repulsionBonus + histPenalty + tb;
            if (score > bestScore) { bestScore = score; bestPos = cand; }
        }

        // If the restricted sweep produced no movement, retry with a full 2π sweep.
        if (!fullSweep && bestPos.distance(pos) < 0.5) {
            return dwaStepInternal(pos, subGoal, true);

        return bestPos;
    }

    /**
     * Selects an escape waypoint when the UAV is stalled or oscillating.
     *Samples candidate positions on a circle of radius 3 * gridCellM
     * around  pos, discarding any that land in an obstacle cell or
     * within uavSeparationM of a peer.  Candidates are scored by a
     * weighted combination of obstacle clearance and alignment with the goal
     * bearing, and the highest-scoring free candidate is returned.
     
     */
    private Coord dwaEscapePoint(Coord pos, Coord goal) {
        double best=Double.NEGATIVE_INFINITY; Coord out=pos.clone();
        double step=gridCellM*3;
        double gb=Math.atan2(goal.getY()-pos.getY(), goal.getX()-pos.getX());
        int sw=dwaSteps*2;
        for (int i=0; i<=sw; i++) {
            double angle=((double)i/sw)*2*Math.PI;
            Coord c=pathUtils.clampToWorld(new Coord(
                pos.getX()+step*Math.cos(angle), pos.getY()+step*Math.sin(angle)));
            int[] cell=pathUtils.worldToGrid(c); if(UavObstacleGrid.obstacleGrid[cell[1]][cell[0]]) continue;

            // Discard candidates too close to any peer.
            boolean peerTooClose = false;
            for (Map.Entry<Integer, double[]> e : PEER_REGISTRY.entrySet()) {
                if (e.getKey() == uavId) continue;
                double[] p = e.getValue();
                if (Math.hypot(p[0] - c.getX(), p[1] - c.getY()) < uavSeparationM) {
                    peerTooClose = true;
                    break;
                }
            }
            if (peerTooClose) continue;

            // Score: 75 % obstacle clearance, 25 % alignment with goal bearing.
            double sc=0.75*Math.min(1,obstacleGridHelper.nearestObstacleDistance(c)/distAlert)
                    +0.25*0.5*(1+Math.cos(angle-gb));
            if (sc>best) { best=sc; out=c; }
        }
        return out;
    }
    

    /**
     * Builds the list of POI lattice waypoints by dividing the world into a
     * poiGridCols × poiGridRows grid and placing one
     * waypoint at the centre of each cell that is not blocked by an obstacle.
     * Returns an empty list when either dimension is zero (POI grid disabled).
     */
    private List<Coord> buildPoiGridWaypoints() {
        if (poiGridCols<=0||poiGridRows<=0) return new ArrayList<>();
        double W=worldW(), H=worldH();
        List<Coord> out=new ArrayList<>();
        for (int ix=0; ix<poiGridCols; ix++)
            for (int iy=0; iy<poiGridRows; iy++) {
                double x=(ix+0.5)*W/poiGridCols, y=(iy+0.5)*H/poiGridRows;
                if (obstacleGridHelper.isBlockedWorldXY(x,y)) continue;
                Coord c=new Coord(x,y);
                if (snapLocationsToGrid) c=obstacleGridHelper.snapToNearestFreeCell(c);
                out.add(c);
            }
        return out;
    }

    /**
     * Take input  the starting position used as the tour's departure point
     *               for the nearest-neighbour ordering
     * return ,  ordered list of world-space waypoints ending at the final goal
     */
    private List<Coord> buildMissionPoiTour(Coord origin) {
        List<Coord> lattice=buildPoiGridWaypoints();
        List<Coord> tour=new ArrayList<>();
        if (!lattice.isEmpty()) tour.addAll(nearestNeighbourTour(origin, new ArrayList<>(lattice)));
        Coord goal=new Coord(targetX, targetY);
        if (snapLocationsToGrid) goal=obstacleGridHelper.snapToNearestFreeCell(goal);
        tour.add(goal);
        return tour;
    }

    /**
     * Orders a list of POI coordinates using a greedy nearest-neighbour
     * heuristic, starting from  start.
     */
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

    

    /**
     * Clears all shared obstacle state and resets peer and trail registries.
     */
    public static synchronized void reset() {
        UavObstacleGrid.reset();
        PEER_REGISTRY.clear();
        TRAIL_REGISTRY.clear();
        ID_COUNTER.set(0);
    }

    /**
     * return , Samples a cruise speed uniformly from speedMin ,speedMax.
     */
    private double sampleSpeed() {
        return speedMin + this.rng.nextDouble() * (speedMax - speedMin);
    }

    

    /**
     * it return ,  the key nodes for the current POI segment, or an empty list
     *         if planning has not yet started
     */
    public List<Coord> getKeyNodes() {
        return keyNodes==null ? Collections.emptyList()
                              : Collections.unmodifiableList(keyNodes);
    }

    /**
     * Returns the index of the current key node within keyNodes.
     */
    public int getKeyIndex()     { return keyIndex; }

    /**
     * Returns the index of the current POI within poiTour.
     
     */
    public int getPoiIndex()     { return poiIndex; }

    /**
     * Returns an unmodifiable view of the current mission POI tour.
     * it return ,  the ordered tour waypoints, or an empty list before initialisation
     */
    public List<Coord> getPoiTour()  {
        return poiTour==null ? Collections.emptyList()
                             : Collections.unmodifiableList(poiTour);
    }

    /**
     * Returns  true while the UAV is hovering at a POI and the hover
     * period has not yet expired.
     */
    public boolean isHovering()      { return hovering && SimClock.getTime()<hoverUntil; }

    /**
     * Returns {true} when a Layer-1 re-plan has been scheduled for
     * the next getPath() call.

     */
    public boolean isReplanPending() { return replanNeeded; }

    /**
     * Returns {} true} once the UAV has arrived within
     */
    public boolean isReachedFinalGoal() { return reachedFinalGoal; }

    /**
     * Returns the current physical stall count.
     * it, return number of consecutive ticks with no net forward movement
     */
    public int getStallCount()   { return stallCount; }

    /**
     * Returns a defensive copy of the current UAV world-space position.
    */
    public Coord getUavPos()       { return uavPos==null ? null : uavPos.clone(); }

    /**
     * Returns the unique integer ID assigned to this UAV instance.
    */
    public int getUavId()        { return uavId; }

    /**
     * Returns the accumulated flight-trail path for GUI rendering.
     */
    public Path getTrailPath() {
        return trailPath;
    }

    /**
     * Returns an unmodifiable view of the shared peer registry.
     */
    public static Map<Integer,double[]> getPeerRegistry() {
        return Collections.unmodifiableMap(PEER_REGISTRY);
    }

    /**
     * Returns the set of WKT obstacle file paths that have been loaded into
     * the shared grid.  
     */
    public static Set<String> getLoadedWktFiles() {
        return UavObstacleGrid.getLoadedWktFiles();
    }

    /**
     * Returns the total CPU time spent inside getInitialLocation() and
     * getPath() for this instance, in seconds.
     
     */
    public double getComputeTimeSeconds() {
        return this.totalComputeTimeNs / 1e9;
    }

    /**
     * Returns the cumulative sum of absolute heading-change angles (in radians)
     * across all movement steps.  Lower values indicate a smoother flight path.
     */
    public double getPathSmoothness() {
        return this.totalTurnCost;
    }
}
