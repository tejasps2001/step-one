package movement;

import core.Coord;
import core.SimError;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.*;

/**
 * Manages the shared static obstacle grid for UAV movement.
 * Stores rasterized obstacles, render data, and geographic transform parameters.
 */
public class UavObstacleGrid {

    // Snapshot of the A* grid for the GUI
    public static final class PlanningGridSnapshot {
        public final double      gridCellM;
        public final int         gridW, gridH;
        public final boolean[][] blocked;

        public PlanningGridSnapshot(double c, int w, int h, boolean[][] b) {
            this.gridCellM = c; this.gridW = w; this.gridH = h; this.blocked = b;
        }
    }

    // Data for rendering WKT obstacles
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

    //  Shared static obstacle state
    static boolean[][]    obstacleGrid   = null; // Rasterized grid (true = blocked)
    static List<double[]> obstacleDiscs  = null;
    static List<double[]> obstacleSegBuf = null;

    private static final Set<String> loadedWktFiles
            = Collections.synchronizedSet(new LinkedHashSet<>());

    private static List<ObstacleRenderData> obstacleRenderData = null;

    private static PlanningGridSnapshot planningGridSnapshot = null;
    private static boolean gridRenderingEnabled = true;

    // Geographic transform parameters
    static double geoMinX = Double.MAX_VALUE;
    static double geoMaxX = -Double.MAX_VALUE;
    static double geoMinY = Double.MAX_VALUE;
    static double geoMaxY = -Double.MAX_VALUE;
    static boolean geoScaleInitialised = false;

    static double geoOriginLon   = 78.300;
    static double geoOriginLat   = 17.480;
    static double geoScaleFactor = 100000.0;

    private final UavPathUtils pathUtils;
    private final double       pointObstacleRadius;
    private final double       lineObstacleHalfWidth;
    private final double       distAlert;

    /**
     * Initializes with per-group parameters.
     */
    public UavObstacleGrid(UavPathUtils pathUtils,
                           double pointObstacleRadius,
                           double lineObstacleHalfWidth,
                           double distAlert) {
        this.pathUtils            = pathUtils;
        this.pointObstacleRadius  = pointObstacleRadius;
        this.lineObstacleHalfWidth = lineObstacleHalfWidth;
        this.distAlert            = distAlert;
    }

    public static PlanningGridSnapshot getPlanningGridSnapshot() {
        return gridRenderingEnabled ? planningGridSnapshot : null;
    }

    public static void setGridRenderingEnabled(boolean enabled) {
        gridRenderingEnabled = enabled;
    }

    public static boolean isGridRenderingEnabled() {
        return gridRenderingEnabled;
    }

    public static List<ObstacleRenderData> getObstacleRenderData() {
        return obstacleRenderData != null
               ? new ArrayList<>(obstacleRenderData)
               : new ArrayList<>();
    }

    public static Set<String> getLoadedWktFiles() {
        return Collections.unmodifiableSet(loadedWktFiles);
    }

    /** Returns the live grid reference. */
    public static boolean[][] getGrid() {
        return obstacleGrid;
    }

 /**
     * Sets geo-scale parameters (once per simulation).
     */
    public static synchronized void initGeoScaleIfNeeded(
            double originLon, double originLat, double scaleFactor) {
        if (!geoScaleInitialised) {
            geoOriginLon   = originLon;
            geoOriginLat   = originLat;
            geoScaleFactor = scaleFactor;
        }
    }

    /**
     * Merges WKT obstacle files into the shared grid.
     */
    public void mergeObstaclesIfNeeded(List<String> wktFiles) {
        synchronized (UavObstacleGrid.class) {
            mergeObstaclesIfNeededLocked(wktFiles);
        }
    }

    /** Internal locked merge logic. */
    private void mergeObstaclesIfNeededLocked(List<String> wktFiles) {
        if (obstacleGrid == null) {
            obstacleGrid      = new boolean[pathUtils.gridH][pathUtils.gridW];
            obstacleDiscs     = new ArrayList<>();
            obstacleSegBuf    = new ArrayList<>();
            obstacleRenderData = new ArrayList<>();
            publishPlanningGridSnapshot();
        }

        if (wktFiles == null || wktFiles.isEmpty()) return;

        boolean anyNewFile = false;

        for (String wkt : wktFiles) {
            if (wkt == null || wkt.trim().isEmpty()) continue;
            String trimmed = wkt.trim();

            String canonical;
            try {
                Path p = Paths.get(trimmed);
                if (!p.isAbsolute())
                    p = Paths.get(System.getProperty("user.dir", ".")).resolve(p);
                p = p.normalize();
                canonical = Files.exists(p) ? p.toRealPath().toString()
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

     /** Reads and rasterizes geometries from a WKT file. */
    private void loadObstaclesFromWkt(String rel) {
        Path path = Paths.get(rel);
        if (!path.isAbsolute())
            path = Paths.get(System.getProperty("user.dir", ".")).resolve(path);

        List<String> lines = WktObstacleParser.readWktFile(path);

        List<double[]>       points      = new ArrayList<>();
        List<List<double[]>> linestrings = new ArrayList<>();
        List<List<double[]>> polygons    = new ArrayList<>();

        for (int lineNo = 0; lineNo < lines.size(); lineNo++) {
            String line = lines.get(lineNo).trim();
            if (line.isEmpty() || line.startsWith("#")
                    || line.startsWith("//")) continue;
            try {
                WktObstacleParser.extractGeometry(
                        line, points, linestrings, polygons);
            } catch (Exception e) {
                System.err.println("[UAVWaypointMovement] Skipping malformed"
                    + " geometry at " + path.getFileName() + " line "
                    + (lineNo + 1) + ": " + e.getMessage());
            }
        }

        normaliseCoordinatesIfGeographic(points, linestrings, polygons);

        for (double[] pt : points) {
            rasterizeDiscOnGrid(pt[0], pt[1], pointObstacleRadius);
            obstacleDiscs.add(new double[]{ pt[0], pt[1],
                                            pointObstacleRadius });
            if (obstacleRenderData != null)
                obstacleRenderData.add(new ObstacleRenderData(
                    ObstacleRenderData.Type.POINT,
                    Arrays.asList(new Coord(pt[0], pt[1])),
                    pointObstacleRadius));
        }
        for (List<double[]> ls : linestrings) {
            List<Coord> lsCoords = obstacleRenderData != null
                                   ? new ArrayList<>(ls.size()) : null;
            for (int i = 0; i < ls.size() - 1; i++) {
                double[] a = ls.get(i), b = ls.get(i + 1);
                rasterizeStripOnGrid(a[0], a[1], b[0], b[1],
                                     lineObstacleHalfWidth);
                obstacleSegBuf.add(new double[]{ a[0], a[1], b[0], b[1],
                                                  lineObstacleHalfWidth });
            }
            if (lsCoords != null) {
                for (double[] p : ls) lsCoords.add(new Coord(p[0], p[1]));
                if (lsCoords.size() >= 2)
                    obstacleRenderData.add(new ObstacleRenderData(
                        ObstacleRenderData.Type.LINE, lsCoords,
                        lineObstacleHalfWidth));
            }
        }
        for (List<double[]> ring : polygons) {
            rasterizeFilledPolygon(ring);
            for (int i = 0; i < ring.size() - 1; i++) {
                double[] a = ring.get(i), b = ring.get(i + 1);
                obstacleSegBuf.add(new double[]{ a[0], a[1],
                                                  b[0], b[1], 0.0 });
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

    /** Normalizes coordinates if the file contains geographic (lat/lon) data. */
    private void normaliseCoordinatesIfGeographic(
            List<double[]>       points,
            List<List<double[]>> linestrings,
            List<List<double[]>> polygons) {

        double fileMinX = Double.MAX_VALUE, fileMaxX = -Double.MAX_VALUE;
        double fileMinY = Double.MAX_VALUE, fileMaxY = -Double.MAX_VALUE;

        for (double[] p : points) {
            fileMinX = Math.min(fileMinX, p[0]);
            fileMaxX = Math.max(fileMaxX, p[0]);
            fileMinY = Math.min(fileMinY, p[1]);
            fileMaxY = Math.max(fileMaxY, p[1]);
        }
        for (List<double[]> ls : linestrings)
            for (double[] p : ls) {
                fileMinX = Math.min(fileMinX, p[0]);
                fileMaxX = Math.max(fileMaxX, p[0]);
                fileMinY = Math.min(fileMinY, p[1]);
                fileMaxY = Math.max(fileMaxY, p[1]);
            }
        for (List<double[]> pg : polygons)
            for (double[] p : pg) {
                fileMinX = Math.min(fileMinX, p[0]);
                fileMaxX = Math.max(fileMaxX, p[0]);
                fileMinY = Math.min(fileMinY, p[1]);
                fileMaxY = Math.max(fileMaxY, p[1]);
            }

        if (fileMinX == Double.MAX_VALUE) return;

        double spanX = fileMaxX - fileMinX, spanY = fileMaxY - fileMinY;

        boolean isGeographic = (spanX <= 5.0 && spanY <= 5.0)
                && (fileMinX >= -180.0 && fileMinX <= 180.0)
                && (fileMinY >= -90.0  && fileMinY <= 90.0);
        if (!isGeographic) return;

        if (!geoScaleInitialised) {
            geoScaleInitialised = true;
            System.out.printf("[UAVWaypointMovement] Geographic transform"
                + " locked: originLon=%.4f  originLat=%.4f  scale=%.1f%n",
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

  /** Rasterizes a circle on the grid. */
    private void rasterizeDiscOnGrid(double cx, double cy, double r) {
        double r2 = r * r;
        int c0 = pathUtils.worldToGridCol(cx - r - pathUtils.gridCellM);
        int c1 = pathUtils.worldToGridCol(cx + r + pathUtils.gridCellM);
        int r0 = pathUtils.worldToGridRow(cy - r - pathUtils.gridCellM);
        int r1 = pathUtils.worldToGridRow(cy + r + pathUtils.gridCellM);
        for (int row = Math.max(0, r0);
             row <= Math.min(pathUtils.gridH - 1, r1); row++)
            for (int col = Math.max(0, c0);
                 col <= Math.min(pathUtils.gridW - 1, c1); col++) {
                Coord c = pathUtils.gridToWorld(col, row);
                double dx = c.getX() - cx, dy = c.getY() - cy;
                if (dx * dx + dy * dy <= r2) obstacleGrid[row][col] = true;
            }
    }

    /** Rasterizes a line segment with a half-width buffer. */
    private void rasterizeStripOnGrid(double x0, double y0,
                                      double x1, double y1, double hw) {
        double pad = hw + pathUtils.gridCellM;
        int c0 = pathUtils.worldToGridCol(Math.min(x0, x1) - pad);
        int c1 = pathUtils.worldToGridCol(Math.max(x0, x1) + pad);
        int r0 = pathUtils.worldToGridRow(Math.min(y0, y1) - pad);
        int r1 = pathUtils.worldToGridRow(Math.max(y0, y1) + pad);
        for (int row = Math.max(0, r0);
             row <= Math.min(pathUtils.gridH - 1, r1); row++)
            for (int col = Math.max(0, c0);
                 col <= Math.min(pathUtils.gridW - 1, c1); col++) {
                Coord c = pathUtils.gridToWorld(col, row);
                if (UavPathUtils.pointToSegmentDist(c,
                        new Coord(x0, y0), new Coord(x1, y1)) <= hw)
                    obstacleGrid[row][col] = true;
            }
    }

    /** Rasterizes a filled polygon. */
    private void rasterizeFilledPolygon(List<double[]> ring) {
        if (ring == null || ring.size() < 3) return;

        double minX = Double.MAX_VALUE, maxX = -Double.MAX_VALUE;
        double minY = Double.MAX_VALUE, maxY = -Double.MAX_VALUE;
        for (double[] p : ring) {
            minX = Math.min(minX, p[0]); maxX = Math.max(maxX, p[0]);
            minY = Math.min(minY, p[1]); maxY = Math.max(maxY, p[1]);
        }
        int colMin = Math.max(0, pathUtils.worldToGridCol(minX));
        int colMax = Math.min(pathUtils.gridW - 1,
                              pathUtils.worldToGridCol(maxX));
        int rowMin = Math.max(0, pathUtils.worldToGridRow(minY));
        int rowMax = Math.min(pathUtils.gridH - 1,
                              pathUtils.worldToGridRow(maxY));

        int n = ring.size();

        for (int row = rowMin; row <= rowMax; row++) {
            double cy = (row + 0.5) * pathUtils.gridCellM;

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
                int cLeft  = Math.max(colMin,
                                      pathUtils.worldToGridCol(xLeft));
                int cRight = Math.min(colMax,
                                      pathUtils.worldToGridCol(xRight));
                for (int col = cLeft; col <= cRight; col++)
                    obstacleGrid[row][col] = true;
            }
        }
    }

    /** Publishes a snapshot of the grid for GUI rendering. */
    private void publishPlanningGridSnapshot() {
        boolean[][] copy = new boolean[pathUtils.gridH][pathUtils.gridW];
        for (int r = 0; r < pathUtils.gridH; r++)
            System.arraycopy(obstacleGrid[r], 0, copy[r], 0, pathUtils.gridW);
        planningGridSnapshot = new PlanningGridSnapshot(
            pathUtils.gridCellM, pathUtils.gridW, pathUtils.gridH, copy);
    }

    /*
     * Finds the nearest free cell to a coordinate using BFS.
     */
    public Coord snapToNearestFreeCell(Coord c) {
        int[] s = pathUtils.worldToGrid(c);
        s[0] = Math.max(0, Math.min(pathUtils.gridW - 1, s[0]));
        s[1] = Math.max(0, Math.min(pathUtils.gridH - 1, s[1]));
        if (!obstacleGrid[s[1]][s[0]]) return c.clone();
        ArrayDeque<int[]> q = new ArrayDeque<>();
        boolean[][] seen = new boolean[pathUtils.gridH][pathUtils.gridW];
        q.add(s); seen[s[1]][s[0]] = true;
        int[][] dirs = {{1,0},{-1,0},{0,1},{0,-1},
                         {1,1},{1,-1},{-1,1},{-1,-1}};
        while (!q.isEmpty()) {
            int[] cur = q.poll();
            if (!obstacleGrid[cur[1]][cur[0]])
                return pathUtils.gridToWorld(cur[0], cur[1]);
            for (int[] d : dirs) {
                int nc = cur[0] + d[0], nr = cur[1] + d[1];
                if (!pathUtils.inBounds(nc, nr) || seen[nr][nc]) continue;
                seen[nr][nc] = true;
                q.add(new int[]{nc, nr});
            }
        }
        throw new SimError("UAVWaypointMovement: no free cell near " + c);
    }

    /** Checks if a world position is blocked. */
    public boolean isBlockedWorldXY(double x, double y) {
        int col = pathUtils.worldToGridCol(x);
        int row = pathUtils.worldToGridRow(y);
        return obstacleGrid[row][col];
    }

    /**
     * Calculates shortest distance to any obstacle surface.
     */
    public double nearestObstacleDistance(Coord p) {
        double minD = Double.MAX_VALUE, px = p.getX(), py = p.getY();
        if (obstacleDiscs != null)
            for (double[] d : obstacleDiscs)
                minD = Math.min(minD,
                    Math.max(0, Math.hypot(px - d[0], py - d[1]) - d[2]));
        if (obstacleSegBuf != null)
            for (double[] s : obstacleSegBuf)
                minD = Math.min(minD, Math.max(0,
                    UavPathUtils.pointToSegmentDist(p,
                        new Coord(s[0], s[1]),
                        new Coord(s[2], s[3])) - s[4]));
        return minD == Double.MAX_VALUE ? distAlert * 2 : minD;
    }

     /**
     * Resets shared static state between simulation runs.
     */
    public static synchronized void reset() {
        obstacleGrid         = null;
        obstacleDiscs        = null;
        obstacleSegBuf       = null;
        obstacleRenderData   = null;
        planningGridSnapshot = null;
        gridRenderingEnabled = true;
        loadedWktFiles.clear();
        geoMinX = Double.MAX_VALUE;  geoMaxX = -Double.MAX_VALUE;
        geoMinY = Double.MAX_VALUE;  geoMaxY = -Double.MAX_VALUE;
        geoScaleInitialised = false;
        geoOriginLon   = 78.300;
        geoOriginLat   = 17.480;
        geoScaleFactor = 100000.0;
    }
}
