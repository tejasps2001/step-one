package movement;

import core.Coord;
import core.SimError;

import java.util.*;

/**
 * ============================================================
 *  UavPathUtils — Stateless geometry and planning utilities
 * ============================================================
 *
 * All methods are pure functions (no mutable state, no static fields).
 * They are package-private so that {@link UAVWaypointMovement} can call them
 * directly without extra verbosity, and unit tests in the same package can
 * reach them without reflection.
 *
 * Sections
 * --------
 *  1. Grid ↔ world coordinate conversions
 *  2. Obstacle distance queries
 *  3. Geometry primitives (angle, clamp, segment distance)
 *  4. Bresenham line-of-sight check
 *  5. BFS snap-to-nearest-free-cell
 *  6. POI tour construction (nearest-neighbour heuristic)
 *
 * ============================================================
 */
final class UavPathUtils {

    // Prevent instantiation.
    private UavPathUtils() {}

    // ================================================================== //
    //  1. Grid ↔ world conversions                                         //
    // ================================================================== //

    /** Converts a world coordinate to a [col, row] grid cell index pair. */
    static int[] worldToGrid(Coord c, double gridCellM, int gridW, int gridH) {
        return new int[]{ worldToCol(c.getX(), gridW, gridCellM),
                          worldToRow(c.getY(), gridH, gridCellM) };
    }

    static int worldToCol(double x, int gridW, double gridCellM) {
        return Math.max(0, Math.min((int)(x / gridCellM), gridW - 1));
    }

    static int worldToRow(double y, int gridH, double gridCellM) {
        return Math.max(0, Math.min((int)(y / gridCellM), gridH - 1));
    }

    /** Returns the world-space centre of cell (col, row). */
    static Coord gridToWorld(int col, int row, double gridCellM) {
        return new Coord((col + 0.5) * gridCellM, (row + 0.5) * gridCellM);
    }

    /** Converts a grid-cell path to world-space {@link Coord} waypoints. */
    static List<Coord> gridPathToWorld(List<int[]> cells, double gridCellM) {
        List<Coord> wps = new ArrayList<>(cells.size());
        for (int[] c : cells) wps.add(gridToWorld(c[0], c[1], gridCellM));
        return wps;
    }

    /** Flat index of (col, row) in a row-major grid of width {@code gridW}. */
    static int cellIndex(int col, int row, int gridW) {
        return row * gridW + col;
    }

    /** Returns true if (col, row) is inside the grid bounds. */
    static boolean inBounds(int col, int row, int gridW, int gridH) {
        return col >= 0 && col < gridW && row >= 0 && row < gridH;
    }

    // ================================================================== //
    //  2. Obstacle distance queries                                         //
    // ================================================================== //

    /**
     * Returns the shortest Euclidean distance from {@code p} to the nearest
     * obstacle primitive.  If no primitives have been loaded yet the method
     * returns {@code fallback} (typically {@code distAlert * 2}, meaning
     * "safely far away").
     *
     * @param p              query point in world metres
     * @param obstacleDiscs  list of [cx, cy, r] disc primitives (may be null)
     * @param obstacleSegBuf list of [ax, ay, bx, by, hw] segment primitives (may be null)
     * @param fallback       value returned when neither list has entries
     */
    static double nearestObstacleDistance(Coord p,
                                          List<double[]> obstacleDiscs,
                                          List<double[]> obstacleSegBuf,
                                          double fallback) {
        double minD = Double.MAX_VALUE;
        double px = p.getX(), py = p.getY();

        if (obstacleDiscs != null)
            for (double[] d : obstacleDiscs)
                minD = Math.min(minD, Math.max(0, Math.hypot(px - d[0], py - d[1]) - d[2]));

        if (obstacleSegBuf != null)
            for (double[] s : obstacleSegBuf)
                minD = Math.min(minD, Math.max(0,
                    pointToSegmentDist(p, new Coord(s[0], s[1]), new Coord(s[2], s[3])) - s[4]));

        return minD == Double.MAX_VALUE ? fallback : minD;
    }

    // ================================================================== //
    //  3. Geometry primitives                                               //
    // ================================================================== //

    /**
     * Returns the minimum distance from point {@code p} to the line segment
     * {@code [a, b]}.
     */
    static double pointToSegmentDist(Coord p, Coord a, Coord b) {
        double ax = a.getX(), ay = a.getY();
        double bx = b.getX(), by = b.getY();
        double px = p.getX(), py = p.getY();
        double dx = bx - ax, dy = by - ay;
        double ls = dx * dx + dy * dy;
        if (ls < 1e-9) return p.distance(a);
        double t = Math.max(0, Math.min(1, ((px - ax) * dx + (py - ay) * dy) / ls));
        return Math.hypot(px - (ax + t * dx), py - (ay + t * dy));
    }

    /**
     * Normalises angle {@code a} to (-π, π].
     */
    static double normaliseAngle(double a) {
        while (a >  Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }

    /**
     * Clamps {@code c} to the interior of the world rectangle
     * [1, worldW-1] × [1, worldH-1].
     */
    static Coord clampToWorld(Coord c, double worldW, double worldH) {
        return new Coord(
            Math.max(1, Math.min(Math.max(2, worldW - 1), c.getX())),
            Math.max(1, Math.min(Math.max(2, worldH - 1), c.getY())));
    }

    // ================================================================== //
    //  4. Bresenham line-of-sight check                                    //
    // ================================================================== //

    /**
     * Returns {@code true} if the straight line from {@code a} to {@code b}
     * passes through no blocked cell in {@code obstacleGrid}, as determined
     * by Bresenham's line algorithm.
     *
     * @param a             world-space start
     * @param b             world-space end
     * @param obstacleGrid  [row][col] blocked flags
     * @param gridCellM     metres per cell side
     * @param gridW         grid width
     * @param gridH         grid height
     */
    static boolean bresenhamLOS(Coord a, Coord b,
                                 boolean[][] obstacleGrid,
                                 double gridCellM, int gridW, int gridH) {
        int[] ca = worldToGrid(a, gridCellM, gridW, gridH);
        int[] cb = worldToGrid(b, gridCellM, gridW, gridH);
        int x0 = ca[0], y0 = ca[1], x1 = cb[0], y1 = cb[1];
        int dx = Math.abs(x1 - x0), dy = Math.abs(y1 - y0);
        int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
        int err = dx - dy, x = x0, y = y0;
        while (true) {
            if (!inBounds(x, y, gridW, gridH) || obstacleGrid[y][x]) return false;
            if (x == x1 && y == y1) break;
            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x += sx; }
            if (e2 <  dx) { err += dx; y += sy; }
        }
        return true;
    }

    /**
     * Extracts key turning-point waypoints from the raw A* path using the
     * Bresenham line-of-sight check (paper Fig. 2).
     *
     * @param raw           raw waypoint list from A*
     * @param obstacleGrid  [row][col] blocked flags
     * @param gridCellM     metres per cell side
     * @param gridW         grid width
     * @param gridH         grid height
     * @return compressed list containing only necessary waypoints
     */
    static List<Coord> bresenhamExtractKeyNodes(List<Coord> raw,
                                                boolean[][] obstacleGrid,
                                                double gridCellM,
                                                int gridW, int gridH) {
        List<Coord> keys = new ArrayList<>();
        if (raw.size() <= 1) { keys.addAll(raw); return keys; }
        int start = 0;
        keys.add(raw.get(0));
        while (start < raw.size() - 1) {
            int lastSafe = start + 1;
            for (int j = start + 2; j < raw.size(); j++) {
                if (bresenhamLOS(raw.get(start), raw.get(j),
                                 obstacleGrid, gridCellM, gridW, gridH))
                    lastSafe = j;
                else
                    break;
            }
            keys.add(raw.get(lastSafe));
            start = lastSafe;
        }
        return keys;
    }

    // ================================================================== //
    //  5. BFS snap-to-nearest-free-cell                                    //
    // ================================================================== //

    /**
     * Snaps {@code c} to the nearest cell in {@code obstacleGrid} that is
     * not blocked.  Uses a BFS so the result is always the globally closest
     * free cell (in grid-hop distance).
     *
     * @throws SimError if the entire grid is blocked (degenerate scenario)
     */
    static Coord snapToNearestFreeCell(Coord c,
                                       boolean[][] obstacleGrid,
                                       double gridCellM, int gridW, int gridH) {
        int[] s = worldToGrid(c, gridCellM, gridW, gridH);
        s[0] = Math.max(0, Math.min(gridW - 1, s[0]));
        s[1] = Math.max(0, Math.min(gridH - 1, s[1]));
        if (!obstacleGrid[s[1]][s[0]]) return gridToWorld(s[0], s[1], gridCellM);

        ArrayDeque<int[]> q = new ArrayDeque<>();
        boolean[][] seen = new boolean[gridH][gridW];
        q.add(s);
        seen[s[1]][s[0]] = true;
        int[][] dirs = { {1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1} };
        while (!q.isEmpty()) {
            int[] cur = q.poll();
            if (!obstacleGrid[cur[1]][cur[0]])
                return gridToWorld(cur[0], cur[1], gridCellM);
            for (int[] d : dirs) {
                int nc = cur[0] + d[0], nr = cur[1] + d[1];
                if (!inBounds(nc, nr, gridW, gridH) || seen[nr][nc]) continue;
                seen[nr][nc] = true;
                q.add(new int[]{ nc, nr });
            }
        }
        throw new SimError("UAVWaypointMovement: no free cell near " + c);
    }

    // ================================================================== //
    //  6. POI tour construction                                             //
    // ================================================================== //

    /**
     * Builds a lattice of POI waypoints evenly spread across the world, then
     * solves a nearest-neighbour tour starting from {@code origin}.  Grid
     * cells that are already blocked are excluded.
     *
     * @param origin         current UAV position (tour start)
     * @param poiGridCols    number of POI columns (0 = no POIs)
     * @param poiGridRows    number of POI rows    (0 = no POIs)
     * @param worldW         world width in metres
     * @param worldH         world height in metres
     * @param obstacleGrid   [row][col] blocked flags
     * @param gridCellM      metres per cell side
     * @param gridW          grid width
     * @param gridH          grid height
     * @param snap           if true, POIs are snapped to the nearest free cell
     * @return ordered list of POI waypoints (may be empty)
     */
    static List<Coord> buildPoiGridWaypoints(Coord origin,
                                             int poiGridCols, int poiGridRows,
                                             double worldW, double worldH,
                                             boolean[][] obstacleGrid,
                                             double gridCellM, int gridW, int gridH,
                                             boolean snap) {
        if (poiGridCols <= 0 || poiGridRows <= 0) return new ArrayList<>();
        List<Coord> lattice = new ArrayList<>();
        for (int ix = 0; ix < poiGridCols; ix++)
            for (int iy = 0; iy < poiGridRows; iy++) {
                double x = (ix + 0.5) * worldW / poiGridCols;
                double y = (iy + 0.5) * worldH / poiGridRows;
                int col = worldToCol(x, gridW, gridCellM);
                int row = worldToRow(y, gridH, gridCellM);
                if (obstacleGrid[row][col]) continue;
                Coord c = new Coord(x, y);
                if (snap) c = snapToNearestFreeCell(c, obstacleGrid, gridCellM, gridW, gridH);
                lattice.add(c);
            }
        return nearestNeighbourTour(origin, lattice);
    }

    /**
     * Greedy nearest-neighbour tour through {@code pois} starting from
     * {@code start}.  Runs in O(n²) which is fine for typical small POI sets.
     */
    static List<Coord> nearestNeighbourTour(Coord start, List<Coord> pois) {
        List<Coord> rem = new ArrayList<>(pois);
        List<Coord> ord = new ArrayList<>(pois.size());
        Coord cur = start;
        while (!rem.isEmpty()) {
            Coord nn = null;
            double md = Double.MAX_VALUE;
            for (Coord c : rem) {
                double d = cur.distance(c);
                if (d < md) { md = d; nn = c; }
            }
            ord.add(nn);
            rem.remove(nn);
            cur = nn;
        }
        return ord;
    }
}
