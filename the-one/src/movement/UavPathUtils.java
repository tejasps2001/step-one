package movement;

import core.Coord;

import java.util.ArrayList;
import java.util.List;

/**
 * Grid-coordinate conversion utilities and pure-geometry helpers used by
 * {@link UAVWaypointMovement} and {@link UavObstacleGrid}.
 *
 * <p>An instance of this class is created once per movement-model instance,
 * capturing the fixed grid parameters ({@code gridCellM}, {@code gridW},
 * {@code gridH}) and the world dimensions.  The two static methods
 * ({@link #pointToSegmentDist} and {@link #normaliseAngle}) are stateless
 * geometry routines.
 */
public class UavPathUtils {

    /** Side length of one A* grid cell (metres). */
    final double gridCellM;

    /** Number of grid columns. */
    final int gridW;

    /** Number of grid rows. */
    final int gridH;

    /** World width (metres). */
    final double worldW;

    /** World height (metres). */
    final double worldH;

    public UavPathUtils(double gridCellM, int gridW, int gridH,
                        double worldW, double worldH) {
        this.gridCellM = gridCellM;
        this.gridW     = gridW;
        this.gridH     = gridH;
        this.worldW    = worldW;
        this.worldH    = worldH;
    }

    // ================================================================ //
    //  World ↔ Grid conversions
    // ================================================================ //

    /** Converts a world coordinate to grid (col, row). */
    public int[] worldToGrid(Coord c) {
        return new int[]{ worldToGridCol(c.getX()), worldToGridRow(c.getY()) };
    }

    /** Converts a world X value to a grid column index, clamped to [0, gridW-1]. */
    public int worldToGridCol(double x) {
        return Math.max(0, Math.min((int)(x / gridCellM), gridW - 1));
    }

    /** Converts a world Y value to a grid row index, clamped to [0, gridH-1]. */
    public int worldToGridRow(double y) {
        return Math.max(0, Math.min((int)(y / gridCellM), gridH - 1));
    }

    /** Converts grid (col, row) back to the centre of that cell in world coords. */
    public Coord gridToWorld(int col, int row) {
        return new Coord((col + 0.5) * gridCellM, (row + 0.5) * gridCellM);
    }

    /** Converts a list of grid cells to world-coordinate waypoints. */
    public List<Coord> gridPathToWorld(List<int[]> cells) {
        List<Coord> wps = new ArrayList<>(cells.size());
        for (int[] c : cells) wps.add(gridToWorld(c[0], c[1]));
        return wps;
    }

    /** Flattens (col, row) into a single array index. */
    public int cellIndex(int col, int row) {
        return row * gridW + col;
    }

    /** Returns {@code true} if (col, row) is inside the grid. */
    public boolean inBounds(int col, int row) {
        return col >= 0 && col < gridW && row >= 0 && row < gridH;
    }

    /** Clamps a world coordinate to the valid world area (1-pixel inset). */
    public Coord clampToWorld(Coord c) {
        return new Coord(
            Math.max(1, Math.min(Math.max(2, worldW - 1), c.getX())),
            Math.max(1, Math.min(Math.max(2, worldH - 1), c.getY())));
    }

    // ================================================================ //
    //  Pure-static geometry helpers
    // ================================================================ //

    /**
     * Shortest distance from point {@code p} to segment {@code a–b}.
     */
    public static double pointToSegmentDist(Coord p, Coord a, Coord b) {
        double ax = a.getX(), ay = a.getY(), bx = b.getX(), by = b.getY();
        double px = p.getX(), py = p.getY(), dx = bx - ax, dy = by - ay;
        double ls = dx * dx + dy * dy;
        if (ls < 1e-9) return p.distance(a);
        double t = Math.max(0, Math.min(1,
                   ((px - ax) * dx + (py - ay) * dy) / ls));
        return Math.hypot(px - (ax + t * dx), py - (ay + t * dy));
    }

    /**
     * Normalises an angle to the range (−π, +π].
     */
    public static double normaliseAngle(double a) {
        while (a >  Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }
}
