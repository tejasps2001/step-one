package movement;

import core.Coord;

import java.util.ArrayList;
import java.util.List;

/**
 * Grid-coordinate conversion and geometry helpers for UAV movement.
 */
public class UavPathUtils {

    final double gridCellM; // Cell size in meters
    final int gridW;        // Grid columns
    final int gridH;        // Grid rows
    final double worldW;    // World width in meters
    final double worldH;    // World height in meters

    /**
     * Initializes grid and world parameters.
     */
    public UavPathUtils(double gridCellM, int gridW, int gridH,
                        double worldW, double worldH) {
        this.gridCellM = gridCellM;
        this.gridW     = gridW;
        this.gridH     = gridH;
        this.worldW    = worldW;
        this.worldH    = worldH;
    }

    /** Convert world coordinates to grid (col, row). */
    public int[] worldToGrid(Coord c) {
        return new int[]{ worldToGridCol(c.getX()), worldToGridRow(c.getY()) };
    }

    /** Convert world X to grid column index. */
    public int worldToGridCol(double x) {
        return Math.max(0, Math.min((int)(x / gridCellM), gridW - 1));
    }

    /** Convert world Y to grid row index. */
    public int worldToGridRow(double y) {
        return Math.max(0, Math.min((int)(y / gridCellM), gridH - 1));
    }

    /** Convert grid (col, row) to world coordinates (cell center). */
    public Coord gridToWorld(int col, int row) {
        return new Coord((col + 0.5) * gridCellM, (row + 0.5) * gridCellM);
    }

    /** Convert a list of grid cells to world waypoints. */
    public List<Coord> gridPathToWorld(List<int[]> cells) {
        List<Coord> wps = new ArrayList<>(cells.size());
        for (int[] c : cells) wps.add(gridToWorld(c[0], c[1]));
        return wps;
    }

    /** Flatten (col, row) to a 1D array index. */
    public int cellIndex(int col, int row) {
        return row * gridW + col;
    }

    /** Check if a grid coordinate is within bounds. */
    public boolean inBounds(int col, int row) {
        return col >= 0 && col < gridW && row >= 0 && row < gridH;
    }

    /** Clamp coordinates to the world area. */
    public Coord clampToWorld(Coord c) {
        return new Coord(
            Math.max(1, Math.min(Math.max(2, worldW - 1), c.getX())),
            Math.max(1, Math.min(Math.max(2, worldH - 1), c.getY())));
    }

    /**
     * Distance from point p to line segment a-b.
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
     * Normalize an angle to [-PI, PI].
     */
    public static double normaliseAngle(double a) {
        while (a >  Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }
}
