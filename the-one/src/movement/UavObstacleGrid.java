package movement;

import core.Coord;
import core.SimError;

import java.io.IOException;
import java.nio.charset.Charset;
import java.nio.charset.MalformedInputException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.*;

/**
 * ============================================================
 *  UavObstacleGrid — Shared obstacle state for UAVWaypointMovement
 * ============================================================
 *
 * Owns all static obstacle data shared across every UAV instance:
 *
 *   • obstacleGrid   — boolean[gridH][gridW] raster used by A* and DWA
 *   • obstacleDiscs  — exact (cx, cy, r) tuples for continuous distance queries
 *   • obstacleSegBuf — exact (ax, ay, bx, by, hw) tuples for continuous distance queries
 *   • obstacleRenderData — geometry records for the GUI overlay
 *   • planningGridSnapshot — immutable snapshot passed to the GUI renderer
 *
 * All mutation is performed inside the class-level lock of
 * {@link UAVWaypointMovement}; callers acquire that lock before invoking
 * {@link #mergeIfNeeded}.
 *
 * Responsibilities
 * ----------------
 *  1. Lazy grid initialisation on first call to {@link #mergeIfNeeded}.
 *  2. WKT file loading (multi-charset fallback, relative-path resolution).
 *  3. WKT geometry extraction: POINT, MULTIPOINT, LINESTRING,
 *     MULTILINESTRING / MULTISTRING, POLYGON, MULTIPOLYGON, GEOMETRYCOLLECTION.
 *  4. Geographic coordinate normalisation (lon/lat → sim metres).
 *  5. Rasterization: disc, strip, filled-polygon.
 *  6. Publishing the {@link UAVWaypointMovement.PlanningGridSnapshot} after
 *     each merge round.
 *  7. Full state reset for simulator re-runs ({@link #reset}).
 *
 * ============================================================
 */
final class UavObstacleGrid {

    // ------------------------------------------------------------------ //
    //  Shared static obstacle state                                        //
    // ------------------------------------------------------------------ //

    static boolean[][]    obstacleGrid   = null;
    static List<double[]> obstacleDiscs  = null;
    static List<double[]> obstacleSegBuf = null;

    static List<UAVWaypointMovement.ObstacleRenderData> obstacleRenderData = null;

    /** Canonical paths of WKT files already loaded — prevents double-loading. */
    static final Set<String> loadedWktFiles =
            Collections.synchronizedSet(new LinkedHashSet<>());

    // ------------------------------------------------------------------ //
    //  Geographic transform state                                          //
    // ------------------------------------------------------------------ //

    static double  geoMinX = Double.MAX_VALUE,  geoMaxX = -Double.MAX_VALUE;
    static double  geoMinY = Double.MAX_VALUE,  geoMaxY = -Double.MAX_VALUE;
    static boolean geoScaleInitialised = false;

    static double geoOriginLon   = 78.300;
    static double geoOriginLat   = 17.480;
    static double geoScaleFactor = 100000.0;

    // ------------------------------------------------------------------ //
    //  Grid snapshot (GUI)                                                 //
    // ------------------------------------------------------------------ //

    static UAVWaypointMovement.PlanningGridSnapshot planningGridSnapshot = null;

    // ------------------------------------------------------------------ //
    //  Charset fallback order for WKT reading                             //
    // ------------------------------------------------------------------ //

    private static final Charset[] WKT_CHARSETS = {
        StandardCharsets.UTF_8,
        Charset.forName("Windows-1252"),
        StandardCharsets.ISO_8859_1
    };

    // Prevent instantiation — all state is static.
    private UavObstacleGrid() {}

    // ================================================================== //
    //  Public API                                                          //
    // ================================================================== //

    /**
     * Merges obstacles from {@code wktFilePaths} into the shared grid.
     * Already-loaded files (tracked by canonical path) are skipped.
     * Must be called while holding the lock on {@link UAVWaypointMovement}.
     *
     * @param wktFilePaths list of WKT file paths to load (may be empty)
     * @param gridW        number of grid columns
     * @param gridH        number of grid rows
     * @param gridCellM    metres per cell side
     * @param pointObstacleRadius    buffer radius for POINT obstacles (m)
     * @param lineObstacleHalfWidth  half-width for LINESTRING obstacles (m)
     */
    static void mergeIfNeeded(List<String> wktFilePaths,
                              int gridW, int gridH, double gridCellM,
                              double pointObstacleRadius,
                              double lineObstacleHalfWidth) {
        // Lazy grid initialisation
        if (obstacleGrid == null) {
            obstacleGrid       = new boolean[gridH][gridW];
            obstacleDiscs      = new ArrayList<>();
            obstacleSegBuf     = new ArrayList<>();
            obstacleRenderData = new ArrayList<>();
            publishSnapshot(gridCellM, gridW, gridH);
        }

        if (wktFilePaths == null || wktFilePaths.isEmpty()) return;

        boolean anyNew = false;
        for (String wkt : wktFilePaths) {
            if (wkt == null || wkt.trim().isEmpty()) continue;
            String trimmed = wkt.trim();

            String canonical = resolveCanonical(trimmed);
            if (!loadedWktFiles.add(canonical)) continue;

            loadFromWkt(trimmed, gridW, gridH, gridCellM,
                        pointObstacleRadius, lineObstacleHalfWidth);
            anyNew = true;
        }

        if (anyNew) {
            publishSnapshot(gridCellM, gridW, gridH);
        }
    }

    /**
     * Resets all shared state — call before each simulator re-run.
     */
    static void reset() {
        obstacleGrid         = null;
        obstacleDiscs        = null;
        obstacleSegBuf       = null;
        obstacleRenderData   = null;
        planningGridSnapshot = null;
        loadedWktFiles.clear();
        geoMinX = Double.MAX_VALUE;  geoMaxX = -Double.MAX_VALUE;
        geoMinY = Double.MAX_VALUE;  geoMaxY = -Double.MAX_VALUE;
        geoScaleInitialised  = false;
        geoOriginLon         = 78.300;
        geoOriginLat         = 17.480;
        geoScaleFactor       = 100000.0;
    }

    /**
     * Returns the list of render-data records for the GUI obstacle overlay,
     * or an empty list if none have been loaded yet.
     */
    static List<UAVWaypointMovement.ObstacleRenderData> getRenderData() {
        return obstacleRenderData != null
               ? new ArrayList<>(obstacleRenderData)
               : new ArrayList<>();
    }

    // ================================================================== //
    //  WKT file loading                                                    //
    // ================================================================== //

    private static String resolveCanonical(String rel) {
        try {
            Path p = Paths.get(rel);
            if (!p.isAbsolute())
                p = Paths.get(System.getProperty("user.dir", ".")).resolve(p).normalize();
            return Files.exists(p) ? p.toRealPath().toString() : p.toString();
        } catch (IOException e) {
            return rel;
        }
    }

    private static void loadFromWkt(String rel,
                                    int gridW, int gridH, double gridCellM,
                                    double pointObstacleRadius,
                                    double lineObstacleHalfWidth) {
        Path path = Paths.get(rel);
        if (!path.isAbsolute())
            path = Paths.get(System.getProperty("user.dir", ".")).resolve(path);

        if (!Files.exists(path))
            throw new SimError(
                "UAVWaypointMovement: WKT file not found: " + path
                + "\n  Check that the path in your settings file is correct"
                + " and that the file exists relative to the ONE root directory.");
        if (!Files.isReadable(path))
            throw new SimError(
                "UAVWaypointMovement: WKT file exists but is not readable"
                + " (check OS permissions): " + path);

        List<String> lines = readLines(path);

        List<double[]>       points      = new ArrayList<>();
        List<List<double[]>> linestrings = new ArrayList<>();
        List<List<double[]>> polygons    = new ArrayList<>();

        for (int lineNo = 0; lineNo < lines.size(); lineNo++) {
            String line = lines.get(lineNo).trim();
            if (line.isEmpty() || line.startsWith("#") || line.startsWith("//")) continue;
            try {
                extractGeometry(line, points, linestrings, polygons);
            } catch (Exception e) {
                System.err.println("[UAVWaypointMovement] Skipping malformed geometry"
                    + " at " + path.getFileName() + " line " + (lineNo + 1)
                    + ": " + e.getMessage());
            }
        }

        normaliseCoordinatesIfGeographic(points, linestrings, polygons);

        // ── Rasterize and record render data ───────────────────────────────
        for (double[] pt : points) {
            rasterizeDisc(pt[0], pt[1], pointObstacleRadius,
                          gridW, gridH, gridCellM);
            obstacleDiscs.add(new double[]{ pt[0], pt[1], pointObstacleRadius });
            if (obstacleRenderData != null)
                obstacleRenderData.add(new UAVWaypointMovement.ObstacleRenderData(
                    UAVWaypointMovement.ObstacleRenderData.Type.POINT,
                    Arrays.asList(new Coord(pt[0], pt[1])),
                    pointObstacleRadius));
        }

        for (List<double[]> ls : linestrings) {
            List<Coord> lsCoords = obstacleRenderData != null
                                   ? new ArrayList<>(ls.size()) : null;
            for (int i = 0; i < ls.size() - 1; i++) {
                double[] a = ls.get(i), b = ls.get(i + 1);
                rasterizeStrip(a[0], a[1], b[0], b[1], lineObstacleHalfWidth,
                               gridW, gridH, gridCellM);
                obstacleSegBuf.add(new double[]{ a[0], a[1], b[0], b[1], lineObstacleHalfWidth });
            }
            if (lsCoords != null) {
                for (double[] p : ls) lsCoords.add(new Coord(p[0], p[1]));
                if (lsCoords.size() >= 2)
                    obstacleRenderData.add(new UAVWaypointMovement.ObstacleRenderData(
                        UAVWaypointMovement.ObstacleRenderData.Type.LINE,
                        lsCoords, lineObstacleHalfWidth));
            }
        }

        for (List<double[]> ring : polygons) {
            rasterizeFilledPolygon(ring, gridW, gridH, gridCellM);
            for (int i = 0; i < ring.size() - 1; i++) {
                double[] a = ring.get(i), b = ring.get(i + 1);
                obstacleSegBuf.add(new double[]{ a[0], a[1], b[0], b[1], 0.0 });
            }
            if (obstacleRenderData != null) {
                List<Coord> pgCoords = new ArrayList<>(ring.size());
                for (double[] p : ring) pgCoords.add(new Coord(p[0], p[1]));
                if (pgCoords.size() >= 3)
                    obstacleRenderData.add(new UAVWaypointMovement.ObstacleRenderData(
                        UAVWaypointMovement.ObstacleRenderData.Type.POLYGON,
                        pgCoords, 0.0));
            }
        }
    }

    // ------------------------------------------------------------------ //
    //  Charset-fallback file reader                                        //
    // ------------------------------------------------------------------ //

    private static List<String> readLines(Path path) {
        IOException lastEx = null;
        for (Charset cs : WKT_CHARSETS) {
            try {
                return Files.readAllLines(path, cs);
            } catch (MalformedInputException mie) {
                lastEx = mie;
            } catch (IOException e) {
                throw new SimError(
                    "UAVWaypointMovement: I/O error reading WKT file: " + path
                    + "\n  " + e.getMessage());
            }
        }
        throw new SimError(
            "UAVWaypointMovement: WKT file could not be decoded with any"
            + " supported charset (UTF-8, Windows-1252, ISO-8859-1): " + path
            + (lastEx != null ? "\n  " + lastEx.getMessage() : ""));
    }

    // ================================================================== //
    //  WKT geometry extraction                                             //
    // ================================================================== //

    static void extractGeometry(String wkt,
                                List<double[]>       points,
                                List<List<double[]>> linestrings,
                                List<List<double[]>> polygons) {
        String upper = wkt.trim().replaceAll("\\s+", " ").toUpperCase(Locale.ROOT);
        upper = upper.replaceAll("\\b(Z M|ZM|M|Z)\\s*(?=\\()", "");

        if (upper.startsWith("GEOMETRYCOLLECTION")) {
            String inner = extractOuterBody(wkt);
            if (inner == null) return;
            for (String member : splitTopLevelCommas(inner))
                extractGeometry(member.trim(), points, linestrings, polygons);

        } else if (upper.startsWith("MULTIPOLYGON")) {
            String inner = extractOuterBody(wkt);
            if (inner == null) return;
            for (String polyBody : splitTopLevelCommas(inner))
                extractGeometry("POLYGON " + polyBody.trim(), points, linestrings, polygons);

        } else if (upper.startsWith("POLYGON")) {
            String inner = extractOuterBody(wkt);
            if (inner == null) return;
            List<String> rings = splitTopLevelCommas(inner);
            for (int ri = 0; ri < rings.size(); ri++) {
                String stripped = rings.get(ri).trim();
                if (stripped.startsWith("(") && stripped.endsWith(")"))
                    stripped = stripped.substring(1, stripped.length() - 1);
                List<double[]> ring = parseCoordSequence(stripped);
                if (ring.size() >= 3 && ri == 0) polygons.add(ring);
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

    // ------------------------------------------------------------------ //
    //  WKT string-parsing helpers                                          //
    // ------------------------------------------------------------------ //

    static String extractOuterBody(String wkt) {
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

    static List<String> splitTopLevelCommas(String s) {
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

    static List<double[]> parseCoordSequence(String seq) {
        List<double[]> coords = new ArrayList<>();
        if (seq == null || seq.trim().isEmpty()) return coords;
        for (String token : seq.split(",")) {
            double[] c = parseOneCoord(token.trim());
            if (c != null) coords.add(c);
        }
        return coords;
    }

    static double[] parseOneCoord(String token) {
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

    // ================================================================== //
    //  Geographic coordinate normalisation                                 //
    // ================================================================== //

    private static void normaliseCoordinatesIfGeographic(
            List<double[]>       points,
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

    // ================================================================== //
    //  Rasterization                                                        //
    // ================================================================== //

    private static void rasterizeDisc(double cx, double cy, double r,
                                      int gridW, int gridH, double gridCellM) {
        double r2 = r * r;
        int c0 = worldToCol(cx - r - gridCellM, gridW, gridCellM);
        int c1 = worldToCol(cx + r + gridCellM, gridW, gridCellM);
        int r0 = worldToRow(cy - r - gridCellM, gridH, gridCellM);
        int r1 = worldToRow(cy + r + gridCellM, gridH, gridCellM);
        for (int row = Math.max(0, r0); row <= Math.min(gridH - 1, r1); row++)
            for (int col = Math.max(0, c0); col <= Math.min(gridW - 1, c1); col++) {
                double gx = (col + 0.5) * gridCellM;
                double gy = (row + 0.5) * gridCellM;
                double dx = gx - cx, dy = gy - cy;
                if (dx * dx + dy * dy <= r2)
                    obstacleGrid[row][col] = true;
            }
    }

    private static void rasterizeStrip(double x0, double y0, double x1, double y1,
                                       double hw,
                                       int gridW, int gridH, double gridCellM) {
        double pad = hw + gridCellM;
        int c0 = worldToCol(Math.min(x0, x1) - pad, gridW, gridCellM);
        int c1 = worldToCol(Math.max(x0, x1) + pad, gridW, gridCellM);
        int r0 = worldToRow(Math.min(y0, y1) - pad, gridH, gridCellM);
        int r1 = worldToRow(Math.max(y0, y1) + pad, gridH, gridCellM);
        Coord a = new Coord(x0, y0), b = new Coord(x1, y1);
        for (int row = Math.max(0, r0); row <= Math.min(gridH - 1, r1); row++)
            for (int col = Math.max(0, c0); col <= Math.min(gridW - 1, c1); col++) {
                Coord cell = new Coord((col + 0.5) * gridCellM, (row + 0.5) * gridCellM);
                if (UavPathUtils.pointToSegmentDist(cell, a, b) <= hw)
                    obstacleGrid[row][col] = true;
            }
    }

    private static void rasterizeFilledPolygon(List<double[]> ring,
                                               int gridW, int gridH, double gridCellM) {
        if (ring == null || ring.size() < 3) return;
        double minX = Double.MAX_VALUE, maxX = -Double.MAX_VALUE;
        double minY = Double.MAX_VALUE, maxY = -Double.MAX_VALUE;
        for (double[] p : ring) {
            minX = Math.min(minX, p[0]); maxX = Math.max(maxX, p[0]);
            minY = Math.min(minY, p[1]); maxY = Math.max(maxY, p[1]);
        }
        int colMin = Math.max(0, worldToCol(minX, gridW, gridCellM));
        int colMax = Math.min(gridW - 1, worldToCol(maxX, gridW, gridCellM));
        int rowMin = Math.max(0, worldToRow(minY, gridH, gridCellM));
        int rowMax = Math.min(gridH - 1, worldToRow(maxY, gridH, gridCellM));
        int n = ring.size();

        for (int row = rowMin; row <= rowMax; row++) {
            double cy = (row + 0.5) * gridCellM;
            List<Double> xs = new ArrayList<>();
            for (int i = 0, j = n - 1; i < n; j = i++) {
                double ry0 = ring.get(i)[1], ry1 = ring.get(j)[1];
                double rx0 = ring.get(i)[0], rx1 = ring.get(j)[0];
                if ((ry0 <= cy && ry1 > cy) || (ry1 <= cy && ry0 > cy)) {
                    double t = (cy - ry0) / (ry1 - ry0);
                    xs.add(rx0 + t * (rx1 - rx0));
                }
            }
            if (xs.isEmpty()) continue;
            Collections.sort(xs);
            for (int k = 0; k + 1 < xs.size(); k += 2) {
                int cLeft  = Math.max(colMin, worldToCol(xs.get(k),     gridW, gridCellM));
                int cRight = Math.min(colMax, worldToCol(xs.get(k + 1), gridW, gridCellM));
                for (int col = cLeft; col <= cRight; col++)
                    obstacleGrid[row][col] = true;
            }
        }
    }

    // ------------------------------------------------------------------ //
    //  Grid coordinate helpers (local — mirror of UavPathUtils for         //
    //  use before the main instance has been created)                      //
    // ------------------------------------------------------------------ //

    private static int worldToCol(double x, int gridW, double gridCellM) {
        return Math.max(0, Math.min((int)(x / gridCellM), gridW - 1));
    }

    private static int worldToRow(double y, int gridH, double gridCellM) {
        return Math.max(0, Math.min((int)(y / gridCellM), gridH - 1));
    }

    // ================================================================== //
    //  Grid snapshot publication                                           //
    // ================================================================== //

    static void publishSnapshot(double gridCellM, int gridW, int gridH) {
        boolean[][] copy = new boolean[gridH][gridW];
        for (int r = 0; r < gridH; r++)
            System.arraycopy(obstacleGrid[r], 0, copy[r], 0, gridW);
        planningGridSnapshot =
            new UAVWaypointMovement.PlanningGridSnapshot(gridCellM, gridW, gridH, copy);
    }

    // ================================================================== //
    //  Bounding-box scanner (used for geo-extent auto-detection)          //
    // ================================================================== //

    /**
     * Returns the geographic bounding box [minX, minY, maxX, maxY] of a WKT
     * file if its coordinates look geographic (lon/lat), or {@code null} if
     * they appear to already be in simulation metres.
     */
    static double[] computeWktBoundingBox(String relPath) {
        try {
            Path path = Paths.get(relPath);
            if (!path.isAbsolute())
                path = Paths.get(System.getProperty("user.dir", "."))
                           .resolve(path).normalize();
            if (!Files.exists(path)) return null;

            List<double[]>       pts = new ArrayList<>();
            List<List<double[]>> lss = new ArrayList<>(), pgs = new ArrayList<>();

            List<String> lines;
            try {
                lines = Files.readAllLines(path, StandardCharsets.UTF_8);
            } catch (MalformedInputException e) {
                lines = Files.readAllLines(path, StandardCharsets.ISO_8859_1);
            }

            for (String line : lines) {
                line = line.trim();
                if (line.isEmpty() || line.startsWith("#") || line.startsWith("//")) continue;
                try { extractGeometry(line, pts, lss, pgs); }
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
}
