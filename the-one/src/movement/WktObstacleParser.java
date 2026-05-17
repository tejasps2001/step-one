package movement;

import core.SimError;

import java.io.IOException;
import java.nio.charset.Charset;
import java.nio.charset.MalformedInputException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Utility for parsing WKT (Well-Known Text) geometries.
 * Supports POINT, LINESTRING, POLYGON, and their MULTI-variants.
 */
public final class WktObstacleParser {

    private WktObstacleParser() { /* utility class */ }

    static final Charset[] WKT_CHARSETS = { // Charsets for reading WKT files
        StandardCharsets.UTF_8,
        Charset.forName("Windows-1252"),
        StandardCharsets.ISO_8859_1
    };

    /** Parses a CSV string of file paths into a list. */
    public static List<String> parseObstacleFilePaths(String raw) {
        List<String> result = new ArrayList<>();
        if (raw == null || raw.trim().isEmpty()) return result;
        for (String token : raw.split(",")) {
            String path = token.trim();
            if (!path.isEmpty()) result.add(path);
        }
        return result;
    }

    /** Reads WKT file lines with charset fallback. Throws SimError on failure. */
    public static List<String> readWktFile(Path path) {
        if (!Files.exists(path)) {
            throw new SimError(
                "UAVWaypointMovement: WKT file not found: " + path
                + "\n  Check that the path in your settings file is correct"
                + " and that the file exists relative to the ONE root directory.");
        }
        if (!Files.isReadable(path)) {
            throw new SimError(
                "UAVWaypointMovement: WKT file exists but is not readable"
                + " (check OS permissions): " + path);
        }

        List<String> lines = null;
        IOException lastEx = null;
        for (Charset cs : WKT_CHARSETS) {
            try {
                lines = Files.readAllLines(path, cs);
                break;
            } catch (MalformedInputException mie) {
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
        return lines;
    }

    /** Parses a WKT string and extracts points, lines, and polygons. */
    public static void extractGeometry(String wkt,
                                       List<double[]> points,
                                       List<List<double[]>> linestrings,
                                       List<List<double[]>> polygons) {
        String upper = wkt.trim().replaceAll("\\s+", " ")
                          .toUpperCase(Locale.ROOT);
        upper = upper.replaceAll("\\b(Z M|ZM|M|Z)\\s*(?=\\()", "");

        if (upper.startsWith("GEOMETRYCOLLECTION")) {
            String inner = extractOuterBody(wkt);
            if (inner == null) return;
            for (String member : splitTopLevelCommas(inner)) {
                extractGeometry(member.trim(), points, linestrings, polygons);
            }

        } else if (upper.startsWith("MULTIPOLYGON")) {
            String inner = extractOuterBody(wkt);
            if (inner == null) return;
            for (String polyBody : splitTopLevelCommas(inner)) {
                extractGeometry("POLYGON " + polyBody.trim(),
                                points, linestrings, polygons);
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

        } else if (upper.startsWith("MULTILINESTRING")
                || upper.startsWith("MULTISTRING")) {
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
            System.err.println("[UAVWaypointMovement] Skipping unsupported"
                    + " WKT type: " + upper.split("\\s+|\\(")[0]);
        }
    }

    /** Calculates the geographic bounding box [minX, minY, maxX, maxY] of a WKT file. */
    public static double[] computeBoundingBox(String relPath) {
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
                if (line.isEmpty() || line.startsWith("#")
                        || line.startsWith("//")) continue;
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

    /** Extracts the content inside the outermost parentheses. */
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

    /** Splits a string by top-level commas (not nested in parentheses). */
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

    /** Parses a comma-separated sequence of coordinates. */
    static List<double[]> parseCoordSequence(String seq) {
        List<double[]> coords = new ArrayList<>();
        if (seq == null || seq.trim().isEmpty()) return coords;
        for (String token : seq.split(",")) {
            double[] c = parseOneCoord(token.trim());
            if (c != null) coords.add(c);
        }
        return coords;
    }

    /** Parses a single "X Y" coordinate pair. */
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
}
