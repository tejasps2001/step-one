package movement;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Minimal WKT reader for obstacle geometry: {@code POINT} and {@code LINESTRING}.
 * One geometry per non-empty, non-comment line. Coordinates are 2D (x y); optional
 * third numbers are ignored.
 */
public final class WktObstacleParser {

    private static final Pattern POINT_PAT = Pattern.compile(
            "^POINT\\s*\\(\\s*([^)]+)\\)",
            Pattern.CASE_INSENSITIVE);
    private static final Pattern LINESTRING_PAT = Pattern.compile(
            "^LINESTRING\\s*\\(\\s*([^)]+)\\)",
            Pattern.CASE_INSENSITIVE);

    public static final class Parsed {
        /** Each entry: {@code [x, y]} in world metres */
        public final List<double[]> points = new ArrayList<>();
        /** Each linestring: ordered vertices {@code [x, y]} */
        public final List<List<double[]>> linestrings = new ArrayList<>();
    }

    public static Parsed parse(Path path) throws IOException {
        Parsed out = new Parsed();
        List<String> lines = Files.readAllLines(path, StandardCharsets.UTF_8);
        for (String raw : lines) {
            String line = stripComment(raw).trim();
            if (line.isEmpty()) {
                continue;
            }
            String upper = line.toUpperCase(Locale.ROOT);
            if (upper.startsWith("POINT")) {
                Matcher m = POINT_PAT.matcher(line);
                if (!m.find()) {
                    throw new IOException("Bad POINT syntax: " + line);
                }
                double[] xy = parseCoordinatePair(m.group(1));
                out.points.add(xy);
            } else if (upper.startsWith("LINESTRING")) {
                Matcher m = LINESTRING_PAT.matcher(line);
                if (!m.find()) {
                    throw new IOException("Bad LINESTRING syntax: " + line);
                }
                out.linestrings.add(parseVertexList(m.group(1)));
            } else {
                throw new IOException("Unsupported WKT type (use POINT or LINESTRING): " + line);
            }
        }
        return out;
    }

    private static String stripComment(String raw) {
        int hash = raw.indexOf('#');
        if (hash < 0) {
            return raw;
        }
        return raw.substring(0, hash);
    }

    private static List<double[]> parseVertexList(String inner) throws IOException {
        String[] parts = inner.split(",");
        List<double[]> verts = new ArrayList<>();
        for (String part : parts) {
            verts.add(parseCoordinatePair(part));
        }
        if (verts.size() < 2) {
            throw new IOException("LINESTRING needs at least 2 vertices: " + inner);
        }
        return verts;
    }

    private static double[] parseCoordinatePair(String token) throws IOException {
        String[] nums = token.trim().split("\\s+");
        if (nums.length < 2) {
            throw new IOException("Expected at least x y in: " + token);
        }
        double x = Double.parseDouble(nums[0]);
        double y = Double.parseDouble(nums[1]);
        return new double[] { x, y };
    }
}
