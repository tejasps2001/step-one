/*
 * Optional UAV obstacle visualization. Renders obstacle shapes from original
 * WKT coordinate data — completely independent of the planning-grid toggle.
 */
package gui.uavviz;

import gui.playfield.PlayFieldGraphic;
import movement.UavObstacleGrid;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.util.List;

/**
 * Draws obstacle shapes (points, lines, polygons) on the GUI.
 * Uses data from {@link UavObstacleGrid} and original WKT coordinates.
 */
public class ObstacleOverlayGraphic extends PlayFieldGraphic {

    // Colors for drawing obstacles
    private static final Color OBSTACLE_FILL   = new Color(220, 60, 60, 110);
    private static final Color OBSTACLE_STROKE = new Color(180, 30, 30, 180);

    // List of obstacle data to render
    private final List<UavObstacleGrid.ObstacleRenderData> obstacles;

    /**
     * Constructor.
     * @param obstacles List of obstacle data to draw.
     */
    public ObstacleOverlayGraphic(List<UavObstacleGrid.ObstacleRenderData> obstacles) {
        this.obstacles = obstacles;
    }

    @Override
    public void draw(Graphics2D g2) {
        if (obstacles == null || obstacles.isEmpty()) {
            return;
        }

        Graphics2D g = (Graphics2D) g2.create();
        try {
            // Enable anti-aliasing for smoother lines
            g.setRenderingHint(RenderingHints.KEY_ANTIALIASING,
                               RenderingHints.VALUE_ANTIALIAS_ON);

            // Iterate through each obstacle and draw it based on its type
            for (UavObstacleGrid.ObstacleRenderData obs : obstacles) {
                if (obs.coords == null || obs.coords.isEmpty()) continue;

                // Draw different obstacle types
                switch (obs.type) {

                    case POINT: {
                        // Filled circle with radius = obs.radius
                        core.Coord c = obs.coords.get(0);
                        int cx = scale(c.getX());
                        int cy = scale(c.getY());
                        int r  = Math.max(1, scale(obs.radius));
                        g.setColor(OBSTACLE_FILL);
                        g.fillOval(cx - r, cy - r, r * 2, r * 2);
                        g.setColor(OBSTACLE_STROKE);
                        g.drawOval(cx - r, cy - r, r * 2, r * 2);
                        break;
                    }

                    case LINE: {
                        // Draw line segments with a thick fill and a thin stroke
                        // Thick stroke representing the half-width buffer
                        int halfW = Math.max(1, scale(obs.radius));
                        g.setStroke(new BasicStroke(halfW * 2,
                                BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
                        g.setColor(OBSTACLE_FILL);
                        for (int i = 0; i < obs.coords.size() - 1; i++) {
                            core.Coord a = obs.coords.get(i);
                            core.Coord b = obs.coords.get(i + 1);
                            g.drawLine(scale(a.getX()), scale(a.getY()),
                                       scale(b.getX()), scale(b.getY()));
                        }
                        // Draw outline on top
                        g.setStroke(new BasicStroke(1f,
                                BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
                        g.setColor(OBSTACLE_STROKE);
                        for (int i = 0; i < obs.coords.size() - 1; i++) {
                            core.Coord a = obs.coords.get(i);
                            core.Coord b = obs.coords.get(i + 1);
                            g.drawLine(scale(a.getX()), scale(a.getY()),
                                       scale(b.getX()), scale(b.getY()));
                        }
                        break;
                    }

                    case POLYGON: {
                        // Draw filled polygons with an outline
                        // Filled polygon
                        int n = obs.coords.size();
                        int[] xs = new int[n];
                        int[] ys = new int[n];
                        for (int i = 0; i < n; i++) {
                            xs[i] = scale(obs.coords.get(i).getX());
                            ys[i] = scale(obs.coords.get(i).getY());
                        }
                        g.setColor(OBSTACLE_FILL);
                        g.fillPolygon(xs, ys, n);
                        g.setColor(OBSTACLE_STROKE);
                        g.setStroke(new BasicStroke(1f));
                        g.drawPolygon(xs, ys, n);
                        break;
                    }
                }
            }
        } finally {
            g.dispose();
        }
    }
}
