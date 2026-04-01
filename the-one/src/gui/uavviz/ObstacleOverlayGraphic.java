/*
 * Optional UAV obstacle visualization. Renders obstacle shapes from original
 * WKT coordinate data — completely independent of the planning-grid toggle.
 */
package gui.uavviz;

import gui.playfield.PlayFieldGraphic;
import movement.UAVWaypointMovement;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.util.List;

/**
 * Draws obstacle shapes (points, lines, polygons) from
 * {@link UAVWaypointMovement#getObstacleRenderData()} using original WKT
 * world coordinates.
 *
 * <p>This graphic is rendered by {@link UavPlanningGridRenderer} on every
 * repaint, regardless of whether the planning-grid toggle is ON or OFF.
 * It is separate from {@link PlanningGridOverlayGraphic}, which handles
 * grid lines and is gated by the toggle.
 */
public class ObstacleOverlayGraphic extends PlayFieldGraphic {

    private static final Color OBSTACLE_FILL   = new Color(220, 60, 60, 110);
    private static final Color OBSTACLE_STROKE = new Color(180, 30, 30, 180);

    private final List<UAVWaypointMovement.ObstacleRenderData> obstacles;

    public ObstacleOverlayGraphic(List<UAVWaypointMovement.ObstacleRenderData> obstacles) {
        this.obstacles = obstacles;
    }

    @Override
    public void draw(Graphics2D g2) {
        if (obstacles == null || obstacles.isEmpty()) {
            return;
        }

        Graphics2D g = (Graphics2D) g2.create();
        try {
            g.setRenderingHint(RenderingHints.KEY_ANTIALIASING,
                               RenderingHints.VALUE_ANTIALIAS_ON);

            for (UAVWaypointMovement.ObstacleRenderData obs : obstacles) {
                if (obs.coords == null || obs.coords.isEmpty()) continue;

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
