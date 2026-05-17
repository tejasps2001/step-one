package gui.uavviz;

import core.Settings;
import movement.UAVWaypointMovement;
import movement.UavObstacleGrid;

import java.awt.Graphics2D;

/**
 * Orchestrates rendering of UAV obstacles and the A* planning grid.
 * Separates obstacle drawing from grid line drawing.
 */
public final class UavPlanningGridRenderer {

    private UavPlanningGridRenderer() {
        // Utility class
    }

    /**
     * Renders obstacles and the grid if enabled.
     */
    public static void renderIfEnabled(Graphics2D g2) {
        // Draw obstacles (always visible if data exists)
        java.util.List<UavObstacleGrid.ObstacleRenderData> obstacles =
                UavObstacleGrid.getObstacleRenderData();
        if (!obstacles.isEmpty()) {
            new ObstacleOverlayGraphic(obstacles).draw(g2);
        }

        // Draw A* grid lines (if enabled in settings and toggle is ON)
        Settings cfg = new Settings(UAVWaypointMovement.SETTINGS_NS);
        if (!cfg.getBoolean(UAVWaypointMovement.SHOW_PLANNING_GRID_S, false)) {
            return;
        }
        UavObstacleGrid.PlanningGridSnapshot snap =
                UavObstacleGrid.getPlanningGridSnapshot();
        if (snap == null) {
            return;
        }
        new PlanningGridOverlayGraphic(snap).draw(g2);
    }
}
