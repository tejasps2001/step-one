package gui.uavviz;

import core.Settings;
import movement.UAVWaypointMovement;

import java.awt.Graphics2D;

/**
 * Draws the planning grid and obstacles inside {@link gui.playfield.PlayField#paint}
 * so they are not affected by GUI update throttling or
 * {@link gui.playfield.PlayField#clearOverlays}.
 *
 * <p><b>Rendering is split into two independent passes:</b>
 * <ol>
 *   <li><b>Obstacle pass</b> — always runs when obstacles exist, regardless of
 *       whether the grid toggle is on or off.  Uses
 *       {@link UAVWaypointMovement#getObstacleRenderData()}, which is never
 *       gated by {@code gridRenderingEnabled}.</li>
 *   <li><b>Grid pass</b> — runs only when {@code showPlanningGrid=true} in the
 *       settings AND the runtime toggle is ON.  Uses
 *       {@link UAVWaypointMovement#getPlanningGridSnapshot()}, which returns
 *       {@code null} when the toggle is OFF.</li>
 * </ol>
 *
 * <p>This ensures that turning the grid OFF hides grid lines only — obstacles
 * remain visible at all times.
 */
public final class UavPlanningGridRenderer {

    private UavPlanningGridRenderer() {
    }

    /**
     * Called from {@link gui.playfield.PlayField#paint} on every repaint.
     *
     * <p>Two independent rendering passes:
     * <ul>
     *   <li>Obstacles — always rendered when obstacle data is available.</li>
     *   <li>Grid lines — rendered only when showPlanningGrid=true AND the
     *       runtime grid toggle is enabled.</li>
     * </ul>
     */
    public static void renderIfEnabled(Graphics2D g2) {
        // ── Pass 1: Obstacles (always rendered, independent of grid toggle) ──
        java.util.List<UAVWaypointMovement.ObstacleRenderData> obstacles =
                UAVWaypointMovement.getObstacleRenderData();
        if (!obstacles.isEmpty()) {
            new ObstacleOverlayGraphic(obstacles).draw(g2);
        }

        // ── Pass 2: Grid lines (only when showPlanningGrid=true AND toggle ON) ──
        Settings cfg = new Settings(UAVWaypointMovement.SETTINGS_NS);
        if (!cfg.getBoolean(UAVWaypointMovement.SHOW_PLANNING_GRID_S, false)) {
            return;
        }
        UAVWaypointMovement.PlanningGridSnapshot snap =
                UAVWaypointMovement.getPlanningGridSnapshot();
        if (snap == null) {
            // null means grid toggle is OFF — skip grid lines, obstacles already drawn above
            return;
        }
        new PlanningGridOverlayGraphic(snap).draw(g2);
    }
}
