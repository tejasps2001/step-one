/*
 * Optional UAV planning-grid visualization. Remove this package or disable
 * UAVWaypointMovement.showPlanningGrid to drop the feature.
 */
package gui.uavviz;

import gui.playfield.PlayFieldGraphic;
import movement.UAVWaypointMovement;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;

/** Draws A* cell boundaries and blocked cells from a {@link UAVWaypointMovement} snapshot. */
public class PlanningGridOverlayGraphic extends PlayFieldGraphic {

    private static final Color GRID_LINE   = new Color(90, 110, 160);
    private static final Color BLOCK_FILL  = new Color(220, 60, 60, 110);

    private final UAVWaypointMovement.PlanningGridSnapshot snap;

    public PlanningGridOverlayGraphic(UAVWaypointMovement.PlanningGridSnapshot snap) {
        this.snap = snap;
    }

    @Override
    public void draw(Graphics2D g2) {
        if (snap == null) {
            return;
        }

        double W = snap.gridW * snap.gridCellM;
        double H = snap.gridH * snap.gridCellM;

        Graphics2D g = (Graphics2D) g2.create();
        try {
            java.awt.Stroke oldStroke = g.getStroke();
            g.setStroke(new BasicStroke(1f));
            g.setColor(GRID_LINE);
            for (int c = 0; c <= snap.gridW; c++) {
                double x = c * snap.gridCellM;
                g.drawLine(scale(x), scale(0), scale(x), scale(H));
            }
            for (int r = 0; r <= snap.gridH; r++) {
                double y = r * snap.gridCellM;
                g.drawLine(scale(0), scale(y), scale(W), scale(y));
            }
            g.setStroke(oldStroke);

            g.setColor(BLOCK_FILL);
            for (int row = 0; row < snap.gridH; row++) {
                for (int col = 0; col < snap.gridW; col++) {
                    if (snap.blocked[row][col]) {
                        double x0 = col * snap.gridCellM;
                        double y0 = row * snap.gridCellM;
                        int px = scale(x0);
                        int py = scale(y0);
                        int sz = Math.max(1, scale(snap.gridCellM));
                        g.fillRect(px, py, sz, sz);
                    }
                }
            }
        } finally {
            g.dispose();
        }
    }
}
