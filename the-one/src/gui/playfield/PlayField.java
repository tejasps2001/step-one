/*
 * Copyright 2010 Aalto University, ComNet
 * Released under GPLv3. See LICENSE.txt for details.
 */
package gui.playfield;

import core.Coord;
import core.DTNHost;
import core.World;
import gui.DTNSimGUI;
import gui.uavviz.UavPlanningGridRenderer;
import movement.Path;
import movement.map.SimMap;

import javax.swing.*;
import java.awt.*;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.geom.AffineTransform;
import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;


/**
 * The canvas where node graphics and message visualizations are drawn.
 */
public class PlayField extends JPanel {

	public static final int PLAYFIELD_OFFSET = 10;

	// ── Core references ───────────────────────────────────────────────────────
	private World      w;
	private DTNSimGUI  gui;

	// ── Rendering state ───────────────────────────────────────────────────────
	private Color bgColor = Color.WHITE;

	private List<PlayFieldGraphic> overlayGraphics;
	private boolean                autoClearOverlay; // clear overlays on each new graphic
	private MapGraphic             mapGraphic;
	private boolean                showMapGraphic;
	private ScaleReferenceGraphic  refGraphic;
	private boolean                focusOnClick;

	private BufferedImage  underlayImage;
	private AffineTransform imageTransform;
	private AffineTransform curTransform;
	private double          underlayImgDx;
	private double          underlayImgDy;

	// ─────────────────────────────────────────────────────────────────────────
	// Constructor
	// ─────────────────────────────────────────────────────────────────────────

	/**
	 * Creates a playfield.
	 *
	 * @param w   The world that contains the actors to be drawn
	 * @param gui The parent GUI
	 */
	public PlayField(World w, DTNSimGUI gui) {
		this.w   = w;
		this.gui = gui;

		this.refGraphic = new ScaleReferenceGraphic();
		updateFieldSize();
		this.setBackground(bgColor);
		this.overlayGraphics = Collections.synchronizedList(
				new ArrayList<PlayFieldGraphic>());
		this.mapGraphic      = null;
		this.underlayImage   = null;
		this.imageTransform  = null;
		this.autoClearOverlay = true;

		this.addMouseListener(new MouseAdapter() {
			@Override
			public void mouseClicked(MouseEvent e) {
				if (focusOnClick) {
					focusClosestNode(e.getX(), e.getY());
				}
			}
		});
	}

	// ─────────────────────────────────────────────────────────────────────────
	// Public API
	// ─────────────────────────────────────────────────────────────────────────

	/**
	 * Schedules the play field to be redrawn.
	 */
	public void updateField() {
		this.repaint();
	}

	/**
	 * Sets an image to show under the host graphics.
	 *
	 * @param image    The image to set, or {@code null} to remove it
	 * @param dx       X offset of the image
	 * @param dy       Y offset of the image
	 * @param scale    Image scaling factor
	 * @param rotation Rotation angle of the image (radians)
	 */
	public void setUnderlayImage(BufferedImage image,
			double dx, double dy, double scale, double rotation) {
		if (image == null) {
			this.underlayImage  = null;
			this.imageTransform = null;
			this.curTransform   = null;
			return;
		}
		this.underlayImage  = image;
		this.imageTransform = AffineTransform.getRotateInstance(rotation);
		this.imageTransform.scale(scale, scale);
		this.curTransform   = new AffineTransform(imageTransform);
		this.underlayImgDx  = dx;
		this.underlayImgDy  = dy;

		curTransform.scale(PlayFieldGraphic.getScale(),
				PlayFieldGraphic.getScale());
		curTransform.translate(this.underlayImgDx, this.underlayImgDy);
	}

	/**
	 * Sets the zooming/scaling factor.
	 *
	 * @param scale The new scale
	 */
	public void setScale(double scale) {
		PlayFieldGraphic.setScale(scale);
		this.updateFieldSize();
		if (this.imageTransform != null) {
			this.curTransform = new AffineTransform(imageTransform);
			curTransform.scale(scale, scale);
			curTransform.translate(this.underlayImgDx, this.underlayImgDy);
		}
	}

	/**
	 * Sets the source for the map graphics and enables map display.
	 * Map is shown by default once set; call {@link #setShowMapGraphic(boolean)}
	 * to override.
	 *
	 * @param simMap The map to show
	 */
	public void setMap(SimMap simMap) {
		this.mapGraphic     = new MapGraphic(simMap);
		this.showMapGraphic = true;  // show map by default (original behaviour)
	}

	/**
	 * Enables/disables showing of map graphics.
	 *
	 * @param show {@code true} to show the map, {@code false} to hide it
	 */
	public void setShowMapGraphic(boolean show) {
		this.showMapGraphic = show;
	}

	/**
	 * Enables or disables automatic clearing of overlay graphics.
	 * When enabled, overlays are cleared every time a new graphic is added
	 * via {@link #addMessageTransfer}.
	 *
	 * @param clear {@code true} to enable auto-clear
	 */
	public void setAutoClearOverlay(boolean clear) {
		this.autoClearOverlay = clear;
	}

	/**
	 * Enables or disables automatic focus-on-click behaviour.
	 * When enabled, clicking the field focuses the nearest node.
	 *
	 * @param focus {@code true} to enable
	 */
	public void setFocusOnClick(boolean focus) {
		this.focusOnClick = focus;
	}

	/**
	 * Draws the play field. Called by the Swing framework, or directly when
	 * rendering to an off-screen context (e.g. screenshots).
	 *
	 * <p>Render order:
	 * <ol>
	 *   <li>Underlay image (if any)</li>
	 *   <li>Map graphic (if enabled)</li>
	 *   <li>UAV planning grid (if enabled — rendered below hosts so nodes
	 *       remain visible on top)</li>
	 *   <li>Host nodes</li>
	 *   <li>Overlay graphics (message transfers, paths, etc.)</li>
	 *   <li>Scale reference</li>
	 * </ol>
	 *
	 * @param g The graphics context
	 */
	@Override
	public void paint(Graphics g) {
		Graphics2D g2 = (Graphics2D) g;
		g2.setBackground(bgColor);

		g2.translate(PLAYFIELD_OFFSET, PLAYFIELD_OFFSET);

		// Clear previous frame
		g2.clearRect(-PLAYFIELD_OFFSET, -PLAYFIELD_OFFSET,
				this.getWidth()  + PLAYFIELD_OFFSET,
				this.getHeight() + PLAYFIELD_OFFSET);

		// 1. Underlay image
		if (underlayImage != null) {
			g2.drawImage(underlayImage, curTransform, null);
		}

		// 2. Map graphic
		if (mapGraphic != null && showMapGraphic) {
			mapGraphic.draw(g2);
		}

		// 3. UAV planning grid (drawn under hosts; lives outside the overlay
		//    list so it is not erased by clearOverlays())
		UavPlanningGridRenderer.renderIfEnabled(g2);

		// 4. Host nodes
		for (DTNHost h : w.getHosts()) {
			new NodeGraphic(h).draw(g2);
		}

		// 5. Overlay graphics — synchronized to prevent ConcurrentModificationException
		//    if graphics are added from another thread while painting
		synchronized (overlayGraphics) {
			for (int i = 0, n = overlayGraphics.size(); i < n; i++) {
				overlayGraphics.get(i).draw(g2);
			}
		}

		// TODO: Use some listener
		// draw road analysis segments
		// RoadMonitoringApp.drawSegments(g2);

		// 6. Scale reference
		this.refGraphic.draw(g2);
	}

	/**
	 * Removes all overlay graphics.
	 */
	public void clearOverlays() {
		this.overlayGraphics.clear();
	}

	/**
	 * Adds a message-transfer graphic to the overlay.
	 *
	 * @param from Source host
	 * @param to   Destination host
	 */
	public void addMessageTransfer(DTNHost from, DTNHost to) {
		autoClear();
		this.overlayGraphics.add(new MessageGraphic(from, to));
	}

	/**
	 * Adds a path to the overlay graphics (default colour).
	 *
	 * @param path Path to add
	 */
	public void addPath(Path path) {
		// autoClear();  // intentionally disabled for paths
		this.overlayGraphics.add(new PathGraphic(path));
		this.updateField();
	}

	/**
	 * Adds a path to the overlay graphics with a specified colour.
	 *
	 * @param path  Path to add
	 * @param color Colour to draw the path in
	 */
	public void addPath(Path path, Color color) {
		// autoClear();  // intentionally disabled for paths
		this.overlayGraphics.add(new PathGraphic(path, color));
		this.updateField();
	}

	/**
	 * Adds an arbitrary overlay graphic (e.g. a debug grid).
	 * Does not trigger auto-clear, so existing paths and message graphics
	 * are preserved.
	 *
	 * @param graphic The overlay to draw until {@link #clearOverlays()} is called;
	 *                ignored if {@code null}
	 */
	public void addOverlayGraphic(PlayFieldGraphic graphic) {
		if (graphic == null) {
			return;
		}
		this.overlayGraphics.add(graphic);
		this.updateField();
	}

	/**
	 * Returns the graphical position for a given world coordinate.
	 *
	 * @param loc World coordinate
	 * @return Corresponding graphics-space coordinate
	 * @see #getWorldPosition(Coord)
	 */
	public Coord getGraphicsPosition(Coord loc) {
		Coord c = loc.clone();
		c.setLocation(
				PlayFieldGraphic.scale(c.getX()) + PLAYFIELD_OFFSET,
				PlayFieldGraphic.scale(c.getY()) + PLAYFIELD_OFFSET);
		return c;
	}

	/**
	 * Returns the world coordinate for a given graphical position.
	 * Minor rounding inaccuracies may apply.
	 *
	 * @param loc Graphics-space coordinate
	 * @return Corresponding world coordinate
	 * @see #getGraphicsPosition(Coord)
	 */
	public Coord getWorldPosition(Coord loc) {
		Coord c = loc.clone();
		c.setLocation(
				PlayFieldGraphic.invScale(c.getX() - PLAYFIELD_OFFSET),
				PlayFieldGraphic.invScale(c.getY() - PLAYFIELD_OFFSET));
		return c;
	}

	// ─────────────────────────────────────────────────────────────────────────
	// Private helpers
	// ─────────────────────────────────────────────────────────────────────────

	/**
	 * Updates the playfield's preferred/minimum size to match the world
	 * dimensions at the current zoom level.
	 */
	private void updateFieldSize() {
		Dimension minSize = new Dimension(
				PlayFieldGraphic.scale(w.getSizeX()),
				PlayFieldGraphic.scale(w.getSizeY()));
		this.setMinimumSize(minSize);
		this.setPreferredSize(minSize);
		this.setSize(minSize);
	}

	/**
	 * Focuses the GUI on the host closest to the clicked screen position.
	 *
	 * @param x Screen X coordinate
	 * @param y Screen Y coordinate
	 */
	private void focusClosestNode(int x, int y) {
		DTNHost closest     = w.getHosts().get(0);
		double  closestDist = Double.MAX_VALUE;

		Coord clickLoc = getWorldPosition(new Coord(x, y));

		for (DTNHost h : w.getHosts()) {
			double dist = h.getLocation().distance(clickLoc);
			if (dist < closestDist) {
				closest     = h;
				closestDist = dist;
			}
		}

		gui.setFocus(closest);
	}

	/**
	 * Clears overlays if {@link #autoClearOverlay} is set.
	 */
	private void autoClear() {
		if (this.autoClearOverlay) {
			this.clearOverlays();
		}
	}
}
