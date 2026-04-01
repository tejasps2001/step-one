/*
 * Copyright 2010 Aalto University, ComNet
 * Released under GPLv3. See LICENSE.txt for details.
 */
package gui;

import gui.playfield.PlayField;

import java.awt.FlowLayout;
import java.awt.Graphics2D;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import javax.swing.event.*;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.image.BufferedImage;
import java.io.File;

import javax.imageio.ImageIO;
import javax.swing.ImageIcon;
import javax.swing.JButton;
import javax.swing.JComboBox;
import javax.swing.JFileChooser;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JSpinner;
import javax.swing.JTextField;
import javax.swing.JToggleButton;
import javax.swing.SpinnerNumberModel;

import core.Coord;
import core.SimClock;
import movement.UAVWaypointMovement;

/**
 * GUI's control panel
 */
public class GUIControls extends JPanel implements ActionListener, ChangeListener {

	// ── Icon paths ────────────────────────────────────────────────────────────
	private static final String PATH_GRAPHICS  = "buttonGraphics/";
	private static final String ICON_PAUSE     = "Pause16.gif";
	private static final String ICON_PLAY      = "Play16.gif";
	private static final String ICON_ZOOM      = "Zoom24.gif";
	private static final String ICON_STEP      = "StepForward16.gif";
	private static final String ICON_FFW       = "FastForward16.gif";

	// ── Tooltip / label strings ───────────────────────────────────────────────
	private static final String TEXT_PAUSE        = "pause simulation";
	private static final String TEXT_PLAY         = "play simulation";
	private static final String TEXT_PLAY_UNTIL   = "play simulation until sim time...";
	private static final String TEXT_STEP         = "step forward one interval";
	private static final String TEXT_FFW          = "enable/disable fast forward";
	private static final String TEXT_UP_CHOOSER   = "GUI update:";
	private static final String TEXT_SCREEN_SHOT  = "screen shot";
	private static final String TEXT_SIMTIME      = "Simulation time - " +
		"click to force update, right click to change format";
	private static final String TEXT_SEPS         = "simulated seconds per second";
	private static final String TEXT_GRID_TOGGLE  = "toggle UAV planning grid overlay";

	// ── Misc constants ────────────────────────────────────────────────────────
	/** "Simulated events per second" averaging window (milliseconds) */
	private static final int    EPS_AVG_TIME        = 2000;
	private static final String SCREENSHOT_FILE_TYPE = "png";
	private static final String SCREENSHOT_FILE      = "screenshot";

	// ── UI components ─────────────────────────────────────────────────────────
	private JTextField        simTimeField;
	private JLabel            sepsField;       // simulated events per second
	private JButton           playButton;
	private JButton           playUntilButton;
	private JButton           stepButton;
	private JButton           ffwButton;
	private JButton           screenShotButton;
	private JComboBox<String> guiUpdateChooser;
	private JToggleButton     gridToggleButton; // UAV grid overlay toggle (optional)

	// ── State ─────────────────────────────────────────────────────────────────
	private boolean paused;
	private boolean step;
	private boolean isFfw;
	private int     oldSpeedIndex; // speed selected before FFW was engaged

	/**
	 * GUI update speeds.
	 * Negative values → how many 1/10 seconds to wait between updates.
	 * Positive values → show every Nth simulation update.
	 */
	public static final String[] UP_SPEEDS = {
		"-10", "-1", "0.1", "1", "10", "100", "1000", "10000", "100000"
	};

	/** Smallest allowed zoom level */
	public static final double ZOOM_MIN = 0.001;
	/** Largest allowed zoom level */
	public static final double ZOOM_MAX = 10;

	/** Index of the initial update-speed selection */
	public static final int INITIAL_SPEED_SELECTION = 3;
	/** Index of the fast-forward update-speed selection */
	public static final int FFW_SPEED_INDEX = 7;

	private double             guiUpdateInterval;
	private javax.swing.JSpinner zoomSelector;

	private PlayField  pf;
	private DTNSimGUI  gui;

	private long   lastUpdate;
	private double lastSimTime;
	private double playUntilTime;

	private boolean useHourDisplay = false;

	// ─────────────────────────────────────────────────────────────────────────
	// Constructor
	// ─────────────────────────────────────────────────────────────────────────

	public GUIControls(DTNSimGUI gui, PlayField pf) {
		/* TODO: read values for paused, isFfw etc from a settings file */
		this.pf            = pf;
		this.gui           = gui;
		this.lastUpdate    = System.currentTimeMillis();
		this.lastSimTime   = 0;
		this.paused        = true;
		this.isFfw         = false;
		this.playUntilTime = Double.MAX_VALUE;
		initPanel();
	}

	// ─────────────────────────────────────────────────────────────────────────
	// Panel initialisation
	// ─────────────────────────────────────────────────────────────────────────

	/**
	 * Creates and lays out all panel components.
	 */
	private void initPanel() {
		this.setLayout(new FlowLayout());

		// Simulation-time display
		this.simTimeField = new JTextField("0.0");
		this.simTimeField.setColumns(6);
		this.simTimeField.setEditable(false);
		this.simTimeField.setToolTipText(TEXT_SIMTIME);
		this.simTimeField.addMouseListener(new MouseAdapter() {
			public void mouseClicked(MouseEvent e) {
				if (e.getButton() == MouseEvent.BUTTON3) {
					useHourDisplay = !useHourDisplay;
				}
				setSimTime(SimClock.getTime());
			}
		});

		this.sepsField = new JLabel("0.00");
		this.sepsField.setToolTipText(TEXT_SEPS);

		this.screenShotButton = new JButton(TEXT_SCREEN_SHOT);
		this.guiUpdateChooser = new JComboBox<>(UP_SPEEDS);
		this.zoomSelector     = new JSpinner(
			new SpinnerNumberModel(1.0, ZOOM_MIN, ZOOM_MAX, 0.001));

		// Add sim-time widgets
		this.add(simTimeField);
		this.add(sepsField);

		// Playback control buttons
		playButton      = addButton(paused ? ICON_PLAY : ICON_PAUSE,
		                            paused ? TEXT_PLAY  : TEXT_PAUSE);
		stepButton      = addButton(ICON_STEP, TEXT_STEP);
		ffwButton       = addButton(ICON_FFW,  TEXT_FFW);
		playUntilButton = addButton(ICON_PLAY, TEXT_PLAY_UNTIL);
		playUntilButton.setText("...");

		// ── UAV Grid Toggle (only shown when UAVWaypointMovement is on classpath) ──
		if (isUAVMovementAvailable()) {
			gridToggleButton = new JToggleButton("Grid");
			gridToggleButton.setToolTipText(TEXT_GRID_TOGGLE);
			try {
				gridToggleButton.setSelected(UAVWaypointMovement.isGridRenderingEnabled());
			} catch (Exception e) {
				gridToggleButton.setSelected(true); // safe fallback
			}
			gridToggleButton.addActionListener(this);
			this.add(gridToggleButton);
			updateGridButtonText();
		}
		// ─────────────────────────────────────────────────────────────────────

		// Update-speed chooser
		this.add(new JLabel(TEXT_UP_CHOOSER));
		this.add(this.guiUpdateChooser);
		this.guiUpdateChooser.setSelectedIndex(INITIAL_SPEED_SELECTION);
		this.updateUpdateInterval();

		// Zoom controls
		this.add(new JLabel(createImageIcon(ICON_ZOOM)));
		this.updateZoomScale(false);
		this.add(this.zoomSelector);

		// Screenshot button
		this.add(this.screenShotButton);

		// Register remaining listeners
		guiUpdateChooser.addActionListener(this);
		zoomSelector.addChangeListener(this);
		this.screenShotButton.addActionListener(this);
	}

	// ─────────────────────────────────────────────────────────────────────────
	// Helper: icon factory
	// ─────────────────────────────────────────────────────────────────────────

	private ImageIcon createImageIcon(String path) {
		java.net.URL imgURL = getClass().getResource(PATH_GRAPHICS + path);
		if (imgURL != null) return new ImageIcon(imgURL);
		return null;
	}

	private JButton addButton(String iconPath, String tooltip) {
		ImageIcon icon   = createImageIcon(iconPath);
		JButton   button = new JButton(icon);
		if (icon == null) {
			button.setText(tooltip);
		}
		button.setToolTipText(tooltip);
		button.addActionListener(this);
		this.add(button);
		return button;
	}

	// ─────────────────────────────────────────────────────────────────────────
	// Public API
	// ─────────────────────────────────────────────────────────────────────────

	/**
	 * Sets the simulation time displayed in the control panel.
	 * Also recalculates the simulated-seconds-per-second rate periodically.
	 *
	 * @param time The current simulation time
	 */
	public void setSimTime(double time) {
		long timeSinceUpdate = System.currentTimeMillis() - this.lastUpdate;

		if (timeSinceUpdate > EPS_AVG_TIME) {
			double val = ((time - this.lastSimTime) * 1000) / timeSinceUpdate;
			this.sepsField.setText(String.format("%.2f 1/s", val));
			this.lastSimTime = time;
			this.lastUpdate  = System.currentTimeMillis();
		}

		if (this.useHourDisplay) {
			int    hours = (int)(time / 3600);
			int    mins  = (int)((time - hours * 3600) / 60);
			double secs  = time % 60;
			this.simTimeField.setText(
				String.format("%02d:%02d:%02.1f", hours, mins, secs));
		} else {
			this.simTimeField.setText(String.format("%.1f", time));
		}
	}

	/**
	 * Puts the simulation into paused or playing state and updates the UI.
	 *
	 * @param paused {@code true} to pause, {@code false} to play
	 */
	public void setPaused(boolean paused) {
		if (!paused) {
			this.playButton.setIcon(createImageIcon(ICON_PAUSE));
			this.playButton.setToolTipText(TEXT_PAUSE);
			this.paused = false;
			if (SimClock.getTime() >= this.playUntilTime) {
				this.playUntilTime = Double.MAX_VALUE; // reset play-until
			}
		} else {
			this.playButton.setIcon(createImageIcon(ICON_PLAY));
			this.playButton.setToolTipText(TEXT_PLAY);
			this.paused = true;
			this.setSimTime(SimClock.getTime());
			this.pf.updateField();
		}
	}

	/**
	 * Returns whether the simulation is currently paused.
	 * Handles single-step logic: returns {@code false} once after a step request.
	 *
	 * @return {@code true} if paused
	 */
	public boolean isPaused() {
		if (step) { // consume one step tick
			step = false;
			return false;
		}
		if (SimClock.getTime() >= this.playUntilTime) {
			this.setPaused(true);
		}
		return this.paused;
	}

	/**
	 * @return {@code true} if fast-forward mode is active
	 */
	public boolean isFfw() {
		return this.isFfw;
	}

	/**
	 * @return The currently selected GUI update interval (seconds)
	 */
	public double getUpdateInterval() {
		return this.guiUpdateInterval;
	}

	/**
	 * Adjusts the zoom level by {@code delta} steps.
	 *
	 * @param delta Positive to zoom in, negative to zoom out
	 */
	public void changeZoom(int delta) {
		SpinnerNumberModel model =
			(SpinnerNumberModel) this.zoomSelector.getModel();
		double curZoom  = model.getNumber().doubleValue();
		Number newValue = curZoom + model.getStepSize().doubleValue()
		                           * delta * curZoom * 100;

		if (newValue.doubleValue() < ZOOM_MIN) newValue = ZOOM_MIN;
		else if (newValue.doubleValue() > ZOOM_MAX) newValue = ZOOM_MAX;

		model.setValue(newValue);
		this.updateZoomScale(true);
	}

	// ─────────────────────────────────────────────────────────────────────────
	// Event handlers
	// ─────────────────────────────────────────────────────────────────────────

	@Override
	public void actionPerformed(ActionEvent e) {
		Object src = e.getSource();

		if (src == this.playButton) {
			setPaused(!this.paused);

		} else if (src == this.stepButton) {
			setPaused(true);
			this.step = true;

		} else if (src == this.ffwButton) {
			switchFfw();

		} else if (src == this.playUntilButton) {
			setPlayUntil();

		} else if (src == this.guiUpdateChooser) {
			updateUpdateInterval();

		} else if (src == this.screenShotButton) {
			takeScreenShot();

		} else if (gridToggleButton != null && src == this.gridToggleButton) {
			// UAV grid toggle
			try {
				boolean enabled = gridToggleButton.isSelected();
				UAVWaypointMovement.setGridRenderingEnabled(enabled);
				updateGridButtonText();
				this.pf.updateField(); // repaint immediately
			} catch (Exception ex) {
				System.err.println("Warning: Could not toggle UAV grid: "
					+ ex.getMessage());
			}
		}
	}

	@Override
	public void stateChanged(ChangeEvent e) {
		updateZoomScale(true);
	}

	// ─────────────────────────────────────────────────────────────────────────
	// Private helpers
	// ─────────────────────────────────────────────────────────────────────────

	private void switchFfw() {
		if (isFfw) {
			this.isFfw = false;
			this.ffwButton.setIcon(createImageIcon(ICON_FFW));
			this.guiUpdateChooser.setSelectedIndex(oldSpeedIndex);
			this.ffwButton.setSelected(false);
		} else {
			this.oldSpeedIndex = this.guiUpdateChooser.getSelectedIndex();
			this.guiUpdateChooser.setSelectedIndex(FFW_SPEED_INDEX);
			this.isFfw = true;
			this.ffwButton.setIcon(createImageIcon(ICON_PLAY));
		}
	}

	private void setPlayUntil() {
		setPaused(true);
		String value = JOptionPane.showInputDialog(TEXT_PLAY_UNTIL);
		if (value == null) return;
		try {
			this.playUntilTime = Double.parseDouble(value);
			setPaused(false);
		} catch (NumberFormatException e) {
			JOptionPane.showMessageDialog(gui.getParentFrame(),
				"Invalid number '" + value + "'",
				"error", JOptionPane.ERROR_MESSAGE);
		}
	}

	private void updateUpdateInterval() {
		String selString = (String) this.guiUpdateChooser.getSelectedItem();
		this.guiUpdateInterval = Double.parseDouble(selString);
	}

	/**
	 * Syncs the playfield zoom to the value currently shown in the spinner.
	 *
	 * @param centerView If {@code true}, the viewport centre is preserved
	 */
	private void updateZoomScale(boolean centerView) {
		double scale = ((SpinnerNumberModel) zoomSelector.getModel())
			.getNumber().doubleValue();

		if (centerView) {
			Coord center = gui.getCenterViewCoord();
			this.pf.setScale(scale);
			gui.centerViewAt(center);
		} else {
			this.pf.setScale(scale);
		}
	}

	private void takeScreenShot() {
		try {
			JFileChooser fc = new JFileChooser();
			fc.setSelectedFile(new File(SCREENSHOT_FILE + "." + SCREENSHOT_FILE_TYPE));
			if (fc.showSaveDialog(this) == JFileChooser.APPROVE_OPTION) {
				File          file = fc.getSelectedFile();
				BufferedImage img  = new BufferedImage(this.pf.getWidth(),
					this.pf.getHeight(), BufferedImage.TYPE_INT_RGB);
				Graphics2D    g2   = img.createGraphics();
				this.pf.paint(g2);
				ImageIO.write(img, SCREENSHOT_FILE_TYPE, file);
			}
		} catch (Exception e) {
			JOptionPane.showMessageDialog(gui.getParentFrame(),
				"screenshot failed (problems with output file?)",
				"Exception", JOptionPane.ERROR_MESSAGE);
		}
	}

	/**
	 * Updates the grid toggle button label to reflect its current state.
	 */
	private void updateGridButtonText() {
		if (gridToggleButton != null) {
			gridToggleButton.setText(
				gridToggleButton.isSelected() ? "Grid ON" : "Grid OFF");
		}
	}

	/**
	 * Checks (via reflection) whether {@code movement.UAVWaypointMovement}
	 * is present on the classpath, so the grid button is only shown in
	 * UAV-enabled builds.
	 *
	 * @return {@code true} if the class is available
	 */
	private boolean isUAVMovementAvailable() {
		try {
			Class.forName("movement.UAVWaypointMovement");
			return true;
		} catch (ClassNotFoundException e) {
			return false;
		}
	}
}
