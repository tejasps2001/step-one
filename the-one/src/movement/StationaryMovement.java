/*
 * Copyright 2010 Aalto University, ComNet
 * Released under GPLv3. See LICENSE.txt for details.
 */
package movement;

import java.util.ArrayList;
import java.util.List;
import core.SettingsError;
import core.Coord;
import core.Settings;

/**
 * A dummy stationary "movement" model where nodes do not move.
 * Might be useful for simulations with only external connection events.
 */
public class StationaryMovement extends MovementModel {
	/** Per node group setting for setting the location ({@value}) */
	public static final String LOCATION_S = "nodeLocation";
	/** Per node group setting for setting multiple locations for hosts in a group ({@value}) */
	public static final String LOCATIONS_S = "nodeLocations";
	private Coord loc; /** The location of the nodes */
	
	// Support for assigning multiple locations to a group of stationary nodes
	private List<Coord> locations;
	private static int nextHostIndex_sm = 0;

	/**
	 * Creates a new movement model based on a Settings object's settings.
	 * @param s The Settings object where the settings are read from
	 */
	public StationaryMovement(Settings s) {
		super(s);
		if (s.contains(LOCATIONS_S)) {
			String raw = s.getRawSetting(LOCATIONS_S);
			this.locations = new ArrayList<>();
			raw = raw.replace("[", "").replace("]", "");
			String[] pairs = raw.split(";");
			for (String pair : pairs) {
				String[] values = pair.split(",");
				if (values.length != 2) {
					throw new SettingsError("Invalid coordinate pair: " + pair + " in setting " + LOCATIONS_S);
				}
				locations.add(new Coord(Double.parseDouble(values[0].trim()), Double.parseDouble(values[1].trim())));
			}
			nextHostIndex_sm = 0;
			this.loc = locations.get(0); // Default for prototype
		} else {
			double[] coords = s.getCsvDoubles(LOCATION_S, 2);
			this.loc = new Coord(coords[0],coords[1]);
			this.locations = null;
		}
	}

	/**
	 * Copy constructor.
	 * @param sm The StationaryMovement prototype
	 */
	public StationaryMovement(StationaryMovement sm) {
		super(sm);
		if (sm.locations != null && nextHostIndex_sm < sm.locations.size()) {
			this.loc = sm.locations.get(nextHostIndex_sm++);
		} else {
			this.loc = sm.loc;
		}
	}

	/**
	 * Returns the only location of this movement model
	 * @return the only location of this movement model
	 */
	@Override
	public Coord getInitialLocation() {
		return loc;
	}

	/**
	 * Returns a single coordinate path (using the only possible coordinate)
	 * @return a single coordinate path
	 */
	@Override
	public Path getPath() {
		Path p = new Path(0);
		p.addWaypoint(loc);
		return p;
	}

	@Override
	public double nextPathAvailable() {
		return Double.MAX_VALUE;	// no new paths available
	}

	@Override
	public StationaryMovement replicate() {
		return new StationaryMovement(this);
	}

}
