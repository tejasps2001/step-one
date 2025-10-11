package movement;

import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.ArrayList;
import java.util.Arrays;

import core.Coord;

/**
 * This class controls a fog vehicle and some number of drones it
 * carries.
 */

public class FogVehicleSystem {
  public static final String FOG_VEHICLE_SYSTEM_NR = "fogVehicleSystemNr";
  private static HashMap<Integer, FogVehicleSystem> systems;
  public FogVehicleMovement fogVehicle;
  public HashMap<Integer, DroneMovement> drones;

  private HashSet<Integer> dronesScanning;

  /**
   * Creates a new instance of FogVehicleSystem without any fog
   * vehicle and drones.
   * @param systemID unique ID of the system
   */
  private FogVehicleSystem(int systemID) {
    drones = new HashMap<Integer, DroneMovement>();
  }

  /**
   * Returns a reference to FogVehicleSystem with the provided ID.
   * @param systemID unique ID of the system
   */
  public static FogVehicleSystem getFogVehicleSystem(int systemID) {
    Integer id = new Integer(systemID);

    if (systems.containsKey(id)) {
      return systems.get(id);
    } else {
      FogVehicleSystem fvs = new FogVehicleSystem(systemID);
      systems.put(id, fvs);
      return fvs;
    }
  }

  // hasStopped -> droneDone -> scanDone -> nowMoving(path)
  
  /**
   * Called by fog vehicle every time it stops.
   * Tells the drones to start scanning the area.
   */
  public void hasStopped() {
    // assuming drones are either one or two
    List<Integer> dronesDirections = Arrays.asList(1, -1);
    dronesScanning = new HashSet<>();
    int count = 0;
    for (DroneMovement drone : drones.values()) {
      drone.startScan(dronesDirections.get(count));
      dronesScanning.add(drone.getID());
      count++;
    }
  }

  /**
   * Called by a drone when it's done scanning.
   * System removes the drone from scanning set.
   * @param droneID id of the drone that is done
   */
  public void droneDone(int droneID) {
    dronesScanning.remove(droneID);
    if (dronesScanning.isEmpty()) {
      fogVehicle.scanDone();
    }
  }

  /**
   * Called by the fog vehicle when it is going to move to the
   * next point.
   */
  public void nowMoving() {
    for (DroneMovement drone: drones.values()) {
      drone.enterFog();
    }
  }

	/**
	 * Registers fog vehicle to be part of fog vehicle system
	 * @param fogVehicle The fog vehicle to register
	 */
	public void registerFogVehicle(FogVehicleMovement fogVehicle) {
		this.fogVehicle = fogVehicle;
	}

	/**
	 * Registers a drone to be part of a fog vehicle system
	 * @param  The drone to register
	 */
	public void registerDrone(DroneMovement drone) {
		drones.put(drone.getID(), drone);
	}
}
