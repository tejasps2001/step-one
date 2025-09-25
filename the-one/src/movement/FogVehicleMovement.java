package movement;

import core.Settings;
import movement.map.MapRoute;

/**
 * This movement model is a controller to coordinate the fog vehicle 
 * and drone's movements. It instructs the fog vehicle execute Map route movement
 * whereas the drones on the fog vehicle are instructed to execute both Map route
 * movement and Lawnmover Movement
 */
public class FogVehicleMovement extends ExtendedMovementModel {
    private MapRouteMovement mapRouteMovement;
    private StationaryMovement stationaryMM;
    private LawnmoverMovement lawnmoverMM;

    /**
	 * Creates a new instance of FogVehicleMovement
	 * @param settings
	 */
    public FogVehicleMovement(Settings settings) {
        super(settings);
        lawnmoverMM = new LawnmoverMovement(settings);
    }

    /**
	 * Creates a new instance of FogVehicleMovement from a prototype
	 * @param proto
	 */
	public FogVehicleMovement(FogVehicleMovement proto) {
        super(proto);
        
    }

    public boolean newOrders() {
        // execute map route movement and get to the first point on both
        // drone and fog vehicle

        // once reached, execute lawnmower on drone and stationary movement on fog vehicle

        // wait for lawnmower movement to end
        // switch to map route movement
        return true;
    }
}
