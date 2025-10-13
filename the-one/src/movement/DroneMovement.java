package movement;

import core.Coord;
import core.Settings;

public class DroneMovement extends ExtendedMovementModel {
  /** Per node group setting for setting the location ({@value}) */
  public static final String LOCATION_S = "nodeLocation";
  private Coord initLoc;

  private FogVehicleSystem vehicleSystem;
  private int id;
  private static int nextID = 0;

  private LawnmowerMovement lawnmowerMM;
  private MapRouteMovement mapRouteMM;
  private SwitchableStationaryMovement stationaryMM;

  private static final int MAP_MODE = 1;
  private static final int SCAN_MODE = 2;
  private static final int STATIONARY_MODE = 3;
  private int state;
  private boolean scanning;

  public DroneMovement(Settings settings) {
    super(settings);
    this.id = nextID++;

    int fvs = settings.getInt(FogVehicleSystem.FOG_VEHICLE_SYSTEM_NR);
    vehicleSystem = FogVehicleSystem.getFogVehicleSystem(fvs);
    vehicleSystem.registerDrone(this);

    lawnmowerMM = new LawnmowerMovement(settings);
    mapRouteMM = new MapRouteMovement(settings);
    stationaryMM = new SwitchableStationaryMovement(settings);
    setCurrentMovementModel(stationaryMM);
  }

  public DroneMovement(DroneMovement proto) {
    super(proto);
    this.vehicleSystem = proto.vehicleSystem;
    this.id = nextID++;

    vehicleSystem.registerDrone(this);
    lawnmowerMM = proto.lawnmowerMM.replicate();
    mapRouteMM = proto.mapRouteMM.replicate();
    stationaryMM = proto.stationaryMM.replicate();
    setCurrentMovementModel(stationaryMM);
    state = MAP_MODE;
    scanning = false;
  }

  /**
   * Called by fog vehicle system.
   */
  public void enterFog() {
    state = MAP_MODE;
  }

  /**
   * Called by system when fog vehicle stops.
   * 
   * @param direction specifies if it goes left(-1) or right(1)
   */
  // TODO: use direction
  public void startScan(int direction) {
    state = SCAN_MODE;
  }

  public boolean newOrders() {
    switch (state) {
      case MAP_MODE:
        setCurrentMovementModel(mapRouteMM);
        state = STATIONARY_MODE;
        break;
      case SCAN_MODE:
        setCurrentMovementModel(lawnmowerMM);
        state = STATIONARY_MODE;
        scanning = true;
        break;
      case STATIONARY_MODE:
        // called if the lawnmowerMM is over
        if (scanning) {
          vehicleSystem.droneDone(this.id);
          scanning = false;
        }
        break;
    }

    return true;
  }

  @Override
  public Coord getInitialLocation() {
    initLoc = mapRouteMM.getInitialLocation().clone();
    stationaryMM.setLocation(initLoc);
    return initLoc;
  }

  @Override
  public DroneMovement replicate() {
    return new DroneMovement(this);
  }

  /**
   * Returns unique ID of the drone
   * 
   * @return unique ID of the drone
   */
  public int getID() {
    return id;
  }
}
