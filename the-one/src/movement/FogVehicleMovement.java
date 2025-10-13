package movement;

import core.Coord;
import core.Settings;

public class FogVehicleMovement extends ExtendedMovementModel {
  /** Per node group setting for setting the location ({@value}) */
  public static final String LOCATION_S = "nodeLocation";
  private Coord initLoc;
  
  private FogVehicleSystem vehicleSystem;

  private SwitchableStationaryMovement stationaryMM;
  private MapRouteMovement mapRouteMM;
  private int fvs;

  private static final int STATIONARY_MODE = 1;
  private static final int MAP_MODE = 2;
  private int state;
  private boolean scanning;

  public FogVehicleMovement(Settings settings) {
    super(settings);
    
    fvs = settings.getInt(FogVehicleSystem.FOG_VEHICLE_SYSTEM_NR);
    vehicleSystem = FogVehicleSystem.getFogVehicleSystem(fvs);
    vehicleSystem.registerFogVehicle(this);
    
    mapRouteMM = new MapRouteMovement(settings);
    stationaryMM = new SwitchableStationaryMovement(settings);
    setCurrentMovementModel(stationaryMM);
  }

  public FogVehicleMovement(FogVehicleMovement proto) {
    super(proto);
    this.vehicleSystem = proto.vehicleSystem;
    
    mapRouteMM = proto.mapRouteMM.replicate();
    System.out.println("init Location for : " + proto.fvs + " : " + mapRouteMM.getInitialLocation());
    stationaryMM = proto.stationaryMM.replicate();
    setCurrentMovementModel(stationaryMM);
    
    state = STATIONARY_MODE;
    scanning = false;
  }

  @Override
  public boolean newOrders() {
    switch (state) {
      case MAP_MODE:
        System.out.println("stopped");
        setCurrentMovementModel(stationaryMM);
        state = STATIONARY_MODE;
        vehicleSystem.hasStopped();
        break;
      case STATIONARY_MODE:
        if (!scanning) {
          setCurrentMovementModel(mapRouteMM);
          state = MAP_MODE;
          System.out.println("Fog vehicle Moving");
          vehicleSystem.nowMoving();
        }
        break;
    }

    return true;
  }

  /**
   * Called by the fog vehicle system when all drones return
   */
  public void scanDone() {
    scanning = false;
  }

  @Override
  public Coord getInitialLocation() {
    initLoc = mapRouteMM.getInitialLocation().clone();
    stationaryMM.setLocation(initLoc);
    return initLoc;
  }
  
  @Override
  public FogVehicleMovement replicate() {
    return new FogVehicleMovement(this);
  }
}
