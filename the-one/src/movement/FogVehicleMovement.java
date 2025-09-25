package movement;

import core.Coord;
import core.Settings;

public class FogVehicleMovement extends ExtendedMovementModel {
  private LawnmoverMovement lawnmoverMM;
  private ExtendedLinearMovement extendedLinearMM;
  private SwitchableStationaryMovement stationaryMM;

  private static final int LINEAR_MODE = 1;
  private static final int STATIONARY_MODE = 2;

  private static boolean isReady;
  private String localMode;
  private int mode;
  private boolean lawnmoverMMOnce;

  public FogVehicleMovement(Settings settings) {
    super(settings);
    extendedLinearMM = new ExtendedLinearMovement(settings);
    stationaryMM = new SwitchableStationaryMovement(settings);
    lawnmoverMM = new LawnmoverMovement(settings);
  }

  public FogVehicleMovement(FogVehicleMovement proto) {
    super(proto);
    extendedLinearMM = new ExtendedLinearMovement(proto.extendedLinearMM);
    stationaryMM = proto.stationaryMM.replicate();
    lawnmoverMM = proto.lawnmoverMM.replicate();
    setCurrentMovementModel(extendedLinearMM);
    mode = proto.mode;
    isReady = true;

  }

  @Override
  public Coord getInitialLocation() {
    String hostName = this.getHost().getName();
    if (hostName.startsWith("f")) {
      localMode = "fogVehicle";
    } else {
      localMode = "droneVehicle";
    }
    
    Coord initLoc = new Coord(300, 300);
    extendedLinearMM.setLocation(initLoc);
    return initLoc;
  }

  @Override
  public FogVehicleMovement replicate() {
    return new FogVehicleMovement (this);
  }

  public boolean newOrders() {
    mode = (mode == LINEAR_MODE) ? STATIONARY_MODE : LINEAR_MODE;
    if (localMode == "fogVehicle") {
      if (!isReady) return false;
      switch (mode) {
        case LINEAR_MODE:
          setCurrentMovementModel(extendedLinearMM);
          break;
        case STATIONARY_MODE:
          setCurrentMovementModel(stationaryMM);
          break;
      }
    } else {
      if (lawnmoverMMOnce) {
        mode = LINEAR_MODE;
      }
      switch (mode) {
        case LINEAR_MODE:
          isReady = true;
          setCurrentMovementModel(extendedLinearMM);
          break;
        case STATIONARY_MODE:
          isReady = false;
          setCurrentMovementModel(lawnmoverMM);
          lawnmoverMMOnce = true;
          break;
      }
    }

    return true;
  }
}
