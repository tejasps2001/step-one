package movement;

import core.Coord;
import core.Settings;
import java.util.*;

public class FogVehicleMovement extends ExtendedMovementModel {
  private LawnmoverMovement lawnmoverMM;
  private ExtendedLinearMovement extendedLinearMM;
  private SwitchableStationaryMovement stationaryMM;

  private static final int LINEAR_MODE = 1;
  private static final int STATIONARY_MODE = 2;

  private static boolean isReady;
  private String localMode;
  private int mode = LINEAR_MODE;
  private static boolean lawnmoverMMOnce;
  private List<Coord> pointList;

  public FogVehicleMovement(Settings settings) {
    super(settings);
    extendedLinearMM = new ExtendedLinearMovement(settings);
    stationaryMM = new SwitchableStationaryMovement(settings);
    lawnmoverMM = new LawnmoverMovement(settings);
  }

  public FogVehicleMovement(FogVehicleMovement proto) {
    super(proto);
    // list of stops
    pointList = new ArrayList<Coord>();
    pointList.add(new Coord(300, 300));
    pointList.add(new Coord(600, 300));
    pointList.add(new Coord(900, 300));
    pointList.add(new Coord(900, 600));
    
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
    
    Coord initLoc = pointList.remove(0);
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
          if (!pointList.isEmpty()) {
            extendedLinearMM.setNextPoint(pointList.remove(0));
            lawnmoverMMOnce = false;
          }
          break;
        case STATIONARY_MODE:
          setCurrentMovementModel(stationaryMM);
          break;
      }
    } else {
      if (mode == STATIONARY_MODE && lawnmoverMMOnce) {
        return false;
      }
      switch (mode) {
        case LINEAR_MODE:
          isReady = true;
          setCurrentMovementModel(extendedLinearMM);
          if (!pointList.isEmpty())
            extendedLinearMM.setNextPoint(pointList.remove(0));
          break;
        case STATIONARY_MODE:
          isReady = false;
          setCurrentMovementModel(lawnmoverMM);
          lawnmoverMM = lawnmoverMM.replicate();
          lawnmoverMMOnce = true;
          break;
      }
    }

    return true;
  }
}
