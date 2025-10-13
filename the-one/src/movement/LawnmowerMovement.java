package movement;

import core.Coord;
import core.Settings;

class LawnmowerMovement extends MovementModel implements SwitchableMovement {
  public static final String LOCATION_S = "nodeLocation";

  private Coord initLoc;
  private Coord endLoc;
  private Path nextPath;
  private double horizontalShift;
  private double verticalShift;
  private double droneRange;
  private double fogRange;
  private double horizontalShiftSum;
  private boolean turning;
  private boolean done;
  private boolean startMode;

  /**
   * Lawnmower Movement executes lawnmower sweeping pattern by a drone
   * around a fog vehicle.
   * Creates a new movement model based on a Settings object's settings.
   * 
   * @param s The Settings object where the settings are read from
   */
  public LawnmowerMovement(Settings settings) {
    super(settings);

    int coords[];
    coords = settings.getCsvInts(LOCATION_S);
    this.initLoc = new Coord(coords[0], coords[1]);

    Settings globalSettings = new Settings();
    droneRange = globalSettings.getDouble("btInterface.transmitRange");
    fogRange = globalSettings.getDouble("wlanInterface.transmitRange");
  }

  /**
   * Copy constructor.
   * 
   * @param lm The LawnmoverMovement prototype
   */
  public LawnmowerMovement(LawnmowerMovement lm) {
    super(lm);
    this.fogRange = lm.fogRange;
    this.droneRange = lm.droneRange;

    horizontalShift = -1 * droneRange * 2;
    verticalShift = -1 * fogRange;
    horizontalShiftSum = 0;
    turning = false;

    startMode = true;
    // this.initLoc = lm.initLoc.clone();
    // this.nextPath = new Path(generateSpeed());
    // this.nextPath.addWaypoint(this.initLoc);
    // this.endLoc = initLoc.clone();
    // endLoc.translate(0, verticalShift);
    //
    // this.nextPath.addWaypoint(endLoc.clone());
  }

  /**
   * Returns a single coordinate path (using the only possible coordinate)
   * 
   * @return a single coordinate path
   */
  @Override
  public Path getPath() {
    this.nextPath = new Path(0.5);
    this.endLoc = initLoc.clone();
    
    System.out.println("initLoc for lawnmower" + initLoc);
    nextPath.addWaypoint(initLoc);
    System.out.println("fogRange: " + fogRange);

    while (horizontalShiftSum <= fogRange) {
      System.out.println("horizontalShiftSum: " + horizontalShiftSum);
      if (turning == true) {
        endLoc.translate(horizontalShift, 0);
        horizontalShiftSum += Math.abs(horizontalShift);
        turning = !turning;
        nextPath.addWaypoint(endLoc.clone());
      } else {
        verticalShift *= -1;
        endLoc.translate(0, 2 * verticalShift);
        nextPath.addWaypoint(endLoc.clone());
        turning = !turning;
      }
    }

    if (horizontalShiftSum > fogRange) {
      nextPath.addWaypoint(initLoc.clone());
      done = true;
    }

    Path p = nextPath;
    if (done) {
      nextPath = null;
    }

    return p;
  }

  @Override
  public double nextPathAvailable() {
    if (nextPath == null && !startMode) {
      return Double.MAX_VALUE;
    } else {
      return 0;
    }
  }

  @Override
  public Coord getInitialLocation() {
    return this.initLoc;
  }

  @Override
  public LawnmowerMovement replicate() {
    return new LawnmowerMovement(this);
  }

  @Override
  public void setLocation(Coord lastWaypoint) {
    initLoc = lastWaypoint.clone();
  }

  @Override
  public Coord getLastLocation() {
    return initLoc.clone();
  }

  @Override
  public boolean isReady() {
    return done;
  }
}
