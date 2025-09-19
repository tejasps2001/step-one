package movement;

import core.Coord;
import core.Settings;
import movement.MovementModel;
import movement.Path;

/**
 * Movement model where each path is generated based on offset and direction
 * (left, right, bottom, up) (work in progress)
 */
public class GranularMovement extends MovementModel {
  /** Name space of the settings (append to group name space) */
  public static final String GRANULAR_MOVEMENT_NS = "GranularMovement.";
  /**
   * Per node group setting for defining the start coordinates of the
   * line({@value})
   */
  public static final String START_LOCATION_S = "startLocation";

  /** Per node group setting for offset distance ({@value}) **/
  public static final String OFFSET_S = "offset";

  private Coord startLoc;
  private int offset;

  private Coord initLoc;
  private Coord lastLoc;
  private Path nextPath;

  /**
   * Creates a new movement model based on a Settings object's settings.
   * @param s The Settings object where the settings are read from
   */
  public GranularMovement(Settings s) {
    super(s);
    int coords[];

    coords = s.getCsvInts(GRANULAR_MOVEMENT_NS + START_LOCATION_S, 2);
    this.startLoc = new Coord(coords[0], coords[1]);
    this.offset = s.getCsvInts("Group." + OFFSET_S, 1)[0];
  }

  /**
   * Copy constructor.
   * @param gm The GranularMovement prototype
   */
  public GranularMovement(GranularMovement gm) {
    super(gm);
    this.initLoc = gm.startLoc;
    this.nextPath = new Path(generateSpeed());
    this.nextPath.addWaypoint(initLoc);
    this.lastLoc = gm.startLoc;
  }

  public generateNextPath(String direction) {
    this.nextPath = new Path(generateSpeed());
    // TODO add direction based offsets
    double x = this.lastLoc.getX();
    double y = this.lastLoc.getY() + offset;

    Coord nextPoint = new Coord(x, y);
    this.nextPath.addWaypoint(nextPoint);
    this.lastLoc = nextPoint;
  }

  /**
   * Returns the the location of the node
   * @return the the location of the node
   */
  @Override
  public Coord getInitialLocation() {
    return this.initLoc;
  }

  /**
   * Returns a single coordinate path (using the only possible coordinate)
   * @return a single coordinate path
   */
  @Override
  public Path getPath() {
    Path p = nextPath;
    this.nextPath = null;
    return p;
  }

  /**
   * Returns Double.MAX_VALUE (no paths available)
   */
  @Override
  public double nextPathAvailable() {
    if (nextPath == null) {
      return Double.MAX_VALUE; // no new paths available
    } else {
      return 0;
    }
  }

  @Override
  public GranularMovement replicate() {
    return new GranularMovement(this);
  }
}
