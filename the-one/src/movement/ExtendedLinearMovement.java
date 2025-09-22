package movement;

import core.Coord;
import core.Settings;
import java.util.*;

public class ExtendedLinearMovement extends MovementModel implements SwitchableMovement {
  private ArrayList<Coord> locations;

  private Path nextPath;
  private Coord initLoc;
  private Coord lastLoc;

  public ExtendedLinearMovement(Settings s) {
    super(s);

    this.locations = new ArrayList<>();
    this.locations.add(new Coord(150, 200));
    this.locations.add(new Coord(150, 150));
    this.locations.add(new Coord(150, 200));
  }

  // Copy constructor
  public ExtendedLinearMovement(ExtendedLinearMovement elm) {
    super(elm);
    this.locations = (ArrayList<Coord>) elm.locations.clone();

    this.initLoc = this.locations.get(0);
    this.nextPath = new Path(0.1);
    this.nextPath.addWaypoint(initLoc);
    this.nextPath.addWaypoint(this.locations.get(1));
    lastLoc = this.locations.get(1);


    // this.nextPath.addWaypoint(this.locations.get(2));
  }

  @Override
  public Path getPath() {
    Path p = nextPath;
    this.nextPath = null;
    return p;
  }

  @Override
  public Coord getInitialLocation() {
    return this.initLoc;
  }

  @Override
  public double nextPathAvailable() {
    if (nextPath == null) {
      return Double.MAX_VALUE;
    } else {
      return 0;
    }
  }

  public boolean setNextPoint(Coord nextLocation) {
    this.nextPath = new Path(0.1);
    this.nextPath.addWaypoint(this.locations.get(2));
    this.nextPath.addWaypoint(nextLocation);
    return true;
  }

  @Override
  public MovementModel replicate() {
    return new ExtendedLinearMovement(this);
  }

  /**
   * @see SwitchableMovement
   */
  public boolean isReady() {
    return true;
  }

  /**
   * @see SwitchableMovement
   */
  public void setLocation(Coord lastWaypoint) {
    initLoc = lastWaypoint.clone();
  }

  /**
   * @see SwitchableMovement
   */
  public Coord getLastLocation() {
    return lastLoc.clone();
  }
}
