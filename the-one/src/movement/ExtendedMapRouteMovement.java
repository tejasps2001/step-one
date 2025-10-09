package movement;

public class ExtendedMapRouteMovement extends MapRouteMovement {
  Path nextPath;
  
  /**
   * Creates a new instance of ExtendedMapRouteMovement
   * @param settings
   */
  public ExtendedMapRouteMovement(Settings settings) {
    super(settings);
    nextPath = null;
  }

  /**
   * Creates a new instance from a prototype
   * @param proto
   */
  public ExtendedMapRouteMovement(ExtendedMapRouteMovement proto) {
    super(proto);
    nextPath = null;
  }

  @Override
  public ExtendedMapRouteMovement replicate() {
    return new ExtendedMapRouteMovement(this);
  }
  
  public Path getNextPath() {
    nextPath = super.getPath();
    return nextPath;
  }
  
  @Override
  public Path getPath() {
    Path returnPath = nextPath;
    nextPath = null;
    return returnPath;
  }
}
