package movement;


import java.util.*;
import core.Coord;
import core.Settings;
import core.SimScenario;
import core.DTNHost;

/**
 * Movemenet Model which move nodes based on the decision
 * made using poi grid
 */

public class GridMovement extends MovementModel {
    public static final String GRID_MOVEMENT_NS = "GridMovement.";

    public static final String START_LOCATION_S = "startLocation";
    // per node group setting for defining the cordinate of the target
    public static final String END_LOCATION_S = "endLocation";
    private DTNHost host;
    // The Starting Location of the node
    private Coord startLoc;
    // The Ending Location of the node
    private Coord endLoc;
    //indexing of the DTNHost
    private int hostCounter;
    //to get set of DTNhost and to find our DTNHost
    private  List<DTNHost> self;

    private POIGrid poiGrid;

    private Path nextPath;

    /**
     * Creates a new movement model based on a Settings object's settings.
     * 
     * @param s The Settings object where the settings are read from
     */
    public GridMovement(Settings s) {
        super(s);
        int coords[];

        coords = s.getCsvInts(GRID_MOVEMENT_NS + START_LOCATION_S, 2);
        this.startLoc = new Coord(coords[0], coords[1]);
        coords = s.getCsvInts(GRID_MOVEMENT_NS + END_LOCATION_S, 2);
        this.endLoc = new Coord(coords[0], coords[1]);
    }

    /**
     * Copy constructor.
     * 
     * @param ilm The GridMovement prototype
     */
    public GridMovement(GridMovement ilm) {
        super(ilm);
        self = SimScenario.getInstance().getHosts();
        while (hostCounter < self.size()) {
            DTNHost host = self.get(hostCounter++);
            String hostName = host.getName();
            if (hostName.startsWith("drone")&& host.getMovement()==this) {
                this.host = host;
                break;
            }
        }
        this.poiGrid =new POIGrid();
        this.nextPath = new Path(generateSpeed());
        this.nextPath.addWaypoint(ilm.startLoc);
    }

    /**
     * Returns the the location of the node in the formation
     * 
     * @return the the location of the node in the formation
     */
    @Override
    public Coord getInitialLocation() {
        return this.startLoc;
    }

    /**
     * Returns a single coordinate path (using the only possible coordinate)
     * 
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
    public void update

    @Override
    public GridMovement replicate() {
        return new GridMovement(this);
    }

}