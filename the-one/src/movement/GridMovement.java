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
    // indexing of the DTNHost
    private int hostCounter;
    // to get set of DTNhost and to find our DTNHost
    private List<DTNHost> self;
    private boolean hostReceived;
    private boolean printOnce;
    private POIGrid poiGrid;

    private Path nextPath;

    private static final int NE = 1;
    private static final int E = 2;
    private static final int SE = 3;
    private static final int S = 4;
    private static final int SW = 5;
    private static final int W = 6;
    private static final int NW = 7;
    private static final int N = 8;

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
        this.nextPath = new Path(generateSpeed());
        
        this.nextPath.addWaypoint(ilm.startLoc);
        this.startLoc=ilm.startLoc.clone();
        this.endLoc=ilm.endLoc.clone();
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
        if (hostReceived == false) {
            this.hostCounter = 0;
            self = SimScenario.getInstance().getHosts();
            while (hostCounter < self.size()) {
                DTNHost temp = self.get(hostCounter++);
                String hostName = temp.getName();
                if (hostName.startsWith("drone") && temp.getMovement() == this) {
                    this.host = temp;
                    hostReceived = true;
                    break;
                }
            }
        }

        if (this.host == null) {
            // Host not found for this movement model, can't generate a path.
            return null;
        }

        if (this.poiGrid == null) {
            this.poiGrid = new POIGrid();
        }
        poiGrid.updateGrid(this.host);
        this.nextPath = new Path(generateSpeed());
        Coord nextCoord = calculateNextCoord();
         System.out.print(nextCoord + " " + poiGrid.getCellAtCoord(nextCoord).getCost());
        if (poiGrid.getCellAtCoord(nextCoord).getCost() == -1) {
            int dir = nextDirection(host.getLocation(), nextCoord);
            nextCoord = nextGridCoord(host.getLocation(), dir);
            this.nextPath.addWaypoint(nextCoord);
        } else {
            this.nextPath.addWaypoint(nextCoord);
        }
        Path p = nextPath;
        return p;
    }

    public Coord calculateNextCoord() {
        Coord currLoc = host.getLocation();
        Coord destLoc = endLoc;
        double distance = currLoc.distance(destLoc);
        if (distance < 1) {
            return endLoc;
        } else {

            double x = currLoc.getX() + ((destLoc.getX() - currLoc.getX()) / distance);
            double y = currLoc.getY() + ((destLoc.getY() - currLoc.getY()) / distance);
            return new Coord(x, y);

        }
    }

    private static int nextDirection(Coord currCoord, Coord nextCoord) {
        if (nextCoord.getX() > currCoord.getX()) {
            if (nextCoord.getY() > currCoord.getY()) {
                return NE;
            } else if (nextCoord.getY() == currCoord.getY()) {
                return E;

            } else {
                return SE;
            }
        } else if (nextCoord.getX() < currCoord.getX()) {
            if (nextCoord.getY() < currCoord.getY()) {
                return SW;
            } else if (nextCoord.getY() == currCoord.getY()) {
                return W;
            } else {
                return NW;
            }
        } else {
            if (nextCoord.getY() > currCoord.getY()) {
                return N;
            } else {
                return S;
            }
        }

    }

    private static Coord nextGridCoord(Coord coord, int dir) {
        switch (dir) {
            case NE:
                return new Coord(coord.getX(), coord.getY() + 10);
            case E:
                return new Coord(coord.getX() + 10, coord.getY() + 10);
            case SE:
                return new Coord(coord.getX() + 10, coord.getY());
            case S:
                return new Coord(coord.getX() + 10, coord.getY() - 10);
            case SW:
                return new Coord(coord.getX(), coord.getY() - 10);
            case W:
                return new Coord(coord.getX() - 10, coord.getY() - 10);
            case NW:
                return new Coord(coord.getX() - 10, coord.getY());
            case N:
                return new Coord(coord.getX() - 10, coord.getY() + 10);
            default:
                return coord;
        }
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
    public GridMovement replicate() {
        return new GridMovement(this);
    }

}