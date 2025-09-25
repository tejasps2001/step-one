package movement;

import java.util.List;
import java.util.ArrayList;
import core.Coord;
import core.DTNHost;
import core.Settings;
import core.SimScenario;

/**
 * This movement model makes the host execute the lawnmover pattern
 */
public class LawnmoverMovement extends MovementModel implements SwitchableMovement {
    /** Name space of the settings (append to group name space) */
    public static final String LAWNMOVER_MOVEMENT_NS = "LawnmoverMovement.";
    /**
     * Per node group setting for defining the start coordinates of the
     * line({@value})
     */
    public static final String START_LOCATION_S = "startLocation";

    public static final String DRONE_GROUP_ID = "Group1.";

    private Coord startLoc;
    private Coord initLoc;
    private Coord endLoc;
    private Path nextPath;
    private boolean turning;
    private double horizontalShift;
    private static double verticalShift;
    private double horizontalShiftSum;
    private static double fogRange;
    private static double droneRange;
    private boolean ready;
    private int direction;

    /**
     * Lawnmower Movement executes lawnmower sweeping pattern by a drone
     * around a fog vehicle.
     * Creates a new movement model based on a Settings object's settings.
     * 
     * @param s The Settings object where the settings are read from
     */
    public LawnmoverMovement(Settings settings) {
        super(settings);
        int coords[];
        coords = settings.getCsvInts(LAWNMOVER_MOVEMENT_NS + START_LOCATION_S, 2);
        this.startLoc = new Coord(coords[0], coords[1]);
        Settings globalSettings = new Settings();
        droneRange = globalSettings.getDouble("btInterface.transmitRange");
        fogRange = globalSettings.getDouble("wlanInterface.transmitRange");
        this.horizontalShiftSum = 0;
    }

    /**
     * Copy constructor.
     * 
     * @param lm The LawnmoverMovement prototype
     */
    public LawnmoverMovement(LawnmoverMovement lm) {
        super(lm);
        direction = -1;
        if(direction == -1) {
            horizontalShift = -1 * droneRange * 2;
            verticalShift = -1 * fogRange;
        }
        else {
            horizontalShift = droneRange * 2;
            verticalShift = fogRange;
        }
        this.horizontalShiftSum = lm.horizontalShiftSum;
        this.turning = true;

        // first motion after starting the simulation
        this.initLoc = lm.startLoc;
        this.nextPath = new Path(generateSpeed());
        this.nextPath.addWaypoint(this.initLoc);
        this.endLoc = initLoc.clone();
        endLoc.translate(0, verticalShift);
        this.nextPath.addWaypoint(endLoc.clone());
    }

    /**
     * Returns a single coordinate path (using the only possible coordinate)
     * 
     * @return a single coordinate path
     */
    @Override
    public Path getPath() {
        newOrders();
        // Pass a clone of the Coord object since the original Coord would
        // be changed by getPath() faster than it is applied in the simulation
        this.nextPath = new Path(0.5);
        this.nextPath.addWaypoint(endLoc.clone());
        System.out.println(this + " " + this.horizontalShiftSum + " " + fogRange + " " + (this.horizontalShiftSum <= fogRange));
        while (this.horizontalShiftSum <= fogRange) {
            if (turning == true) {
                endLoc.translate(horizontalShift, 0);
                
                this.horizontalShiftSum += Math.abs(horizontalShift);
                turning = !turning;
                this.nextPath.addWaypoint(endLoc.clone());
                break;
            } else {
                verticalShift *= -1;
                endLoc.translate(0, 2 * verticalShift);
                this.nextPath.addWaypoint(endLoc.clone());
                turning = !turning;
                break;
            }
        }
        if (this.horizontalShiftSum > fogRange) {
            this.nextPath.addWaypoint(new Coord(300, 300));
            ready = true;
        }
        Path p = nextPath;
        if (ready)
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

    public boolean newOrders() {
        List<DTNHost> self = SimScenario.getInstance().getHosts();
        // Get a list of all drone hosts
        List<DTNHost> droneList = new ArrayList<DTNHost>();
        for (DTNHost i : self) {
            if (i.getName().startsWith("d")) {
                droneList.add(i);
            }
        }

        for (DTNHost drone : droneList) {
            // System.out.println(drone + " linked to " + drone.getMovement());
        }

        // this.nextPath = new Path(generateSpeed());
        // this.nextPath.addWaypoint(new Coord(150,150));
        // this.nextPath.addWaypoint(endLoc.clone());

        // if(turning) {
        // endLoc.translate(-1* horizontalShift , 0 );
        // this.nextPath.addWaypoint(endLoc.clone());
        // }
        // else{
        // endLoc.translate(verticalShift, 0);
        // verticalShift *= -1;
        // this.nextPath.addWaypoint(endLoc.clone());
        // }

        // turning = !turning;
        return true;
    }

    @Override
    public Coord getInitialLocation() {
        return this.initLoc;
    }

    @Override
    public MovementModel replicate() {
        return new LawnmoverMovement(this);
    }

    /**
     * Tell the movement model what its current location is
     * 
     * @param lastWaypoint
     */
    public void setLocation(Coord lastWaypoint) {
        this.nextPath.addWaypoint(lastWaypoint);
    }

    /**
     * Get the last location the getPath() of this movement model has returned
     * 
     * @return the last location
     */
    public Coord getLastLocation() {
        return endLoc.clone();
    }

    /**
     * Checks if the movement model is finished doing its task and it's time to
     * switch to the next movement model. The method should be called between
     * getPath() calls.
     * 
     * @return true if ready
     */
    public boolean isReady() {
        return ready;
    }

    /**
     * Set direction of the lawnmower movement with respect to the fog vehicle
     */
    public void setDirection(String direction) {
        if(direction == "left")
            this.direction = -1;
        else
            this.direction = 1;
    }
}
