package movement;

import java.util.List;
import core.Connection;
import core.Coord;
import core.DTNHost;
import core.Settings;

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
    private boolean done;
    private boolean firstTime;

    /**
     * Lawnmower Movement executes lawnmower sweeping pattern by a drone
     * around a fog vehicle.
    * Creates a new movement model based on a Settings object's settings.
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
    }

    /**
    * Copy constructor.
    * @param lm The LawnmoverMovement prototype
    */
    public LawnmoverMovement(LawnmoverMovement lm) {
        super(lm);
        // Assume the first interface to be bluetooth interface
        horizontalShift = -1 * droneRange * 2;
        verticalShift = -1 * fogRange;
        this.horizontalShiftSum = 0;
        this.turning = true;
        this.firstTime = true;
        this.initLoc = lm.startLoc;
    }
    
    /**
     * Returns a single coordinate path (using the only possible coordinate)
     * 
     * @return a single coordinate path
	 */
    @Override
	public Path getPath() {
        //first motion after starting the simulation 
	    if (firstTime) {
            this.nextPath = new Path(0.05);
            this.nextPath.addWaypoint(this.initLoc);
            this.endLoc = initLoc.clone();
            endLoc.translate(0, verticalShift);
            this.nextPath.addWaypoint(endLoc.clone());
            firstTime = false;
	    }
        // Pass a clone of the Coord object since the original Coord would
        // be changed by getPath() faster than it is applied in the simulation
        this.nextPath = new Path(0.05);
        while(horizontalShiftSum <= fogRange) {
            if(turning==true) {
                endLoc.translate(horizontalShift , 0 );
                horizontalShiftSum += Math.abs(horizontalShift);
                turning = !turning;
                this.nextPath.addWaypoint(endLoc.clone());
            }
            else{
                verticalShift *= -1;
                endLoc.translate(0, 2 * verticalShift);
                this.nextPath.addWaypoint(endLoc.clone());
                turning = !turning;
            }
        }

        horizontalShift *= -1;
        horizontalShiftSum = 0;
        this.nextPath.addWaypoint(this.initLoc);
        this.endLoc = this.initLoc.clone();
        verticalShift *= -1;
        endLoc.translate(0, verticalShift);
        this.nextPath.addWaypoint(endLoc.clone());
        turning = true;
        
        // this is pretty bad form
        // sacrifices are required though
        while(horizontalShiftSum <= fogRange) {
            if(turning==true) {
                endLoc.translate(horizontalShift , 0 );
                horizontalShiftSum += Math.abs(horizontalShift);
                turning = !turning;
                this.nextPath.addWaypoint(endLoc.clone());
            }
            else{
                verticalShift *= -1;
                endLoc.translate(0, 2 * verticalShift);
                this.nextPath.addWaypoint(endLoc.clone());
                turning = !turning;
            }
        }
        
        if (horizontalShiftSum > fogRange){
            this.nextPath.addWaypoint(this.initLoc);
            done = true;
        }
        
        Path p = nextPath;
        if (done) {
            this.nextPath = null;
        }
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
        // self = SimScenario.getInstance().getHosts();
        // while(hostCounter < self.size()) {
        //     DTNHost host = self.get(hostCounter++);
        //     String hostName = host.getName();
        //     if(hostName.startsWith("m")) {
        //         this.host = host;
        //     }
        //     if(hostName.startsWith("p")) {
        //         fogHost = host;
        //     }
        // }
        
        this.nextPath = new Path(generateSpeed());
        this.nextPath.addWaypoint(new Coord(150,150));
        
        // this.initLoc = endLoc.clone();
        // this.nextPath.addWaypoint(this.initLoc);
        // this.endLoc = endLoc.clone();
        this.nextPath.addWaypoint(endLoc.clone());

        if(turning) {
            endLoc.translate(-1* horizontalShift , 0 );
            this.nextPath.addWaypoint(endLoc.clone());
        }
        else{
            endLoc.translate(verticalShift, 0);
            verticalShift *= -1;
            this.nextPath.addWaypoint(endLoc.clone());
        }

        turning = !turning;
        return true;
        

        // if(turning == true) {
        //     if (horizontalShiftSum >= verticalShift) {
        //         // if (horizontalShift < 0) {
        //         //     this.nextPath = null;
        //         //     return false;
        //         // }
        //         this.endLoc = this.startLoc.clone();
        //         horizontalShift = -horizontalShift;
        //         this.horizontalShiftSum = 0;
        //     }

        //     endLoc.translate(horizontalShift, 0 );
        //     horizontalShiftSum += horizontalShift;
        // } else { 
        //     endLoc.translate(0, verticalShift * 2);
        // }
        
        // turning = !turning;
        // this.nextPath.addWaypoint(endLoc);
        // return true;
    }

    @Override
    public Coord getInitialLocation() {
        return this.initLoc;
    }

    @Override
	public LawnmoverMovement replicate() {
		return new LawnmoverMovement(this);
	}

	@Override
	public void setLocation(Coord lastWaypoint) {
	    this.initLoc = lastWaypoint.clone();
	}

	@Override
	public boolean isReady() {
	    return true;
	}

	@Override
	public Coord getLastLocation() {
	    return endLoc.clone();
	}
}
