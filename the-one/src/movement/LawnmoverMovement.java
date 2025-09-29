package movement;

import core.Coord;
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
    /**Group ID of the fog nodes */
    public static final String FOG_GROUP_ID = "Group2.";

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
    private Coord fogLoc;

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
        // Create another settings object to access other node group's 
        // settings
        Settings globalSettings = new Settings();
        droneRange = globalSettings.getDouble("btInterface.transmitRange");
        fogRange = globalSettings.getDouble("wlanInterface.transmitRange");
        coords = globalSettings.getCsvInts(FOG_GROUP_ID + "nodeLocation", 2);
        fogLoc = new Coord(coords[0], coords[1]);
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
        this.nextPath = new Path(generateSpeed());
        this.nextPath.addWaypoint(this.initLoc);
        this.endLoc = initLoc.clone();
        endLoc.translate(0, verticalShift);
        this.nextPath.addWaypoint(endLoc.clone());
        this.fogLoc = lm.fogLoc.clone();
    }
    
    /**
     * Returns a single coordinate path (using the only possible coordinate)
     * 
     * @return a single coordinate path
	 */
    @Override
	public Path getPath() {
        //first motion after starting the simulation 
	    System.out.println("init loc: " + initLoc);
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

        if(horizontalShiftSum > fogRange){
            System.err.println(fogLoc);
            this.nextPath.addWaypoint(fogLoc.clone());
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
	    System.out.println("set initLocation: " + lastWaypoint);
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
