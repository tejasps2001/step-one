package movement;

import java.net.NetworkInterface;
import java.util.List;

import core.Connection;
import core.Coord;
import core.DTNHost;
import core.Settings;
import core.SimScenario;

/**
 * This movement model makes the host execute the lawnmover pattern
 */
public class LawnmoverMovement extends ExtendedMovementModel {
    /** Name space of the settings (append to group name space) */
    public static final String LAWNMOVER_MOVEMENT_NS = "LawnmoverMovement.";
    /** 
     * Per node group setting for defining the start coordinates of the 
     * line({@value})
     */
    public static final String START_LOCATION_S = "startLocation";

    private Coord startLoc;
    private Coord initLoc;
    private Coord endLoc;
    private Path nextPath;
    private double hostRange;
    private boolean firstTime;

    private DTNHost host;
    private int hostCounter;
    private DTNHost fogHost;
    private Connection fogConnection;
    private List<Connection> connections;

    // Host-specific variables
    private static List<DTNHost> self;

    /**
    * Creates a new movement model based on a Settings object's settings.
    * @param s The Settings object where the settings are read from
    */
    public LawnmoverMovement(Settings settings) {
        super(settings);
        int coords[];
        coords = settings.getCsvInts(LAWNMOVER_MOVEMENT_NS + START_LOCATION_S, 2);
        this.startLoc = new Coord(coords[0], coords[1]);
        this.firstTime=true;
    }

    /**
    * Copy constructor.
    * @param lm The LawnmoverMovement prototype
    */
    public LawnmoverMovement(LawnmoverMovement lm) {
        super(lm);
        // the first interface assumed to be bluetooth interface
        this.hostRange = this.host.getInterfaces().get(0).getTransmitRange();
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

    public boolean newOrders() {
        self = SimScenario.getInstance().getHosts();
        while(hostCounter < self.size()) {
            DTNHost host = self.get(hostCounter++);
            String hostName = host.getName();
            if(hostName.startsWith("m")) {
                this.host = host;
            }
            if(hostName.startsWith("p")) {
                fogHost = host;
            }
        }
        this.nextPath = new Path(generateSpeed());
        this.nextPath.addWaypoint(host.getLocation());
        

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
}
