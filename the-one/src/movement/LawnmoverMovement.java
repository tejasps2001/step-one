package movement;

import java.util.List;

import core.Connection;
import core.Coord;
import core.DTNHost;
import core.Settings;
import core.SimScenario;

/**
 * This movement model makes the host execute the lawnmover pattern
 */
public class LawnmoverMovement extends ExtendedMovementModel implements SwitchableMovement {
    /** Name space of the settings (append to group name space) */
    public static final String LAWNMOVER_MOVEMENT_NS = "LawnmoverMovement.";
    /** 
     * Per node group setting for defining the start coordinates of the 
     * line({@value})
     */
    public static final String START_LOCATION_S = "startLocation";

    private Coord startLoc;
    private int offset;
    private GranularMovement granularMM;
    private Coord initLoc;
    private Path nextPath;
    private prevLocation;

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
        granularMM = new GranularMovement(settings);
        coords = settings.getCsvInts(LAWNMOVER_MOVEMENT_NS + START_LOCATION_S, 2);
        this.startLoc = new Coord(coords[0], coords[1]);
        setCurrentMovementModel(granularMM);
    }

    /**
    * Copy constructor.
    * @param lm The LawnmoverMovement prototype
    */
    public LawnmoverMovement(LawnmoverMovement lm) {
        super(lm);
        granularMM.setLocation(initLoc);
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

        this.connections = this.host.getConnections();
        for(Connection connection: connections) {
            if(connection.getOtherNode(this.host) == fogHost) {
                fogConnection = connection;
                // System.out.println(fogConnection);
                break;
            }
        }

        if(fogConnection != null) {
            if(fogConnection.isUp() == false && granularMM.isReady()) {

            }
            else {
                d
                granularMM.generateNextPath("up");

            }
        }
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
