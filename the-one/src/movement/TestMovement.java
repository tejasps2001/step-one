package movement;

import core.Coord;
import java.util.List;
import core.DTNHost;
import core.SimScenario;
import core.Settings;
import core.Connection;

public class TestMovement extends ExtendedMovementModel {
    private ExtendedLinearMovement extendedLinearMM;
    private RandomWalk randomWalkMM;

    private static final int EXTENDED_LINEAR_MODE = 1;
    private static final int RANDOM_WALK_MODE = 2;

    private int mode;
    private DTNHost host;
    private int hostCounter;
    private DTNHost fogHost;
    private Connection fogConnection;
    private List<Connection> connections;

    // Host-specific variables
    private static List<DTNHost> self;

    public TestMovement(Settings settings) {
        super(settings);
        extendedLinearMM = new ExtendedLinearMovement(settings);
        randomWalkMM = new RandomWalk(settings);
        setCurrentMovementModel(extendedLinearMM);
        mode = EXTENDED_LINEAR_MODE;
        hostCounter = 0;
    }

    /**
     * Creates a new instance of TestMovement from a prototype
     * @param proto
     */
    public TestMovement(TestMovement proto) {
        super(proto);
        extendedLinearMM = new ExtendedLinearMovement(proto.extendedLinearMM);
        setCurrentMovementModel(extendedLinearMM);
        randomWalkMM = proto.randomWalkMM.replicate();
		mode = proto.mode;
    }

    @Override
    public Coord getInitialLocation() {
        Coord initLoc = new Coord(150, 150);
        extendedLinearMM.setLocation(initLoc);
        return initLoc;
    }

    @Override
	public MovementModel replicate() {
		return new TestMovement(this);
	}

    @Override 
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
            switch (mode) {
                case EXTENDED_LINEAR_MODE:
                    System.out.println(fogConnection);
                    if(fogConnection.isUp() == false && extendedLinearMM.isReady()){
                        setCurrentMovementModel(randomWalkMM);
                        mode = RANDOM_WALK_MODE;
                    }
                    else {
                        extendedLinearMM.setNextPoint(new Coord(200, 200));
                    }
                    break;
                default:
                    break;
            }
        }
        return true;
    }
}
