package movement;

import core.Coord;
import core.Settings;

public class TestMovement extends ExtendedMovementModel {
    private ExtendedLinearMovement extendedLinearMM;
    private RandomWalk randomWalkMM;

    private static final int EXTENDED_LINEAR_MODE = 1;
    private static final int RANDOM_WALK_MODE = 2;

    private int mode;

    public TestMovement(Settings settings) {
        super(settings);
        extendedLinearMM = new ExtendedLinearMovement(settings);
        randomWalkMM = new RandomWalk(settings);
        setCurrentMovementModel(extendedLinearMM);
        mode = EXTENDED_LINEAR_MODE;
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
        switch (mode) {
            case EXTENDED_LINEAR_MODE:
                if(extendedLinearMM.isReady())
                    setCurrentMovementModel(randomWalkMM);
                break;
        
            default:
                break;
        }
        return true;
    }
}
