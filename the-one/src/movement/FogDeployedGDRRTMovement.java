package movement;

import core.Coord;
import core.Settings;

/**
 * A wrapper movement model that ensures drones are deployed directly
 * from the Fog UAV's starting location.
 * It extends GDRRTMovement to inherit all pathfinding and movement logic,
 * but overrides the initial location to sync dynamically with the Fog UAV.
 */
public class FogDeployedGDRRTMovement extends GDRRTMovement {
    
    private Coord deploymentLoc;

    public FogDeployedGDRRTMovement(Settings s) {
        super(s); 
        
        // Read the Fog UAV's starting coordinate directly from its settings namespace
        Settings fogSettings = new Settings("Group3.MILPClusteredFogMovement");
        if (fogSettings.contains("startLocation")) {
            int[] coords = fogSettings.getCsvInts("startLocation", 2);
            this.deploymentLoc = new Coord(coords[0], coords[1]);
        } else {
            this.deploymentLoc = new Coord(2500, 2500); // Fallback to standard testing start
        }
    }

    public FogDeployedGDRRTMovement(FogDeployedGDRRTMovement proto) {
        super(proto);
        this.deploymentLoc = proto.deploymentLoc.clone();
    }

    @Override
    public Coord getInitialLocation() {
        // Drones start exactly at the Fog UAV's location, deploying outward
        return deploymentLoc.clone(); 
    }

    @Override
    public FogDeployedGDRRTMovement replicate() {
        return new FogDeployedGDRRTMovement(this);
    }
}