package movement;

import core.Coord;
import core.Settings;

/**
 * A movement model that ensures drones are deployed directly
 * from the Fog UAV's starting location.
 */
public class FogDeployedGDRRTMovement extends ExtendedGDRRTMovement {

    private Coord deploymentLoc;

    public FogDeployedGDRRTMovement(Settings s) {
        super(s);
        
        String[] namespacesToTry = {
            "Group3.WOAFogMovement",
            "Group3.MILPClusteredFogMovement"
        };

        Coord resolved = null;
        for (String ns : namespacesToTry) {
            Settings fogSettings = new Settings(ns);
            if (fogSettings.contains("startLocation")) {
                int[] coords = fogSettings.getCsvInts("startLocation", 2);
                resolved = new Coord(coords[0], coords[1]);
                break;
            }
        }

        if (resolved != null) {
            this.deploymentLoc = resolved;
        } else {
            // Print error if startLocation is missing from both WOA and MILP
            // But also select a default location            this.deploymentLoc = new Coord(2500, 2500);
            System.err.println("[FogDeployedGDRRTMovement] WARNING: 'startLocation'"
                    + " not found in any known fog namespace"
                    + " (tried: Group3.WOAFogMovement, Group3.MILPClusteredFogMovement)."
                    + " Falling back to hard-coded (2500, 2500)."
                    + " If this is wrong, add startLocation to your fog group settings.");
        }
    }

    public FogDeployedGDRRTMovement(FogDeployedGDRRTMovement proto) {
        super(proto);
        this.deploymentLoc = proto.deploymentLoc.clone();
    }

    @Override
    public Coord getInitialLocation() {
        // Drones start from the Fog UAV's location.
        return deploymentLoc.clone();
    }

    @Override
    public FogDeployedGDRRTMovement replicate() {
        return new FogDeployedGDRRTMovement(this);
    }
}
