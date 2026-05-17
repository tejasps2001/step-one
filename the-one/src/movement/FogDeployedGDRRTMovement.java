package movement;

import core.Coord;
import core.Settings;

/**
 * A wrapper movement model that ensures drones are deployed directly
 * from the Fog UAV's starting location.
 * It extends ExtendedGDRRTMovement to inherit all pathfinding and movement logic,
 * but overrides the initial location to sync dynamically with the Fog UAV.
 *
 * FIX-B04: The original code hard-coded the settings namespace as
 *   "Group3.MILPClusteredFogMovement"
 * which does not exist in the WOA branch (the fog UAV uses WOAFogMovement).
 * The result was that fogSettings.contains("startLocation") always returned
 * false in the WOA branch, causing every drone to silently deploy from the
 * hard-coded fallback (2500, 2500) regardless of what was set in the config.
 * This was coincidentally correct for the current scenario but would silently
 * break if the fog start location was ever changed in the WOA settings.
 *
 * The fix tries the WOA namespace first, then falls back to the MILP namespace
 * for backwards compatibility with the Tejas branch, and finally falls back to
 * the hard-coded coordinate if neither namespace has the key.
 */
public class FogDeployedGDRRTMovement extends ExtendedGDRRTMovement {

    private Coord deploymentLoc;

    public FogDeployedGDRRTMovement(Settings s) {
        super(s);

        // FIX-B04: Try both known fog-movement namespaces in priority order.
        // 1. WOAFogMovement namespace  — used in the WOA branch
        // 2. MILPClusteredFogMovement  — used in the Tejas/MILP branch
        // This makes FogDeployedGDRRTMovement work correctly in both branches
        // without any manual change when switching between them.
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
            // Hard-coded fallback — only reached if neither namespace has the key.
            // A warning is printed so the researcher is not silently misled.
            this.deploymentLoc = new Coord(2500, 2500);
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
        // Drones start exactly at the Fog UAV's location, deploying outward
        return deploymentLoc.clone();
    }

    @Override
    public FogDeployedGDRRTMovement replicate() {
        return new FogDeployedGDRRTMovement(this);
    }
}
