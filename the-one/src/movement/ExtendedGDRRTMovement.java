package movement;

import core.Coord;
import core.Settings;
import core.SettingsError;

import java.util.ArrayList;
import java.util.List;

public class ExtendedGDRRTMovement extends MovementModel implements SwitchableMovement {
    // ExtendedGDRRTMovement will use the same namespace as GDRRTMovement so that the 
    // settings files work without needing any modification.
    public static final String GDRRT_MOVEMENT_NS = "GDRRTMovement.";

    public static final String START_LOCATION_S = "startLocation";
    public static final String END_LOCATION_S = "endLocation";
    public static final String OBSTACLE_FILE_S = "obstacleFile";

    private List<Coord> startLocs;
    private List<Coord> endLocs;
    private static int nextHostIndex = 0;

    private Coord startLoc;
    private Coord endLoc;
    private String obstacleFilePath;
    private boolean reachedEnd = false;
    private GDRRTPlanner gdrrt;
    private boolean isWaiting = false;
    private boolean isDead = false;
    private double currentPriority = -1.0;
    private long totalComputeTimeNs = 0;
    private double totalTurnCost = 0.0;
    private Double lastHeading = null;

    /**
     * Creates a new movement model based on a Settings object's settings.
     *
     * @param s The Settings object where the settings are read from
     */
    public ExtendedGDRRTMovement(Settings s) {
        super(s);
        this.startLocs = parseCoords(s, GDRRT_MOVEMENT_NS + START_LOCATION_S);
        this.endLocs = parseCoords(s, GDRRT_MOVEMENT_NS + END_LOCATION_S);

        if (startLocs.size() != endLocs.size()) {
            throw new SettingsError("Start and end location arrays must have the same size");
        }

        int nrofHosts = s.getInt(core.SimScenario.NROF_HOSTS_S);
        if (startLocs.size() < nrofHosts) {
            throw new SettingsError("Not enough start/end locations for all hosts in the group");
        }

        this.obstacleFilePath = s.getSetting(GDRRT_MOVEMENT_NS + OBSTACLE_FILE_S);
        this.gdrrt = new GDRRTPlanner(this.obstacleFilePath);
        nextHostIndex = 0; 
    }

    // Helper to parse coordinate arrays like "[x1,y1; x2,y2]"
    private List<Coord> parseCoords(Settings s, String key) {
        String raw = s.getRawSetting(key);
        raw = raw.replace("[", "").replace("]", "");
        String[] pairs = raw.split(";");
        List<Coord> coords = new ArrayList<>();
        for (String pair : pairs) {
            String[] values = pair.split(",");
            if (values.length != 2) {
                throw new SettingsError("Invalid coordinate pair: " + pair + " in setting " + key);
            }
            coords.add(new Coord(Double.parseDouble(values[0].trim()), Double.parseDouble(values[1].trim())));
        }
        return coords;
    }

    /**
     * Copy constructor used for each node in group
     * @param proto
     */
    public ExtendedGDRRTMovement(ExtendedGDRRTMovement proto) {
        super(proto);
        this.startLoc = proto.startLocs.get(nextHostIndex);
        this.endLoc = proto.endLocs.get(nextHostIndex);
        nextHostIndex++;
        this.isDead = proto.isDead;
        this.currentPriority = proto.currentPriority;

        this.obstacleFilePath = proto.obstacleFilePath;
        this.gdrrt = new GDRRTPlanner(this.obstacleFilePath);
    }

    @Override
    public ExtendedGDRRTMovement replicate() {
        return new ExtendedGDRRTMovement(this);
    }

    public double getPriority() {
        if (currentPriority < 0 && getHost() != null) {
            currentPriority = 1.0 + getHost().getAddress();
        }
        return currentPriority;
    }

    public void setPriority(double priority) {
        this.currentPriority = priority;
    }

    @Override
    public Path getPath() {
        long startNs = System.nanoTime();
        try {
            if (reachedEnd || isDead) return null;

            if (getHost() != null && getHost().getLocation().distance(endLoc) < 1.0) {
                reachedEnd = true;
                ExtendedDronePathManager.setStationary(getHost().getAddress());
                System.out.println("Drone " + getHost().getAddress() + " successfully reached its goal at " + core.SimClock.getTime() + "s!");
                return null;
            }

            if (gdrrt == null) gdrrt = new GDRRTPlanner(this.obstacleFilePath);
            if (!gdrrt.isInitialized()) gdrrt.init(getHost().getLocation(), endLoc);

            GDRRTPlanner.PlannedSegment proposedSegment = gdrrt.planNextSegment();

            if (proposedSegment == null) { 
                reachedEnd = true;
                System.out.println("Drone " + getHost().getAddress()
                        + " planner returned null. Setting reachedEnd = true and giving up.");
                ExtendedDronePathManager.setStationary(getHost().getAddress());
                return null;
            }

            if (ExtendedDronePathManager.requestPath(getHost().getAddress(), proposedSegment.path)) {
                isWaiting = false;
                gdrrt.commit(proposedSegment);

                if (proposedSegment.isFinalPath) {
                    ExtendedDronePathManager.setStationary(getHost().getAddress());
                }

                updateTurnCost(proposedSegment.path);
                return proposedSegment.path;
            } else {
                isWaiting = true;
                ExtendedDronePathManager.setStationary(getHost().getAddress());
                gdrrt.init(getHost().getLocation(), endLoc);
                Path waitingPath = new Path(0);
                waitingPath.addWaypoint(getHost().getLocation());
                return waitingPath;
            }
        } finally {
            this.totalComputeTimeNs += (System.nanoTime() - startNs);
        }
    }

    public boolean isDone() { return reachedEnd; }
    @Override public boolean isReady() { return reachedEnd; }
    public boolean isDead() { return isDead; }

    public void kill() {
        this.isDead = true;
        this.reachedEnd = true;
        if (getHost() != null) {
            if (getHost().getPath() != null) {
                Coord currentLoc = getHost().getLocation().clone();
                getHost().getPath().getCoords().clear();
                getHost().getPath().getCoords().add(currentLoc);
            }
        }
        ExtendedDronePathManager.setStationary(getHost().getAddress());
        System.out.println("Drone " + getHost().getAddress() + " was SHOT DOWN at " + core.SimClock.getTime() + "s!");
    }

    public void changeTarget(Coord newTarget, double newPriority) {
        this.endLoc = newTarget;
        this.reachedEnd = false;
        this.currentPriority = newPriority;
        this.gdrrt.init(getHost().getLocation(), newTarget);
        
        if (getHost() != null) {
            if (getHost().getPath() != null) {
                Coord currentLoc = getHost().getLocation().clone();
                getHost().getPath().getCoords().clear();
                getHost().getPath().getCoords().add(currentLoc);
            }
        }
        System.out.println("Drone " + getHost().getAddress() + " re-routed. New target: " + newTarget);
    }

    @Override
    public Coord getInitialLocation() {
        long startNs = System.nanoTime();
        try {
            this.totalTurnCost = 0.0;
            this.lastHeading = null;
            return this.startLoc.clone();
        } finally {
            this.totalComputeTimeNs += (System.nanoTime() - startNs);
        }
    }

    /**
     * Calculate the cost for turning in degrees
     * Used for generating reports
     */
    private void updateTurnCost(Path p) {
        if (p == null || p.getCoords().size() < 2) return;
        List<Coord> coords = p.getCoords();
        for (int i = 0; i < coords.size() - 1; i++) {
            Coord c1 = coords.get(i);
            Coord c2 = coords.get(i + 1);
            double dx = c2.getX() - c1.getX();
            double dy = c2.getY() - c1.getY();
            if (Math.hypot(dx, dy) > 1e-3) {
                double heading = Math.atan2(dy, dx);
                if (lastHeading != null) {
                    double diff = Math.abs(heading - lastHeading);
                    while (diff > Math.PI) diff -= 2 * Math.PI;
                    totalTurnCost += Math.abs(diff);
                }
                lastHeading = heading;
            }
        }
    }

    @Override
    public double nextPathAvailable() {
        if (reachedEnd || isDead) return Double.MAX_VALUE;
        if (isWaiting) return core.SimClock.getTime() + 1.0;
        return 0;
    }

    @Override
    public void setLocation(Coord lastWaypoint) { this.startLoc = lastWaypoint.clone(); }
    @Override
    public Coord getLastLocation() { return getHost() != null ? getHost().getLocation().clone() : this.startLoc.clone(); }

    public void setEndLocation(Coord endLoc) { this.endLoc = endLoc; }
    public Coord getEndLocation() { return this.endLoc; }
    public double getComputeTimeSeconds() { return this.totalComputeTimeNs / 1e9; }
    public double getPathSmoothness() { return this.totalTurnCost; }
}