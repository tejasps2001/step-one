package report;

import movement.GDRRTMovement;

import java.util.HashMap;
import java.util.Map;

public class UavPathPlanningReport extends Report {
    private final Map<Integer, Double> startTimes = new HashMap<>();

    public UavPathPlanningReport() {
        super();

        GDRRTMovement.addListener(this);
    }

    public void droneStarted(int id, double startTime) {
        startTimes.put(id, startTime);
    }

    public void droneArrived(int id, double arrivalTime) {
        Double startTime = startTimes.get(id);
        if (startTime != null) {
            Double duration = (arrivalTime - startTime) / 1e9;
            write(String.format("Drone %d arrived. Total Time %.2f", id, duration));

            startTimes.remove(id);
        }
    }
}
