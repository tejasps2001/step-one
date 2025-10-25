package ee.mass.epm.samples.disaster_scenario;


import java.util.List;

import org.flowable.engine.delegate.DelegateExecution;
import org.flowable.engine.delegate.JavaDelegate;
import ee.mass.epm.sim.task.SimulatedTask;
import core.Connection;
import core.Coord;
import core.DTNHost;

import core.SimScenario;

public class GetHumanLocation extends SimulatedTask {
    @Override
    public void execute(DelegateExecution execution) {
        super.execute(execution);
        // Retrieve the DTNHost based on the 'localhost' variable
        DTNHost localhost = SimScenario.getInstance().getHosts()
                .get(execution.getVariable("localhost", Integer.class));
        // ArrayList<Coord> humanLocations = new ArrayList<Coord>();
        

        List<Connection> droneConnections = localhost.getConnections();
        for(Connection c : droneConnections) {
            DTNHost otherNode = c.getOtherNode(localhost);
            if(c.isUp() && otherNode.getName().startsWith("Human")) {
                Coord humanLocation = c.getOtherNode(localhost).getLocation();
                execution.setVariable("humanLocation", humanLocation.toString());
            }
        }
    }
}