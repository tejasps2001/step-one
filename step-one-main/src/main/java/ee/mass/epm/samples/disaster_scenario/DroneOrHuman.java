package ee.mass.epm.samples.disaster_scenario;

import java.util.List;

import org.flowable.engine.delegate.DelegateExecution;
import core.Connection;
import core.DTNHost;
import core.SimScenario;
import ee.mass.epm.sim.task.SimulatedTask;

public class DroneOrHuman extends SimulatedTask {
    @Override
    public void execute(DelegateExecution execution) {
        super.execute(execution);
        
        List<DTNHost> hostList = SimScenario.getInstance().getHosts();
        DTNHost localhost = hostList.get(execution.getVariable("localhost", Integer.class));
        List<Connection> droneConnections = localhost.getConnections();
        
        for(Connection c : droneConnections) {
            
            if(c.isUp() && c.getOtherNode(localhost).getName().startsWith("Human") ) {
                execution.setVariable("isHuman", true);
                System.out.println("The value of vairable is ISHUMan is "+execution.getVariable("isHuman"));
                return;
            }
        }
        execution.setVariable("isHuman", false);
        System.out.println("The value of vairable is ISHUMan is "+execution.getVariable("isHuman"));
    }
}
