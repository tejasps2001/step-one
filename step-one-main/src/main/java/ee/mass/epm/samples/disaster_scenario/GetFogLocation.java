package ee.mass.epm.samples.disaster_scenario;

import java.util.List;

import org.flowable.engine.delegate.DelegateExecution;
import org.omg.CORBA.portable.Delegate;

import core.DTNHost;
import core.SimScenario;
import ee.mass.epm.sim.task.SimulatedTask;

public class GetFogLocation extends SimulatedTask {
    @Override
    public void execute(DelegateExecution execution) {
        super.execute(execution);
        
        List<DTNHost> hostList = SimScenario.getInstance().getHosts();
        DTNHost localhost = hostList.get(execution.getVariable("localhost", Integer.class));
        DTNHost fogHost = localhost.getConnections().stream()
                .filter(c -> c.isUp() && c.getOtherNode(localhost).getName().startsWith("Fog"))
                .map(c -> c.getOtherNode(localhost))
                .findFirst().get();
        execution.setVariable("fogVehicleAddress", fogHost.getAddress());
        System.out.println("the fog addresss Is "+fogHost.getAddress());
    }
}
