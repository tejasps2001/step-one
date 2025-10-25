package ee.mass.epm.samples.disaster_scenario;

import core.Settings;
import ee.mass.epm.sim.task.SimulatedTask;
import org.flowable.engine.delegate.DelegateExecution;

public class CallFirstResponders extends SimulatedTask {
    String mm;

    @Override
    public void execute(DelegateExecution execution) {
        super.execute(execution);
        String humanLocation = 
            (String) execution.getVariable("humanLocation");
        System.out.println("Called first responders for " + humanLocation);
    }
}
