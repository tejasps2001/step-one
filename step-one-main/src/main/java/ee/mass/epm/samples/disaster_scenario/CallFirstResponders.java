package ee.mass.epm.sim.task;

import core.Settings;
import ee.mass.epm.sim.task.SimulatedTask;
import org.flowable.engine.delegate.DelegateExecution;

public class CallFirstResponders extends SimulatedTask {
    String mm;

    @Override
    public void execute(DelegateExecution execution) {
        super.execute(execution);

        System.out.println("Called first responders");
    }
}
