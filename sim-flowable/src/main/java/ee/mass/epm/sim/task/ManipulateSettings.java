package ee.mass.epm.sim.task;

import core.Settings;
import ee.mass.epm.sim.task.SimulatedTask;
import org.flowable.engine.delegate.DelegateExecution;

public class ManipulateSettings extends SimulatedTask {
    String mm;

    @Override
    public void execute(DelegateExecution execution) {
        super.execute(execution);

        // execution.setVariable("sensorValue", Math.random() );
        Settings s = new Settings("Group2");
        mm = s.getSetting("movementModel");
        System.out.println("Movement model: " + mm);
        Settings.addSettings("C:\\Users\\tejas\\Documents\\Class_14\\Project\\Case_Study\\STEP-ONE\\manipulate_settings.txt");
        // Settings s = new Settings("Group2");
        mm = s.getSetting("movementModel");
        System.out.println("Movement model: " + mm);
    }
}
