package org.murraybridgebunyips.bunyipslib.example.examplerobot.autonomous;

import static org.murraybridgebunyips.bunyipslib.external.units.Units.Seconds;

import org.murraybridgebunyips.bunyipslib.AutonomousBunyipsOpMode;
import org.murraybridgebunyips.bunyipslib.Direction;
import org.murraybridgebunyips.bunyipslib.OpModeSelection;
import org.murraybridgebunyips.bunyipslib.example.examplerobot.components.ExampleConfig;
import org.murraybridgebunyips.bunyipslib.tasks.GetSignalTask;
import org.murraybridgebunyips.bunyipslib.tasks.WaitTask;
import org.murraybridgebunyips.bunyipslib.vision.Vision;

/**
 * Autonomous example of using POWERPLAY AprilTag detection to determine the parking position.
 */
public class ExampleSignalAutonomous extends AutonomousBunyipsOpMode {
    private final ExampleConfig config = new ExampleConfig();
    private GetSignalTask initTask;

    // Generally, fields you want should always be declared as class members rather than local
    // This is for clarity, memory management, and for consistency with the rest of the codebase.
    // All components and tasks must be instantiated during runtime, and not in the constructor or member fields.
    @SuppressWarnings("FieldCanBeLocal")
    private Vision cam;

    @Override
    protected void onInitialise() {
        config.init();
        cam = new Vision(config.webcam);
        initTask = new GetSignalTask(cam);
        // Will run this task until it is complete, then move on to onInitDone(), or will terminate
        // once the OpMode is started.
        setInitTask(initTask);
    }

    @Override
    protected void onInitDone() {
        // Can access initTask.getPosition() here
        // e.g
        if (initTask.getPosition() == Direction.FORWARD) {
            // Do something. Note that the first and last variants of the addTask method respect
            // the asynchronous nature of onReady, and will be queued appropriately.
            addTaskFirst(new WaitTask(Seconds.of(5.0)));
        }
    }

    @Override
    protected void onReady(OpModeSelection selectedOpMode) {
        addTask(new WaitTask(Seconds.of(5.0)));
    }
}
