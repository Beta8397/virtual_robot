package org.murraybridgebunyips.bunyipslib.example.examplerobot.autonomous;

import org.murraybridgebunyips.bunyipslib.AutonomousBunyipsOpMode;
import org.murraybridgebunyips.bunyipslib.Direction;
import org.murraybridgebunyips.bunyipslib.OpModeSelection;
import org.murraybridgebunyips.bunyipslib.example.examplerobot.components.ExampleConfig;
import org.murraybridgebunyips.bunyipslib.tasks.GetSignalTask;
import org.murraybridgebunyips.bunyipslib.tasks.WaitTask;
import org.murraybridgebunyips.bunyipslib.tasks.bases.RobotTask;
import org.murraybridgebunyips.bunyipslib.vision.Vision;

import java.util.List;

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
    protected void onInitialisation() {
        config.init();
        cam = new Vision(config.webcam);
        initTask = new GetSignalTask(cam);
    }

    @Override
    protected List<OpModeSelection> setOpModes() {
        return null;
    }

    @Override
    protected RobotTask setInitTask() {
        // Will run this task until it is complete, then move on to onInitDone(), or will terminate
        // once the OpMode is started.
        return initTask;
    }

    @Override
    protected void onInitDone() {
        // Can access initTask.getPosition() here
        // e.g
        if (initTask.getPosition() == Direction.FORWARD) {
            // Do something. Note that the first and last variants of the addTask method respect
            // the asynchronous nature of onQueueReady, and will be queued appropriately.
            addTaskFirst(new WaitTask(5.0));
        }
    }

    @Override
    protected void onQueueReady(OpModeSelection selectedOpMode) {
        addTask(new WaitTask(5.0));
    }
}
