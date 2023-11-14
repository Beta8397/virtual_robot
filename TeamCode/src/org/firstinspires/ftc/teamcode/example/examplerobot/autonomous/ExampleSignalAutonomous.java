package org.firstinspires.ftc.teamcode.example.examplerobot.autonomous;

import org.firstinspires.ftc.teamcode.common.AutonomousBunyipsOpMode;
import org.firstinspires.ftc.teamcode.common.OpModeSelection;
import org.firstinspires.ftc.teamcode.common.OpenCVCam;
import org.firstinspires.ftc.teamcode.common.RobotConfig;
import org.firstinspires.ftc.teamcode.common.tasks.AutoTask;
import org.firstinspires.ftc.teamcode.common.tasks.GetSignalTask;
import org.firstinspires.ftc.teamcode.common.tasks.WaitTask;
import org.firstinspires.ftc.teamcode.example.examplerobot.components.ExampleConfig;

import java.util.List;

public class ExampleSignalAutonomous extends AutonomousBunyipsOpMode {
    private GetSignalTask initTask;
    private ExampleConfig config = new ExampleConfig();
    @SuppressWarnings("FieldCanBeLocal")
    private OpenCVCam cam;

    @Override
    protected void onInitialisation() {
        config = (ExampleConfig) RobotConfig.newConfig(this, config, hardwareMap);
        cam = new OpenCVCam(this, config.webcam, null);
        initTask = new GetSignalTask(this, cam);
    }

    @Override
    protected List<OpModeSelection> setOpModes() {
        return null;
    }

    @Override
    protected AutoTask setInitTask() {
        return initTask;
    }

    @Override
    protected void onInitDone() {
        // Can access initTask.getPosition() here
        // e.g
        if (initTask.getPosition() == GetSignalTask.ParkingPosition.CENTER) {
            // Do something. Note that the first and last variants of the addTask method respect
            // the asynchronous nature of onQueueReady, and will be queued appropriately.
            addTaskFirst(new WaitTask(this, 5.0));
        }
    }

    @Override
    protected void onQueueReady(OpModeSelection selectedOpMode) {
        addTask(new WaitTask(this, 5.0));
    }
}
