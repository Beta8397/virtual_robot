package org.firstinspires.ftc.teamcode.example.examplerobot.autonomous;

import androidx.annotation.Nullable;
import org.firstinspires.ftc.teamcode.common.AutonomousBunyipsOpMode;
import org.firstinspires.ftc.teamcode.common.OpModeSelection;
import org.firstinspires.ftc.teamcode.common.RobotConfig;
import org.firstinspires.ftc.teamcode.common.tasks.AutoTask;
import org.firstinspires.ftc.teamcode.example.examplerobot.components.ExampleConfig;
import org.firstinspires.ftc.teamcode.example.examplerobot.components.ExampleDrive;
import org.firstinspires.ftc.teamcode.example.examplerobot.tasks.ExampleTimeDriveTask;

import java.util.List;

public class ExampleAuto extends AutonomousBunyipsOpMode {
    private ExampleDrive drive;
    private ExampleConfig config = new ExampleConfig();

    @Override
    protected void onInitialisation() {
        config = (ExampleConfig) RobotConfig.newConfig(this, config, hardwareMap);
        drive = new ExampleDrive(this, config.leftMotor, config.rightMotor);
    }

    @Override
    protected List<OpModeSelection> setOpModes() {
        return null;
    }

    @Override
    protected AutoTask setInitTask() {
        return null;
    }

    @Override
    protected void onQueueReady(@Nullable OpModeSelection selectedOpMode) {
        addTask(new ExampleTimeDriveTask(this, 5.0, drive));
    }
}
