package org.firstinspires.ftc.teamcode.glados.autonomous;

import androidx.annotation.Nullable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.common.AutonomousBunyipsOpMode;
import org.firstinspires.ftc.teamcode.common.OpModeSelection;
import org.firstinspires.ftc.teamcode.common.RobotConfig;
import org.firstinspires.ftc.teamcode.common.StartingPositions;
import org.firstinspires.ftc.teamcode.common.tasks.AutoTask;
import org.firstinspires.ftc.teamcode.glados.components.GLaDOSConfigCore;
import org.firstinspires.ftc.teamcode.glados.components.GLaDOSPOVDriveCore;
import org.firstinspires.ftc.teamcode.glados.tasks.GLaDOSTimeDriveTask;

import java.util.Arrays;
import java.util.List;

/**
 * Drive GLaDOS to the left side of the backstage parking zone immediately.
 *
 * @author Lucas Bubner, 2023
 */
@Autonomous(name = "GLaDOS: Backstage LEFT Park", group = "GLaDOS")
public class GLaDOSParkLeftBackstageAuto extends AutonomousBunyipsOpMode {
    private GLaDOSConfigCore config = new GLaDOSConfigCore();
    private GLaDOSPOVDriveCore drive;

    @Override
    protected void onInitialisation() {
        config = (GLaDOSConfigCore) RobotConfig.newConfig(this, config, hardwareMap);
        drive = new GLaDOSPOVDriveCore(this, config.fl, config.bl, config.fr, config.br);
    }

    @Override
    protected List<OpModeSelection> setOpModes() {
        return Arrays.asList(new OpModeSelection(StartingPositions.RED_LEFT), new OpModeSelection(StartingPositions.RED_RIGHT), new OpModeSelection(StartingPositions.BLUE_LEFT), new OpModeSelection(StartingPositions.BLUE_RIGHT));
    }

    @Override
    protected AutoTask setInitTask() {
        return null;
    }

    @Override
    protected void onQueueReady(@Nullable OpModeSelection selectedOpMode) {
        if (selectedOpMode == null) {
            // Will bail out if the user does not select an OpMode, this is in case the robot
            // is started and cannot perform any useful action without disrupting the alliance.
            return;
        }
        StartingPositions selected = (StartingPositions) selectedOpMode.getObj();
        switch (selected) {
            case RED_LEFT:
                addTask(new GLaDOSTimeDriveTask(this, 2, drive, 0, 0.75, 0));
                addTask(new GLaDOSTimeDriveTask(this, 4, drive, 0.75, 0, 0));
                break;
            case RED_RIGHT:
                addTask(new GLaDOSTimeDriveTask(this, 2, drive, 0, 0.75, 0));
                addTask(new GLaDOSTimeDriveTask(this, 2, drive, 0.75, 0, 0));
                break;
            case BLUE_LEFT:
                addTask(new GLaDOSTimeDriveTask(this, 2, drive, -0.75, 0, 0));
                break;
            case BLUE_RIGHT:
                addTask(new GLaDOSTimeDriveTask(this, 4, drive, -0.75, 0, 0));
                break;
        }
    }
}
