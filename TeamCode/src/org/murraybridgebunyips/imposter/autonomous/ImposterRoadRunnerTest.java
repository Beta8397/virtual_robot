package org.murraybridgebunyips.imposter.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.murraybridgebunyips.bunyipslib.MecanumDrive;
import org.murraybridgebunyips.bunyipslib.OpModeSelection;
import org.murraybridgebunyips.bunyipslib.RoadRunnerAutonomousBunyipsOpMode;
import org.murraybridgebunyips.bunyipslib.tasks.bases.RobotTask;
import org.murraybridgebunyips.imposter.components.ImposterConfig;
import org.jetbrains.annotations.Nullable;

import java.util.List;

@Autonomous(name = "RoadRunnerTest", group = "VIRTUAL_BUNYIPSFTC")
public class ImposterRoadRunnerTest extends RoadRunnerAutonomousBunyipsOpMode<MecanumDrive> {
    private final ImposterConfig config = new ImposterConfig();

    @Override
    protected void onInitialisation() {
        config.init(this);
    }

    @Override
    protected List<OpModeSelection> setOpModes() {
        return null;
    }

    @Override
    protected RobotTask setInitTask() {
        return null;
    }

    @Override
    protected void onQueueReady(@Nullable OpModeSelection selectedOpMode) {
        // Start on RED_LEFT, forward facing Spike Marks, will park left of the backboard
        addNewTrajectory(new Pose2d(-36.76, -61.58, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-34.29, -15.92), Math.toRadians(13.90))
                .splineTo(new Vector2d(63.66, -11.94), Math.toRadians(0.00))
                .build();
    }

    @Override
    protected MecanumDrive setDrive() {
        return new MecanumDrive(this, config.driveConstants, config.mecanumCoefficients, hardwareMap.voltageSensor, config.imu, config.front_left_motor, config.front_right_motor, config.back_left_motor, config.back_right_motor);
    }
}
