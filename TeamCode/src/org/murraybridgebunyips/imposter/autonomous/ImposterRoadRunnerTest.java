package org.murraybridgebunyips.imposter.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.murraybridgebunyips.bunyipslib.drive.MecanumDrive;
import org.murraybridgebunyips.bunyipslib.OpModeSelection;
import org.murraybridgebunyips.bunyipslib.RoadRunnerAutonomousBunyipsOpMode;
import org.murraybridgebunyips.bunyipslib.tasks.bases.RobotTask;
import org.murraybridgebunyips.imposter.components.ImposterConfig;
import org.jetbrains.annotations.Nullable;

import java.util.List;

import static org.murraybridgebunyips.bunyipslib.external.units.Units.FieldTiles;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.Inches;

@Autonomous(name = "RoadRunnerTest", group = "VIRTUAL_BUNYIPSFTC")
public class ImposterRoadRunnerTest extends RoadRunnerAutonomousBunyipsOpMode<MecanumDrive> {
    private final ImposterConfig config = new ImposterConfig();

    @Override
    protected void onInitialise() {
        config.init();
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
//        addNewTrajectory(new Pose2d(-36.76, -61.58, Math.toRadians(90.00)))
//                .splineTo(new Vector2d(-34.29, -15.92), Math.toRadians(13.90))
//                .splineTo(new Vector2d(63.66, -11.94), Math.toRadians(0.00))
//                .build();
        addNewTrajectory()
                .forward(Inches.convertFrom(2, FieldTiles))
                .withName("Forward 2 Tile")
                .build();
        addNewTrajectory()
                .strafeRight(Inches.convertFrom(4, FieldTiles))
                .withName("Right 4 Tiles")
                .build();
    }

    @Override
    protected MecanumDrive setDrive() {
        return new MecanumDrive(config.driveConstants, config.mecanumCoefficients, hardwareMap.voltageSensor, config.imu, config.front_left_motor, config.front_right_motor, config.back_left_motor, config.back_right_motor);
    }
}
