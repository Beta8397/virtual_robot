package org.murraybridgebunyips.imposter.autonomous;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.murraybridgebunyips.bunyipslib.AutonomousBunyipsOpMode;
import org.murraybridgebunyips.bunyipslib.RoadRunner;
import org.murraybridgebunyips.bunyipslib.drive.MecanumDrive;
import org.murraybridgebunyips.bunyipslib.OpModeSelection;
import org.murraybridgebunyips.imposter.components.ImposterConfig;
import org.jetbrains.annotations.Nullable;

import static org.murraybridgebunyips.bunyipslib.external.units.Units.FieldTiles;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.Inches;

@Autonomous(name = "RoadRunnerTest", group = "VIRTUAL_BUNYIPSFTC")
public class ImposterRoadRunnerTest extends AutonomousBunyipsOpMode implements RoadRunner {
    private final ImposterConfig config = new ImposterConfig();
    private MecanumDrive drive;

    @Override
    protected void onInitialise() {
        config.init();
        drive = new MecanumDrive(config.driveConstants, config.mecanumCoefficients, hardwareMap.voltageSensor, config.imu, config.front_left_motor, config.front_right_motor, config.back_left_motor, config.back_right_motor);
    }

    @Override
    protected void onReady(@Nullable OpModeSelection selectedOpMode) {
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

    @NonNull
    @Override
    public MecanumDrive getDrive() {
        return drive;
    }
}
