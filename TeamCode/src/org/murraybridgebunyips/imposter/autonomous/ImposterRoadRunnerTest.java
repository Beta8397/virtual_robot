package org.murraybridgebunyips.imposter.autonomous;

import androidx.annotation.NonNull;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.murraybridgebunyips.bunyipslib.*;
import org.murraybridgebunyips.bunyipslib.drive.MecanumDrive;
import org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence.TrajectorySequence;
import org.murraybridgebunyips.bunyipslib.tasks.RunTask;
import org.murraybridgebunyips.bunyipslib.tasks.WaitTask;
import org.murraybridgebunyips.bunyipslib.tasks.groups.ParallelTaskGroup;
import org.murraybridgebunyips.imposter.components.ImposterConfig;
import org.jetbrains.annotations.Nullable;

import static org.murraybridgebunyips.bunyipslib.external.units.Units.*;

@Autonomous(name = "RoadRunnerTest", group = "VIRTUAL_BUNYIPSFTC")
public class ImposterRoadRunnerTest extends AutonomousBunyipsOpMode implements RoadRunner {
    private final ImposterConfig config = new ImposterConfig();
    private MecanumDrive drive;

    @Override
    protected void onInitialise() {
        config.init();
        drive = new MecanumDrive(config.driveConstants, config.mecanumCoefficients, hardwareMap.voltageSensor, config.imu, config.front_left_motor, config.front_right_motor, config.back_left_motor, config.back_right_motor);
        setOpModes(StartingPositions.use());
        setInitTask(new WaitTask(Seconds.of(1), false));
        addSubsystems(drive);
    }

    @Override
    protected void onReady(@Nullable Reference<?> selectedOpMode, Controls selectedButton) {
        if (selectedOpMode == null) return;
        Dbg.log(selectedButton);
        Reference<TrajectorySequence> blueLeft = Reference.empty();
        Reference<TrajectorySequence> blueRight = Reference.empty();

        float FIELD_TILE_SCALE = 1.1f;
        TrajectorySequence redLeft = makeTrajectory()
                .forward(2 * FIELD_TILE_SCALE, FieldTiles)
                .strafeRight(2.8 * FIELD_TILE_SCALE, FieldTiles)
                .turn(-Math.PI / 2)
                .strafeRight(1 * FIELD_TILE_SCALE, FieldTile)
                .mirrorToRef(blueRight)
                .build();

        TrajectorySequence redRight = makeTrajectory()
                .lineToLinearHeading(new Pose2d(1 * FIELD_TILE_SCALE, -1 * FIELD_TILE_SCALE, -90.0), FieldTiles, Degrees)
                .mirrorToRef(blueLeft)
                .build();

        StartingPositions startingPosition = (StartingPositions) selectedOpMode.require();
        TrajectorySequence targetTrajectory = null;
        switch (startingPosition) {
            case STARTING_RED_LEFT:
                targetTrajectory = redLeft;
                break;
            case STARTING_RED_RIGHT:
                targetTrajectory = redRight;
                break;
            case STARTING_BLUE_LEFT:
                targetTrajectory = blueLeft.require();
                break;
            case STARTING_BLUE_RIGHT:
                targetTrajectory = blueRight.require();
                break;
        }
        assert targetTrajectory != null;
        makeTrajectory()
                .runSequence(targetTrajectory)
                .withName("Navigate to Backboard")
                .addTask();

        addTask(new WaitTask(Seconds.of(2)).withName("Deploy Arm"));
        addTask(new RunTask().withName("Drop Pixels"));
        addTask(new WaitTask(Seconds.of(1)).withName("Wait for Pixels"));
        addTask(new ParallelTaskGroup(
                makeTrajectory()
                        .strafeLeft(0.95 * FIELD_TILE_SCALE, FieldTile)
                        .buildTask(),
                new WaitTask(Seconds.of(2))
        ).withName("Stow and Move to Park"));

        makeTrajectory()
                .forward(1.1 * FIELD_TILE_SCALE, FieldTiles)
                .withName("Finish Park")
                .addTask();
    }

    @Override
    protected void periodic() {
    }

    @NonNull
    @Override
    public MecanumDrive getDrive() {
        return drive;
    }
}
