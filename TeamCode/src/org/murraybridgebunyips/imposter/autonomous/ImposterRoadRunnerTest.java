package org.murraybridgebunyips.imposter.autonomous;

import androidx.annotation.NonNull;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.murraybridgebunyips.bunyipslib.AutonomousBunyipsOpMode;
import org.murraybridgebunyips.bunyipslib.Reference;
import org.murraybridgebunyips.bunyipslib.RoadRunner;
import org.murraybridgebunyips.bunyipslib.drive.MecanumDrive;
import org.murraybridgebunyips.bunyipslib.OpModeSelection;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;
import org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence.TrajectorySequence;
import org.murraybridgebunyips.bunyipslib.tasks.RoadRunnerTask;
import org.murraybridgebunyips.imposter.components.ImposterConfig;
import org.jetbrains.annotations.Nullable;

import static org.murraybridgebunyips.bunyipslib.external.units.Units.InchesPerSecond;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.Second;

@Autonomous(name = "RoadRunnerTest", group = "VIRTUAL_BUNYIPSFTC")
public class ImposterRoadRunnerTest extends AutonomousBunyipsOpMode implements RoadRunner {
    private final ImposterConfig config = new ImposterConfig();
    private MecanumDrive drive;
    private final Reference<TrajectorySequence> ref = Reference.empty();

    @Override
    protected void onInitialise() {
        config.init();
        drive = new MecanumDrive(config.driveConstants, config.mecanumCoefficients, hardwareMap.voltageSensor, config.imu, config.front_left_motor, config.front_right_motor, config.back_left_motor, config.back_right_motor);
        setOpModes("left", "right");
    }

    @Override
    protected void onReady(@Nullable OpModeSelection selectedOpMode) {
        if (selectedOpMode == null) return;
        RoadRunnerTask<RoadRunnerDrive> task = makeTrajectory(new Pose2d(-36.78, -60.52, Math.toRadians(0.00)))
                .splineTo(new Vector2d(17.69, -58.50), Math.toRadians(6.38))
                .splineTo(new Vector2d(34.60, -20.95), Math.toRadians(61.02))
                .splineTo(new Vector2d(60.83, -6.05), Math.toRadians(57.59))
                .withName("Main")
                .mirrorToRef(ref)
                .buildTask(false);

        if (selectedOpMode.toString().equals("left")) {
            addTask(task);
        } else {
            makeTrajectory()
                    .runSequence(ref.require())
                    .withName("Mirror Main")
                    .addTask();
        }
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
