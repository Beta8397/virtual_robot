package org.murraybridgebunyips.imposter.autonomous;

import androidx.annotation.NonNull;
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

import static org.murraybridgebunyips.bunyipslib.external.units.Units.*;

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
        RoadRunnerTask<RoadRunnerDrive> task = makeTrajectory()
                .turn(-90, Degrees)
                .strafeLeft(24)
                .forward(24)
                .withName("Main")
                .mirrorToRef(ref)
                .buildTask(false);

        if (selectedOpMode.toString().equals("left")) {
            addTask(task);
        } else {
            makeTrajectory()
                    .runSequence(ref.get())
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
