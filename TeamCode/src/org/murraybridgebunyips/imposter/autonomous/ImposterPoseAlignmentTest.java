package org.murraybridgebunyips.imposter.autonomous;

import androidx.annotation.NonNull;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.jetbrains.annotations.Nullable;
import org.murraybridgebunyips.bunyipslib.*;
import org.murraybridgebunyips.bunyipslib.drive.MecanumDrive;
import org.murraybridgebunyips.bunyipslib.drive.TriDeadwheelMecanumDrive;
import org.murraybridgebunyips.bunyipslib.external.pid.PIDController;
import org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence.TrajectorySequence;
import org.murraybridgebunyips.bunyipslib.tasks.DriveToPoseTask;
import org.murraybridgebunyips.bunyipslib.tasks.RunTask;
import org.murraybridgebunyips.bunyipslib.tasks.WaitTask;
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;
import org.murraybridgebunyips.bunyipslib.tasks.groups.ParallelTaskGroup;
import org.murraybridgebunyips.imposter.components.ImposterConfig;

import static org.murraybridgebunyips.bunyipslib.external.units.Units.*;

@Autonomous(name = "PoseAlignmenT", group = "VIRTUAL_BUNYIPSFTC")
public class ImposterPoseAlignmentTest extends AutonomousBunyipsOpMode implements RoadRunner {
    private final ImposterConfig config = new ImposterConfig();
    private MecanumDrive drive;

    private final PIDController forwardController = new PIDController(8, 0, 0);
    private final PIDController strafeController = new PIDController(8, 0, 0);
    private final PIDController headingController = new PIDController(10, 0, 0);

    @Override
    protected void onInitialise() {
        config.init();
        drive = new MecanumDrive(config.driveConstants, config.mecanumCoefficients, hardwareMap.voltageSensor, config.imu, config.front_left_motor, config.front_right_motor, config.back_left_motor, config.back_right_motor);
        addSubsystems(drive);
    }

    @Override
    protected void onReady(@Nullable Reference<?> selectedOpMode, Controls selectedButton) {
//        drive.setPoseEstimate(new Pose2d(41.98, 56.84, Math.toRadians(-45.0)));
//        addTask(new DriveToPoseTask(Seconds.of(5), drive, new Pose2d(47.6, 35.56, 0), forwardController, strafeController, headingController));
        addTask(new DriveToPoseTask(Task.INFINITE_TIMEOUT, drive, new Pose2d(20, 20, Math.PI / 2), forwardController, strafeController, headingController));
        addTask(new DriveToPoseTask(Task.INFINITE_TIMEOUT, drive, new Pose2d(0, 0, 3 * Math.PI / 2), forwardController, strafeController, headingController));
        addTask(new DriveToPoseTask(Task.INFINITE_TIMEOUT, drive, new Pose2d(20, -20, 0), forwardController, strafeController, headingController));
        addTask(new DriveToPoseTask(Task.INFINITE_TIMEOUT, drive, new Pose2d(0, 0, 0), forwardController, strafeController, headingController));
        addTask(new DriveToPoseTask(Task.INFINITE_TIMEOUT, drive, new Pose2d(-10, -10, 0), forwardController, strafeController, headingController));
        addTask(new DriveToPoseTask(Task.INFINITE_TIMEOUT, drive, new Pose2d(0, 0, 0), forwardController, strafeController, headingController));
        addTask(new DriveToPoseTask(Task.INFINITE_TIMEOUT, drive, new Pose2d(-10, 10, 0), forwardController, strafeController, headingController));
        addTask(new DriveToPoseTask(Task.INFINITE_TIMEOUT, drive, new Pose2d(0, 0, 0), forwardController, strafeController, headingController));
        addTask(new DriveToPoseTask(Task.INFINITE_TIMEOUT, drive, new Pose2d(-10, 0, 0), forwardController, strafeController, headingController));
        addTask(new DriveToPoseTask(Task.INFINITE_TIMEOUT, drive, new Pose2d(0, 10, 0), forwardController, strafeController, headingController));
        addTask(new DriveToPoseTask(Task.INFINITE_TIMEOUT, drive, new Pose2d(0, 0, 0), forwardController, strafeController, headingController));
        addTask(new DriveToPoseTask(Task.INFINITE_TIMEOUT, drive, new Pose2d(0, 0, 3 * Math.PI / 2), forwardController, strafeController, headingController));
        addTask(new DriveToPoseTask(Task.INFINITE_TIMEOUT, drive, new Pose2d(20, 20, Math.PI / 2), forwardController, strafeController, headingController));
        addTask(new DriveToPoseTask(Task.INFINITE_TIMEOUT, drive, new Pose2d(0, 0, 0), forwardController, strafeController, headingController));
    }

    @NonNull
    @Override
    public MecanumDrive getDrive() {
        return drive;
    }
}
