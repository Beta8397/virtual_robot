package org.murraybridgebunyips.bunyipslib.tasks;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.external.pid.PIDController;
import org.murraybridgebunyips.bunyipslib.external.units.Measure;
import org.murraybridgebunyips.bunyipslib.external.units.Time;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

/**
 * Drive to a pose using RoadRunner.
 * This is useful for pose alignment based on error, rather than using trajectories for planned motion.
 * Designed and tested for Mecanum drivebases as RoadRunner was designed around them.
 *
 * @author Lucas Bubner, 2024
 */
public class DriveToPoseTask extends Task {
    private final RoadRunnerDrive drive;
    private final Pose2d targetPose;
    private final PIDController forwardController;
    private final PIDController strafeController;
    private final PIDController headingController;

    /**
     * Run the Drive To Pose Task on a drive subsystem.
     *
     * @param timeout The maximum time the task can run for.
     * @param driveSubsystem The drive subsystem to run the task on.
     * @param targetPose The target pose to drive to.
     * @param forwardController The PID controller for x.
     * @param strafeController The PID controller for y.
     * @param headingController The PID controller for heading.
     */
    public DriveToPoseTask(@NonNull Measure<Time> timeout, @NonNull BunyipsSubsystem driveSubsystem,
                           Pose2d targetPose, PIDController forwardController, PIDController strafeController, PIDController headingController) {
        super(timeout, driveSubsystem, true);
        if (!(driveSubsystem instanceof RoadRunnerDrive))
            throw new IllegalArgumentException("DriveToPoseTask requires a RoadRunnerDrive subsystem");
        drive = (RoadRunnerDrive) driveSubsystem;
        this.targetPose = targetPose;
        this.forwardController = forwardController;
        this.strafeController = strafeController;
        this.headingController = headingController;
        withName("Drive To Pose: " + targetPose.toString());
    }

    @Override
    protected void init() {
        drive.cancelTrajectory();
        drive.stop();
    }

    @Override
    protected void periodic() {
        Pose2d error = targetPose.minus(drive.getPoseEstimate());
        drive.setWeightedDrivePower(new Pose2d(
                forwardController.calculate(error.getX()),
                strafeController.calculate(error.getY()),
                headingController.calculate(error.getHeading())
        ));
    }

    @Override
    protected void onFinish() {
        drive.stop();
    }

    @Override
    protected boolean isTaskFinished() {
        return forwardController.atSetPoint() && strafeController.atSetPoint() && headingController.atSetPoint();
    }
}
