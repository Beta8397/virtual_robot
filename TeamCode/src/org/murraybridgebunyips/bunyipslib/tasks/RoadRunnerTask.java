package org.murraybridgebunyips.bunyipslib.tasks;

import static org.murraybridgebunyips.bunyipslib.Text.round;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.murraybridgebunyips.bunyipslib.BunyipsOpMode;
import org.murraybridgebunyips.bunyipslib.Inches;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;
import org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence.TrajectorySequence;

/**
 * Task for running RoadRunner trajectories using the BunyipsOpMode Task system.
 * This is the task that is used for all RoadRunner tasks in RoadRunnerAutonomousBunyipsOpMode.
 *
 * @author Lucas Bubner, 2023
 */
public class RoadRunnerTask<T extends RoadRunnerDrive> extends Task {
    private final T drive;

    private Trajectory trajectory;
    private TrajectorySequence trajectorySequence;

    public RoadRunnerTask(@NonNull BunyipsOpMode opMode, double time, T drive, Trajectory trajectory) {
        super(opMode, time);
        this.drive = drive;
        this.trajectory = trajectory;
    }

    public RoadRunnerTask(@NonNull BunyipsOpMode opMode, double time, T drive, TrajectorySequence trajectorySequence) {
        super(opMode, time);
        this.drive = drive;
        this.trajectorySequence = trajectorySequence;
    }

    @Override
    public void init() {
        if (trajectory != null) {
            drive.followTrajectoryAsync(trajectory);
        } else if (trajectorySequence != null) {
            drive.followTrajectorySequenceAsync(trajectorySequence);
        } else {
            throw new NullPointerException("No trajectory or trajectory sequence was provided to the RoadRunnerTask");
        }
    }

    @Override
    public void run() {
        Pose2d endPose;
        double duration;
        if (trajectory != null) {
            endPose = trajectory.end();
            duration = trajectory.duration();
        } else {
            endPose = trajectorySequence.end();
            duration = trajectorySequence.duration();
        }

        // Calculate distance from current pose to end pose using the distance formula
        double distance = Math.sqrt(Math.pow(endPose.getX() - drive.getPoseEstimate().getX(), 2) + Math.pow(endPose.getY() - drive.getPoseEstimate().getY(), 2));

        // Calculate angle from current pose to end pose using the arctangent function
        double angle = Math.atan2(endPose.getY() - drive.getPoseEstimate().getY(), endPose.getX() - drive.getPoseEstimate().getX());

        // Time to completion
        getOpMode().addTelemetry("Duration: %/% sec", round(getDeltaTime(), 2), round(duration, 2));
        drive.update();

        getOpMode().addTelemetry("Distance to target: %cm", round(Inches.toCM(distance), 2));
        getOpMode().addTelemetry("Angle to target: %deg", round(Math.toDegrees(angle), 2));
    }

    @Override
    public void onFinish() {
        drive.stop();
    }

    @Override
    public boolean isTaskFinished() {
        return !drive.isBusy();
    }
}
