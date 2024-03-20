package org.murraybridgebunyips.bunyipslib.tasks;

import static org.murraybridgebunyips.bunyipslib.Text.round;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.Inches;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;
import org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence.TrajectorySequence;
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

/**
 * Task for running RoadRunner trajectories using the BunyipsOpMode Task system.
 * This is the task that is used for all RoadRunner tasks in RoadRunnerAutonomousBunyipsOpMode.
 *
 * @param <T> The type of RoadRunnerDrive to be used
 * @author Lucas Bubner, 2023
 */
public class RoadRunnerTask<T extends RoadRunnerDrive> extends Task {
    private final T drive;

    private Trajectory trajectory;
    private TrajectorySequence trajectorySequence;

    // Needed as the pre-check condition for task finishing will fire since the initialisation routine
    // may not have fired which assigns the drive a task
    private boolean taskStartedRunning;

    /**
     * Create a new RoadRunnerTask with a time, drive, and trajectory.
     *
     * @param time       The time to run the task for
     * @param drive      The drive to use
     * @param trajectory The trajectory to follow
     */
    public RoadRunnerTask(double time, T drive, Trajectory trajectory) {
        super(time);
        this.drive = drive;
        this.trajectory = trajectory;
    }

    /**
     * Create a new RoadRunnerTask with a time, drive, and trajectory sequence.
     *
     * @param time               The time to run the task for
     * @param drive              The drive to use
     * @param trajectorySequence The trajectory sequence to follow
     */
    public RoadRunnerTask(double time, T drive, TrajectorySequence trajectorySequence) {
        super(time);
        this.drive = drive;
        this.trajectorySequence = trajectorySequence;
    }

    /**
     * Create a new RoadRunnerTask with a time, drive, trajectory, and dependency.
     *
     * @param time                           The time to run the task for
     * @param drive                          The drive to use
     * @param trajectory                     The trajectory to follow
     * @param dependency                     The subsystem to run this task on
     * @param shouldOverrideConflictingTasks Whether this task should override conflicting tasks
     */
    public RoadRunnerTask(double time, T drive, Trajectory trajectory, BunyipsSubsystem dependency, boolean shouldOverrideConflictingTasks) {
        super(time, dependency, shouldOverrideConflictingTasks);
        this.drive = drive;
        this.trajectory = trajectory;
    }

    /**
     * Create a new RoadRunnerTask with a time, drive, trajectory sequence, and dependency.
     *
     * @param time                           The time to run the task for
     * @param drive                          The drive to use
     * @param trajectorySequence             The trajectory sequence to follow
     * @param dependency                     The subsystem to run this task on
     * @param shouldOverrideConflictingTasks Whether this task should override conflicting tasks
     */
    public RoadRunnerTask(double time, T drive, TrajectorySequence trajectorySequence, BunyipsSubsystem dependency, boolean shouldOverrideConflictingTasks) {
        super(time, dependency, shouldOverrideConflictingTasks);
        this.drive = drive;
        this.trajectorySequence = trajectorySequence;
    }

    @Override
    protected void init() {
        if (trajectory != null) {
            drive.followTrajectoryAsync(trajectory);
        } else if (trajectorySequence != null) {
            drive.followTrajectorySequenceAsync(trajectorySequence);
        } else {
            throw new NullPointerException("No trajectory or trajectory sequence was provided to the RoadRunnerTask");
        }
        taskStartedRunning = true;
    }

    @Override
    protected void periodic() {
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
        opMode.addTelemetry("Duration: %/% sec", round(getDeltaTime(), 2), round(duration, 2));
        drive.update();

        opMode.addTelemetry("Distance to target: %cm", round(Inches.toCM(distance), 2));
        opMode.addTelemetry("Angle to target: %deg", round(Math.toDegrees(angle), 2));
    }

    @Override
    protected void onFinish() {
        drive.stop();
        taskStartedRunning = false;
    }

    @Override
    protected boolean isTaskFinished() {
        return !drive.isBusy() && taskStartedRunning;
    }

    public Pose2d getEndPose() {
        if (trajectory != null) {
            return trajectory.end();
        } else {
            return trajectorySequence.end();
        }
    }
}
