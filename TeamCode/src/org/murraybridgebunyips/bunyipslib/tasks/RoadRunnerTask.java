package org.murraybridgebunyips.bunyipslib.tasks;

import static org.murraybridgebunyips.bunyipslib.Text.formatString;
import static org.murraybridgebunyips.bunyipslib.Text.round;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.Centimeters;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.Inches;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.Seconds;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;

import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.external.units.Measure;
import org.murraybridgebunyips.bunyipslib.external.units.Time;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;
import org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence.TrajectorySequence;
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

/**
 * Task for running RoadRunner trajectories using the BunyipsOpMode Task system.
 * This is the task that is used for all RoadRunner tasks in the RoadRunner interface.
 *
 * @author Lucas Bubner, 2023
 */
public class RoadRunnerTask extends Task {
    private final RoadRunnerDrive drive;

    private Trajectory trajectory;
    private TrajectorySequence trajectorySequence;

    // Needed as the pre-check condition for task finishing will fire since the initialisation routine
    // may not have fired which assigns the drive a task
    private boolean taskStartedRunning;

    /**
     * Create a new RoadRunnerTask with a time, drive, and trajectory.
     *
     * @param time       The time to run the task for
     * @param drive      The drive to use. Will be casted to a BunyipsSubsystem and used as the dependency if possible.
     * @param trajectory The trajectory to follow
     */
    public RoadRunnerTask(Measure<Time> time, RoadRunnerDrive drive, Trajectory trajectory) {
        super(time.magnitude() != 0.0 ? time : Seconds.of(trajectory.duration()));
        this.drive = drive;
        this.trajectory = trajectory;
        withName(formatString("RoadRunner Trajectory %::%", trajectory.start(), trajectory.end()));
        if (drive instanceof BunyipsSubsystem)
            onSubsystem((BunyipsSubsystem) drive, true);
    }

    /**
     * Create a new RoadRunnerTask with a time, drive, and trajectory sequence.
     *
     * @param time               The time to run the task for
     * @param drive              The drive to use. Will be casted to a BunyipsSubsystem and used as the dependency if possible.
     * @param trajectorySequence The trajectory sequence to follow
     */
    public RoadRunnerTask(Measure<Time> time, RoadRunnerDrive drive, TrajectorySequence trajectorySequence) {
        super(time.magnitude() != 0.0 ? time : Seconds.of(trajectorySequence.duration()));
        this.drive = drive;
        this.trajectorySequence = trajectorySequence;
        withName(formatString("RoadRunner Trajectory %::%", trajectorySequence.start(), trajectorySequence.end()));
        if (drive instanceof BunyipsSubsystem)
            onSubsystem((BunyipsSubsystem) drive, true);
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

        // Angle formula: tan^-1(y/x), using right angle trigonometry and offsetting by the robot rotation to get
        // at what angle we are "facing" the target, negating as pose angles are opposite and similar to the unit circle
        double angle = -Angle.normDelta(Math.atan2(endPose.getY() - drive.getPoseEstimate().getY(), endPose.getX() - drive.getPoseEstimate().getX()) - drive.getPoseEstimate().getHeading());
        // Distance formula: sqrt((x2 - x1)^2 + (y2 - y1)^2)
        double distance = Math.sqrt(Math.pow(endPose.getX() - drive.getPoseEstimate().getX(), 2) + Math.pow(endPose.getY() - drive.getPoseEstimate().getY(), 2));

        // Time to completion
        opMode.telemetry.add("Duration: %/% sec", round(getDeltaTime().in(Seconds), 2), round(duration, 2));
        opMode.telemetry.add("Distance to target: %cm", round(Centimeters.convertFrom(distance, Inches), 2));
        opMode.telemetry.add("Angle to target: %deg", round(Math.toDegrees(angle), 2));
    }

    @Override
    protected void onFinish() {
        taskStartedRunning = false;
        drive.stop();
    }

    @Override
    protected void onInterrupt() {
        drive.cancelTrajectory();
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
