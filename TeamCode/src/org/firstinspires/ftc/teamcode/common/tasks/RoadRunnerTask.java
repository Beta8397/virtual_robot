package org.firstinspires.ftc.teamcode.common.tasks;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.common.BunyipsOpMode;
import org.firstinspires.ftc.teamcode.common.roadrunner.drive.RoadRunnerDrive;
import org.firstinspires.ftc.teamcode.common.roadrunner.trajectorysequence.TrajectorySequence;

/**
 * Task for running RoadRunner trajectories using the BunyipsOpMode Task system
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
        drive.update();
        // TODO: Telemetry for RoadRunner progress other than the default telemetry
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
