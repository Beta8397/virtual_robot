package org.firstinspires.ftc.teamcode.common.tasks;

import androidx.annotation.NonNull;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import org.firstinspires.ftc.teamcode.common.BunyipsOpMode;
import org.firstinspires.ftc.teamcode.common.Mecanum;
import org.firstinspires.ftc.teamcode.common.roadrunner.trajectorysequence.TrajectorySequence;

/**
 * Task for running RoadRunner trajectories using the BunyipsOpMode Task system
 *
 * @author Lucas Bubner, 2023
 */
public class RoadRunnerTask extends Task {
    private final Mecanum drive;

    private Trajectory trajectory;
    private TrajectorySequence trajectorySequence;

    public RoadRunnerTask(@NonNull BunyipsOpMode opMode, double time, Mecanum drive, Trajectory trajectory) {
        super(opMode, time);
        this.drive = drive;
        this.trajectory = trajectory;
    }

    public RoadRunnerTask(@NonNull BunyipsOpMode opMode, double time, Mecanum drive, TrajectorySequence trajectorySequence) {
        super(opMode, time);
        this.drive = drive;
        this.trajectorySequence = trajectorySequence;
    }

    @Override
    public void init() {
        if (trajectory != null) {
            drive.followTrajectory(trajectory);
        } else if (trajectorySequence != null) {
            drive.followTrajectorySequence(trajectorySequence);
        } else {
            throw new NullPointerException("No trajectory or trajectory sequence was provided to the RoadRunnerTask");
        }
    }

    @Override
    public void run() {
        drive.update();
    }

    @Override
    public void onFinish() {
        // noop
    }

    @Override
    public boolean isTaskFinished() {
        return !drive.isBusy();
    }
}
