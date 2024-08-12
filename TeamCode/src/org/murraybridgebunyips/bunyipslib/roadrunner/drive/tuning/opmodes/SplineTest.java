package org.murraybridgebunyips.bunyipslib.roadrunner.drive.tuning.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.murraybridgebunyips.bunyipslib.DualTelemetry;
import org.murraybridgebunyips.bunyipslib.TriConsumer;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;

/**
 * This is an example of a more complex path to really test the tuning.
 */
public class SplineTest implements TriConsumer<LinearOpMode, DualTelemetry, RoadRunnerDrive> {
    @Override
    public void accept(LinearOpMode opMode, DualTelemetry telemetry, RoadRunnerDrive drive) {
        opMode.waitForStart();

        if (opMode.isStopRequested()) return;

        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(60, 60), 0)
                .build();

        drive.followTrajectory(traj);

        opMode.sleep(2000);

        drive.followTrajectory(
                drive.trajectoryBuilder(traj.end(), true)
                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                        .build()
        );
    }
}