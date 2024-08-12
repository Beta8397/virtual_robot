package org.murraybridgebunyips.bunyipslib.roadrunner.drive.tuning.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.murraybridgebunyips.bunyipslib.DualTelemetry;
import org.murraybridgebunyips.bunyipslib.TriConsumer;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;

/**
 * This is a simple routine to test translational drive capabilities.
 */
public class StraightTest implements TriConsumer<LinearOpMode, DualTelemetry, RoadRunnerDrive> {
    /**
     * The amount of distance to test in inches.
     */
    public double DISTANCE_INCHES = 30;

    @Override
    public void accept(LinearOpMode opMode, DualTelemetry telemetry, RoadRunnerDrive drive) {
        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                .forward(DISTANCE_INCHES)
                .build();

        opMode.waitForStart();

        if (opMode.isStopRequested()) return;

        drive.followTrajectory(trajectory);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.add("finalX: %", poseEstimate.getX());
        telemetry.add("finalY: %", poseEstimate.getY());
        telemetry.add("finalHeading: %", poseEstimate.getHeading());
        telemetry.update();

        while (!opMode.isStopRequested() && opMode.opModeIsActive()) opMode.idle();
    }
}