package org.murraybridgebunyips.bunyipslib.roadrunner.drive.tuning.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.murraybridgebunyips.bunyipslib.DualTelemetry;
import org.murraybridgebunyips.bunyipslib.TriConsumer;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;
import org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence.TrajectorySequence;

/**
 * Op mode for preliminary tuning of the follower PID coefficients (located in the drive base
 * classes). The robot drives in a DISTANCE_INCHES-by-DISTANCE_INCHES square indefinitely.
 * <p>
 * Utilization of the dashboard is recommended for this tuning routine. To access the dashboard, connect your computer
 * to the RC's WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're
 * using the RC phone or https://192.168.43.1:8080/dash if you are using the Control Hub. Once
 * you've successfully connected, start the program, and your robot will begin driving in a square.
 * You should observe the target position (green) and your pose estimate (blue) and adjust your
 * follower PID coefficients such that you follow the target position as accurately as possible.
 * <p>
 * If you are using MecanumDrive, you should be tuning TRANSLATIONAL_PID and HEADING_PID.<br>
 * If you are using TankDrive, you should be tuning AXIAL_PID, CROSS_TRACK_PID, and HEADING_PID.
 * <p>
 * These coefficients can be tuned live in dashboard.
 */
public class FollowerPIDTuner implements TriConsumer<LinearOpMode, DualTelemetry, RoadRunnerDrive> {
    /**
     * The amount of distance the robot drives in a square.
     */
    public double DISTANCE_INCHES = 48;

    @Override
    public void accept(LinearOpMode opMode, DualTelemetry telemetry, RoadRunnerDrive drive) {
        Pose2d startPose = new Pose2d(-DISTANCE_INCHES / 2, -DISTANCE_INCHES / 2, 0);
        drive.setPoseEstimate(startPose);

        opMode.waitForStart();

        if (opMode.isStopRequested()) return;
        while (!opMode.isStopRequested()) {
            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                    .forward(DISTANCE_INCHES)
                    .turn(Math.toRadians(90))
                    .forward(DISTANCE_INCHES)
                    .turn(Math.toRadians(90))
                    .forward(DISTANCE_INCHES)
                    .turn(Math.toRadians(90))
                    .forward(DISTANCE_INCHES)
                    .turn(Math.toRadians(90))
                    .build();
            drive.followTrajectorySequence(trajSeq);
        }
    }
}
