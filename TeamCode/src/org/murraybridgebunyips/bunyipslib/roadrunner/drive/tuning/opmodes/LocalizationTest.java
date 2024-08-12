package org.murraybridgebunyips.bunyipslib.roadrunner.drive.tuning.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.murraybridgebunyips.bunyipslib.DualTelemetry;
import org.murraybridgebunyips.bunyipslib.TriConsumer;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions).
 * <p>
 * The goal of this exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
public class LocalizationTest implements TriConsumer<LinearOpMode, DualTelemetry, RoadRunnerDrive> {
    @Override
    public void accept(LinearOpMode opMode, DualTelemetry telemetry, RoadRunnerDrive drive) {
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        opMode.waitForStart();

        while (!opMode.isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -opMode.gamepad1.left_stick_y,
                            -opMode.gamepad1.left_stick_x,
                            -opMode.gamepad1.right_stick_x
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addDS("x: %", poseEstimate.getX());
            telemetry.addDS("y: %", poseEstimate.getY());
            telemetry.addDS("heading: %", poseEstimate.getHeading());
            telemetry.addDashboard("x", poseEstimate.getX());
            telemetry.addDashboard("y", poseEstimate.getY());
            telemetry.addDashboard("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
