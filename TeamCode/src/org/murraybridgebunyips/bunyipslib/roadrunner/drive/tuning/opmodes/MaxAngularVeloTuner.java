package org.murraybridgebunyips.bunyipslib.roadrunner.drive.tuning.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.murraybridgebunyips.bunyipslib.DualTelemetry;
import org.murraybridgebunyips.bunyipslib.TriConsumer;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;

import java.util.Objects;

/**
 * This routine is designed to calculate the maximum angular velocity your bot can achieve under load.
 * <p>
 * Upon pressing start, your bot will turn at max power for RUNTIME_SECONDS seconds.
 * <p>
 * Further fine tuning of MAX_ANG_VEL may be desired.
 */
public class MaxAngularVeloTuner implements TriConsumer<LinearOpMode, DualTelemetry, RoadRunnerDrive> {
    /**
     * Time to run the test for (in seconds).
     */
    public double RUNTIME_SECONDS = 4.0;

    private ElapsedTime timer;
    private double maxAngVelocity;

    @Override
    public void accept(LinearOpMode opMode, DualTelemetry telemetry, RoadRunnerDrive drive) {
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.add("Your bot will turn at full speed for " + RUNTIME_SECONDS + " seconds.");
        telemetry.add("Please ensure you have enough space cleared.");
        telemetry.addNewLine();
        telemetry.add("Press start when ready.");
        telemetry.update();

        opMode.waitForStart();

        telemetry.clear();
        telemetry.update();

        drive.setDrivePower(new Pose2d(0, 0, 1));
        timer = new ElapsedTime();

        while (!opMode.isStopRequested() && timer.seconds() < RUNTIME_SECONDS) {
            drive.updatePoseEstimate();

            Pose2d poseVelo = Objects.requireNonNull(drive.getPoseVelocity(), "poseVelocity() must not be null. Ensure that the getWheelVelocities() method has been overridden in your localizer.");

            maxAngVelocity = Math.max(poseVelo.getHeading(), maxAngVelocity);
        }

        drive.setDrivePower(new Pose2d());

        telemetry.add("Max Angular Velocity (rad): %", maxAngVelocity);
        telemetry.add("Max Angular Velocity (deg): %", Math.toDegrees(maxAngVelocity));
        telemetry.add("Max Recommended Angular Velocity (rad): %", maxAngVelocity * 0.8);
        telemetry.add("Max Recommended Angular Velocity (deg): %", Math.toDegrees(maxAngVelocity * 0.8));
        telemetry.update();

        while (!opMode.isStopRequested()) opMode.idle();
    }
}