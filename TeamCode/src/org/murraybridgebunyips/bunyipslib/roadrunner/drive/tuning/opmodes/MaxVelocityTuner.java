package org.murraybridgebunyips.bunyipslib.roadrunner.drive.tuning.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.murraybridgebunyips.bunyipslib.DualTelemetry;
import org.murraybridgebunyips.bunyipslib.TriConsumer;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.DriveConstants;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;

import java.util.Objects;

/**
 * This routine is designed to calculate the maximum velocity your bot can achieve under load. It
 * will also calculate the effective kF value for your velocity PID.
 * <p>
 * Upon pressing start, your bot will run at max power for RUNTIME_SECONDS seconds.
 * <p>
 * Further fine tuning of kF may be desired.
 */
public class MaxVelocityTuner implements TriConsumer<LinearOpMode, DualTelemetry, RoadRunnerDrive> {
    /**
     * Time to run the test for (in seconds).
     */
    public double RUNTIME_SECONDS = 2.0;

    private ElapsedTime timer;
    private double maxVelocity;
    private VoltageSensor batteryVoltageSensor;

    @Override
    public void accept(LinearOpMode opMode, DualTelemetry telemetry, RoadRunnerDrive drive) {
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        batteryVoltageSensor = opMode.hardwareMap.voltageSensor.iterator().next();

        telemetry.add("Your bot will go at full speed for " + RUNTIME_SECONDS + " seconds.");
        telemetry.add("Please ensure you have enough space cleared.");
        telemetry.addNewLine();
        telemetry.add("Press start when ready.");
        telemetry.update();

        opMode.waitForStart();

        telemetry.clear();
        telemetry.update();

        drive.setDrivePower(new Pose2d(1, 0, 0));
        timer = new ElapsedTime();

        while (!opMode.isStopRequested() && timer.seconds() < RUNTIME_SECONDS) {
            drive.updatePoseEstimate();

            Pose2d poseVelo = Objects.requireNonNull(drive.getPoseVelocity(), "poseVelocity() must not be null. Ensure that the getWheelVelocities() method has been overridden in your localizer.");

            maxVelocity = Math.max(poseVelo.vec().norm(), maxVelocity);
        }

        drive.setDrivePower(new Pose2d());

        double effectiveKf = DriveConstants.getMotorVelocityF(veloInchesToTicks(drive, maxVelocity));

        telemetry.add("Max Velocity (in/s): %", maxVelocity);
        telemetry.add("Max Recommended Velocity (in/s): %", maxVelocity * 0.8);
        telemetry.add("Voltage Compensated kF: %", effectiveKf * batteryVoltageSensor.getVoltage() / 12);
        telemetry.update();

        while (!opMode.isStopRequested() && opMode.opModeIsActive()) opMode.idle();
    }

    private double veloInchesToTicks(RoadRunnerDrive drive, double inchesPerSec) {
        return inchesPerSec / (2 * Math.PI * drive.getConstants().WHEEL_RADIUS) / drive.getConstants().GEAR_RATIO * drive.getConstants().TICKS_PER_REV;
    }
}