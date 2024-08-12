package org.murraybridgebunyips.bunyipslib.roadrunner.drive.tuning.opmodes;

import static org.murraybridgebunyips.bunyipslib.Text.round;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.murraybridgebunyips.bunyipslib.DualTelemetry;
import org.murraybridgebunyips.bunyipslib.TriConsumer;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;
import org.murraybridgebunyips.bunyipslib.roadrunner.util.RegressionUtil;
import org.murraybridgebunyips.deps.LoggingUtil;

import java.util.ArrayList;
import java.util.List;

/**
 * Op mode for computing kV, kStatic, and kA from various drive routines. For the curious, here's an
 * outline of the procedure:<br>
 * 1. Slowly ramp the motor power and record encoder values along the way.<br>
 * 2. Run a linear regression on the encoder velocity vs. motor power plot to obtain a slope (kV)
 * and an optional intercept (kStatic).<br>
 * 3. Accelerate the robot (apply constant power) and record the encoder counts.<br>
 * 4. Adjust the encoder data based on the velocity tuning data and find kA with another linear
 * regression.
 */
public class AutomaticFeedforwardTuner implements TriConsumer<LinearOpMode, DualTelemetry, RoadRunnerDrive> {
    /**
     * The maximum power for the feedforward tuning routine.
     */
    public double MAX_POWER = 0.7;
    /**
     * The distance in inches to travel for the feedforward tuning routine.
     */
    public double DISTANCE_INCHES = 100;

    @Override
    public void accept(LinearOpMode opMode, DualTelemetry telemetry, RoadRunnerDrive drive) {
        if (drive.getConstants().RUN_USING_ENCODER) {
            RobotLog.setGlobalErrorMsg("Feedforward constants usually don't need to be tuned " +
                    "when using the built-in drive motor velocity PID.");
        }

        NanoClock clock = NanoClock.system();

        telemetry.add("Press play to begin the feedforward tuning routine");
        telemetry.update();

        opMode.waitForStart();

        if (opMode.isStopRequested()) return;

        telemetry.clear();
        telemetry.add("Would you like to fit kStatic?");
        telemetry.add("Press (Y/Δ) for yes, (B/O) for no");
        telemetry.update();

        boolean fitIntercept = false;
        while (!opMode.isStopRequested()) {
            if (opMode.gamepad1.y) {
                fitIntercept = true;
                while (!opMode.isStopRequested() && opMode.gamepad1.y) {
                    opMode.idle();
                }
                break;
            } else if (opMode.gamepad1.b) {
                while (!opMode.isStopRequested() && opMode.gamepad1.b) {
                    opMode.idle();
                }
                break;
            }
            opMode.idle();
        }

        telemetry.clear();
        telemetry.add("Place your robot on the field with at least % in of room (inches) in front", round(DISTANCE_INCHES, 2));
        telemetry.add("Press (Y/Δ) to begin");
        telemetry.update();

        while (!opMode.isStopRequested() && !opMode.gamepad1.y) {
            opMode.idle();
        }
        while (!opMode.isStopRequested() && opMode.gamepad1.y) {
            opMode.idle();
        }

        telemetry.clear();
        telemetry.add("Running...");
        telemetry.update();

        double maxVel = drive.getConstants().rpmToVelocity(drive.getConstants().MAX_RPM);
        double finalVel = MAX_POWER * maxVel;
        double accel = (finalVel * finalVel) / (2.0 * DISTANCE_INCHES);
        double rampTime = Math.sqrt(2.0 * DISTANCE_INCHES / accel);

        List<Double> timeSamples = new ArrayList<>();
        List<Double> positionSamples = new ArrayList<>();
        List<Double> powerSamples = new ArrayList<>();

        drive.setPoseEstimate(new Pose2d());

        double startTime = clock.seconds();
        while (!opMode.isStopRequested()) {
            double elapsedTime = clock.seconds() - startTime;
            if (elapsedTime > rampTime) {
                break;
            }
            double vel = accel * elapsedTime;
            double power = vel / maxVel;

            timeSamples.add(elapsedTime);
            positionSamples.add(drive.getPoseEstimate().getX());
            powerSamples.add(power);

            drive.setDrivePower(new Pose2d(power, 0.0, 0.0));
            drive.updatePoseEstimate();
        }
        drive.setDrivePower(new Pose2d(0.0, 0.0, 0.0));

        RegressionUtil.RampResult rampResult = RegressionUtil.fitRampData(
                timeSamples, positionSamples, powerSamples, fitIntercept,
                LoggingUtil.getLogFile(Misc.formatInvariant(
                        "DriveRampRegression-%d.csv", System.currentTimeMillis())));

        telemetry.clear();
        telemetry.add("Quasi-static ramp up test complete");
        if (fitIntercept) {
            telemetry.add("kV = %, kStatic = % (R^2 = %)",
                    round(rampResult.kV, 5), round(rampResult.kStatic, 5), round(rampResult.rSquare, 2));
        } else {
            telemetry.add("kV = % (R^2 = %)", round(rampResult.kStatic, 5), round(rampResult.rSquare, 2));
        }
        telemetry.add("Would you like to fit kA?");
        telemetry.add("Press (Y/Δ) for yes, (B/O) for no");
        telemetry.update();

        boolean fitAccelFF = false;
        while (!opMode.isStopRequested()) {
            if (opMode.gamepad1.y) {
                fitAccelFF = true;
                while (!opMode.isStopRequested() && opMode.gamepad1.y) {
                    opMode.idle();
                }
                break;
            } else if (opMode.gamepad1.b) {
                while (!opMode.isStopRequested() && opMode.gamepad1.b) {
                    opMode.idle();
                }
                break;
            }
            opMode.idle();
        }

        if (fitAccelFF) {
            telemetry.clear();
            telemetry.add("Place the robot back in its starting position");
            telemetry.add("Press (Y/Δ) to continue");
            telemetry.update();

            while (!opMode.isStopRequested() && !opMode.gamepad1.y) {
                opMode.idle();
            }
            while (!opMode.isStopRequested() && opMode.gamepad1.y) {
                opMode.idle();
            }

            telemetry.clear();
            telemetry.add("Running...");
            telemetry.update();

            double maxPowerTime = DISTANCE_INCHES / maxVel;

            timeSamples.clear();
            positionSamples.clear();
            powerSamples.clear();

            drive.setPoseEstimate(new Pose2d());
            drive.setDrivePower(new Pose2d(MAX_POWER, 0.0, 0.0));

            startTime = clock.seconds();
            while (!opMode.isStopRequested()) {
                double elapsedTime = clock.seconds() - startTime;
                if (elapsedTime > maxPowerTime) {
                    break;
                }

                timeSamples.add(elapsedTime);
                positionSamples.add(drive.getPoseEstimate().getX());
                powerSamples.add(MAX_POWER);

                drive.updatePoseEstimate();
            }
            drive.setDrivePower(new Pose2d(0.0, 0.0, 0.0));

            RegressionUtil.AccelResult accelResult = RegressionUtil.fitAccelData(
                    timeSamples, positionSamples, powerSamples, rampResult,
                    LoggingUtil.getLogFile(Misc.formatInvariant(
                            "DriveAccelRegression-%d.csv", System.currentTimeMillis())));

            telemetry.clear();
            telemetry.add("Constant power test complete");
            telemetry.add("kA = % (R^2 = %)",
                    round(accelResult.kA, 5), round(accelResult.rSquare, 2));
            telemetry.update();
        }

        while (!opMode.isStopRequested()) {
            opMode.idle();
        }
    }
}
