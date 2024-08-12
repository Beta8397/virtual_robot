package org.murraybridgebunyips.bunyipslib.roadrunner.drive.tuning.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.murraybridgebunyips.bunyipslib.DualTelemetry;
import org.murraybridgebunyips.bunyipslib.TriConsumer;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;

import java.util.List;

/**
 * This routine is designed to tune the PID coefficients used by the REV Expansion Hubs for closed-
 * loop velocity control. Although it may seem unnecessary, tuning these coefficients is just as
 * important as the positional parameters.
 * <p>
 * Like the other manual tuning routines, this op mode
 * relies heavily upon the dashboard. To access the dashboard, connect your computer to the RC's
 * WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're using the RC
 * phone or https://192.168.43.1:8080/dash if you are using the Control Hub. Once you've successfully
 * connected, start the program, and your robot will begin moving forward and backward according to
 * a motion profile. Your job is to graph the velocity errors over time and adjust the PID
 * coefficients (note: the tuning variable will not appear until the op mode finishes initializing).
 * Once you've found a satisfactory set of gains, add them to the DriveConstants.java file under the
 * MOTOR_VELO_PID field.
 * <p>
 * Recommended tuning process:<br>
 * 1. Increase kP until any phase lag is eliminated. Concurrently increase kD as necessary to
 * mitigate oscillations.<br>
 * 2. Add kI (or adjust kF) until the steady state/constant velocity plateaus are reached.<br>
 * 3. Back off kP and kD a little until the response is less oscillatory (but without lag).
 * <p>
 * Pressing Y/Δ (Xbox/PS4) will pause the tuning process and enter driver override, allowing the
 * user to reset the position of the bot in the event that it drifts off the path.<br>
 * Pressing B/O (Xbox/PS4) will cede control back to the tuning process.
 */
public class DriveVelocityPIDTuner implements TriConsumer<LinearOpMode, DualTelemetry, RoadRunnerDrive> {
    /**
     * The distance the bot will travel back and forth.
     */
    public double DISTANCE_INCHES = 72;

    private MotionProfile generateProfile(RoadRunnerDrive drive, boolean movingForward) {
        MotionState start = new MotionState(movingForward ? 0 : DISTANCE_INCHES, 0, 0, 0);
        MotionState goal = new MotionState(movingForward ? DISTANCE_INCHES : 0, 0, 0, 0);
        return MotionProfileGenerator.generateSimpleMotionProfile(start, goal, drive.getConstants().MAX_VEL, drive.getConstants().MAX_ACCEL);
    }

    @Override
    public void accept(LinearOpMode opMode, DualTelemetry telemetry, RoadRunnerDrive drive) {
        if (!drive.getConstants().RUN_USING_ENCODER) {
            RobotLog.setGlobalErrorMsg("%s does not need to be run if the built-in motor velocity" +
                    "PID is not in use", getClass().getSimpleName());
        }

        Mode mode = Mode.TUNING_MODE;

        double lastKp = drive.getConstants().MOTOR_VELO_PID.p;
        double lastKi = drive.getConstants().MOTOR_VELO_PID.i;
        double lastKd = drive.getConstants().MOTOR_VELO_PID.d;
        double lastKf = drive.getConstants().MOTOR_VELO_PID.f;

        drive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, drive.getConstants().MOTOR_VELO_PID);

        NanoClock clock = NanoClock.system();

        telemetry.add("Ready!");
        telemetry.update();
        telemetry.clear();

        opMode.waitForStart();

        if (opMode.isStopRequested()) return;

        boolean movingForwards = true;
        MotionProfile activeProfile = generateProfile(drive, true);
        double profileStart = clock.seconds();

        while (!opMode.isStopRequested()) {
            telemetry.addDS("You must access FtcDashboard to use this tuning OpMode.");
            telemetry.addDashboard("mode", mode);

            switch (mode) {
                case TUNING_MODE:
                    if (opMode.gamepad1.y) {
                        mode = Mode.DRIVER_MODE;
                        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    }

                    // calculate and set the motor power
                    double profileTime = clock.seconds() - profileStart;

                    if (profileTime > activeProfile.duration()) {
                        // generate a new profile
                        movingForwards = !movingForwards;
                        activeProfile = generateProfile(drive, movingForwards);
                        profileStart = clock.seconds();
                    }

                    MotionState motionState = activeProfile.get(profileTime);
                    double targetPower = drive.getConstants().kV * motionState.getV();
                    drive.setDrivePower(new Pose2d(targetPower, 0, 0));

                    List<Double> velocities = drive.getWheelVelocities();

                    // update telemetry
                    telemetry.addDashboard("targetVelocity", motionState.getV());
                    for (int i = 0; i < velocities.size(); i++) {
                        telemetry.addDashboard("measuredVelocity" + i, velocities.get(i));
                        telemetry.addDashboard(
                                "error" + i,
                                motionState.getV() - velocities.get(i)
                        );
                    }
                    break;
                case DRIVER_MODE:
                    if (opMode.gamepad1.b) {
                        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                        mode = Mode.TUNING_MODE;
                        movingForwards = true;
                        activeProfile = generateProfile(drive, true);
                        profileStart = clock.seconds();
                    }

                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -opMode.gamepad1.left_stick_y,
                                    -opMode.gamepad1.left_stick_x,
                                    -opMode.gamepad1.right_stick_x
                            )
                    );
                    break;
            }

            if (lastKp != drive.getConstants().MOTOR_VELO_PID.p || lastKd != drive.getConstants().MOTOR_VELO_PID.d
                    || lastKi != drive.getConstants().MOTOR_VELO_PID.i || lastKf != drive.getConstants().MOTOR_VELO_PID.f) {
                drive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, drive.getConstants().MOTOR_VELO_PID);

                lastKp = drive.getConstants().MOTOR_VELO_PID.p;
                lastKi = drive.getConstants().MOTOR_VELO_PID.i;
                lastKd = drive.getConstants().MOTOR_VELO_PID.d;
                lastKf = drive.getConstants().MOTOR_VELO_PID.f;
            }

            telemetry.update();
        }
    }

    private enum Mode {
        DRIVER_MODE,
        TUNING_MODE
    }
}
