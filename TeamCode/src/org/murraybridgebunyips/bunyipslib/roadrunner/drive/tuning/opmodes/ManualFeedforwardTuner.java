package org.murraybridgebunyips.bunyipslib.roadrunner.drive.tuning.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.RobotLog;

import org.murraybridgebunyips.bunyipslib.DualTelemetry;
import org.murraybridgebunyips.bunyipslib.TriConsumer;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;

import java.util.Objects;

/**
 * This routine is designed to tune the open-loop feedforward coefficients. Although it may seem unnecessary,
 * tuning these coefficients is just as important as the positional parameters.
 * <p>
 * Like the other manual tuning routines, this op mode relies heavily upon the dashboard. To access the dashboard,
 * connect your computer to the RC's WiFi network. In your browser, navigate to
 * http://192.168.49.1:8080/dash if you're using the RC phone or http://192.168.43.1:8080/dash if
 * you are using the Control Hub. Once you've successfully connected, start the program, and your
 * robot will begin moving forward and backward according to a motion profile.
 * <p>
 * Your job is to graph the velocity errors over time and adjust the feedforward coefficients. Once you've found a
 * satisfactory set of gains, add them to the appropriate fields in your DriveConstants.
 * <p>
 * Pressing Y/Δ (Xbox/PS4) will pause the tuning process and enter driver override, allowing the
 * user to reset the position of the bot in the event that it drifts off the path.<br>
 * Pressing B/O (Xbox/PS4) will cede control back to the tuning process.
 */
public class ManualFeedforwardTuner implements TriConsumer<LinearOpMode, DualTelemetry, RoadRunnerDrive> {
    /**
     * The distance in inches to drive back and forth.
     */
    public double DISTANCE_INCHES = 72;
    /**
     * kV is the velocity feedforward constant.
     */
    public double kV;
    /**
     * kA is the acceleration feedforward constant.
     */
    public double kA;
    /**
     * kStatic is the static feedforward constant.
     */
    public double kStatic;

    private Mode mode;

    private MotionProfile generateProfile(RoadRunnerDrive drive, boolean movingForward) {
        MotionState start = new MotionState(movingForward ? 0 : DISTANCE_INCHES, 0, 0, 0);
        MotionState goal = new MotionState(movingForward ? DISTANCE_INCHES : 0, 0, 0, 0);
        return MotionProfileGenerator.generateSimpleMotionProfile(start, goal, drive.getConstants().MAX_VEL, drive.getConstants().MAX_ACCEL);
    }

    @Override
    public void accept(LinearOpMode opMode, DualTelemetry telemetry, RoadRunnerDrive drive) {
        if (drive.getConstants().RUN_USING_ENCODER) {
            RobotLog.setGlobalErrorMsg("Feedforward constants usually don't need to be tuned " +
                    "when using the built-in drive motor velocity PID.");
        }

        VoltageSensor voltageSensor = opMode.hardwareMap.voltageSensor.iterator().next();

        mode = Mode.TUNING_MODE;

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
                    double targetPower = Kinematics.calculateMotorFeedforward(motionState.getV(), motionState.getA(), kV, kA, kStatic);

                    final double NOMINAL_VOLTAGE = 12.0;
                    double voltage = voltageSensor.getVoltage();
                    drive.setDrivePower(new Pose2d(NOMINAL_VOLTAGE / voltage * targetPower, 0, 0));
                    drive.updatePoseEstimate();

                    Pose2d poseVelo = Objects.requireNonNull(drive.getPoseVelocity(), "poseVelocity() must not be null. Ensure that the getWheelVelocities() method has been overridden in your localizer.");
                    double currentVelo = poseVelo.getX();

                    // update telemetry
                    telemetry.addDashboard("targetVelocity", motionState.getV());
                    telemetry.addDashboard("measuredVelocity", currentVelo);
                    telemetry.addDashboard("error", motionState.getV() - currentVelo);
                    break;
                case DRIVER_MODE:
                    if (opMode.gamepad1.b) {
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

            telemetry.update();
        }
    }

    private enum Mode {
        DRIVER_MODE,
        TUNING_MODE
    }
}
