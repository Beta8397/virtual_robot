package org.murraybridgebunyips.bunyipslib.roadrunner.drive.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;

import java.util.Objects;

/*
 * This routine is designed to tune the open-loop feedforward coefficients. Although it may seem unnecessary,
 * tuning these coefficients is just as important as the positional parameters. Like the other
 * manual tuning routines, this op mode relies heavily upon the dashboard. To access the dashboard,
 * connect your computer to the RC's WiFi network. In your browser, navigate to
 * https://192.168.49.1:8080/dash if you're using the RC phone or https://192.168.43.1:8080/dash if
 * you are using the Control Hub. Once you've successfully connected, start the program, and your
 * robot will begin moving forward and backward according to a motion profile. Your job is to graph
 * the velocity errors over time and adjust the feedforward coefficients. Once you've found a
 * satisfactory set of gains, add them to the appropriate fields in the DriveConstants.java file.
 *
 * Pressing Y/Δ (Xbox/PS4) will pause the tuning process and enter driver override, allowing the
 * user to reset the position of the bot in the event that it drifts off the path.
 * Pressing B/O (Xbox/PS4) will cede control back to the tuning process.
 */
//@Config
public abstract class ManualFeedforwardTuner extends LinearOpMode {
    public static double DISTANCE = 72; // in
    // Coefficients from driveConstants are able to be dynamically set in FtcDashboard
    public static double kV;
    public static double kA;
    public static double kStatic;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    protected RoadRunnerDrive drive;
    private Mode mode;

    private MotionProfile generateProfile(boolean movingForward) {
        MotionState start = new MotionState(movingForward ? 0 : DISTANCE, 0, 0, 0);
        MotionState goal = new MotionState(movingForward ? DISTANCE : 0, 0, 0, 0);
        return MotionProfileGenerator.generateSimpleMotionProfile(start, goal, drive.getConstants().MAX_VEL, drive.getConstants().MAX_ACCEL);
    }

    @Override
    public void runOpMode() {
        if (drive == null) throw new NullPointerException("drive is null!");


        if (drive.getConstants().RUN_USING_ENCODER) {
            RobotLog.setGlobalErrorMsg("Feedforward constants usually don't need to be tuned " +
                    "when using the built-in drive motor velocity PID.");
        }

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());

        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();

        mode = Mode.TUNING_MODE;

        NanoClock clock = NanoClock.system();

        telemetry.addLine("Ready!");
        telemetry.update();
        telemetry.clearAll();

        waitForStart();

        if (isStopRequested()) return;

        boolean movingForwards = true;
        MotionProfile activeProfile = generateProfile(true);
        double profileStart = clock.seconds();


        while (!isStopRequested()) {
            telemetry.addData("mode", mode);

            switch (mode) {
                case TUNING_MODE:
                    if (gamepad1.y) {
                        mode = Mode.DRIVER_MODE;
                    }

                    // calculate and set the motor power
                    double profileTime = clock.seconds() - profileStart;

                    if (profileTime > activeProfile.duration()) {
                        // generate a new profile
                        movingForwards = !movingForwards;
                        activeProfile = generateProfile(movingForwards);
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
                    telemetry.addData("targetVelocity", motionState.getV());
                    telemetry.addData("measuredVelocity", currentVelo);
                    telemetry.addData("error", motionState.getV() - currentVelo);
                    break;
                case DRIVER_MODE:
                    if (gamepad1.b) {
                        mode = Mode.TUNING_MODE;
                        movingForwards = true;
                        activeProfile = generateProfile(movingForwards);
                        profileStart = clock.seconds();
                    }

                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x,
                                    -gamepad1.right_stick_x
                            )
                    );
                    break;
            }

            telemetry.update();
        }
    }

    enum Mode {
        DRIVER_MODE,
        TUNING_MODE
    }
}