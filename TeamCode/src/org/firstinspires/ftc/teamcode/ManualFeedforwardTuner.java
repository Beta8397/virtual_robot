package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import org.jetbrains.annotations.NotNull;
import system.robot.BaseAutonomous;
import system.robot.MainRobot;
import util.control.Button;
import util.control.CustomizableGamepad;
import util.math.geometry.Vector2D;

import java.util.Objects;


public class ManualFeedforwardTuner extends BaseAutonomous {
    public @MainRobot RoadrunnerCalibBot robot;

    private static final String SWITCH_MODE = "swtich mode";
    private static final Button<Boolean> SWITCH_MODE_BUTTON = new Button<>(1, Button.BooleanInputs.a);
    private CustomizableGamepad gamepad;


    public static final double DISTANCE = 72; // in
    private NanoClock clock;

    enum Mode {
        DRIVER_MODE,
        TUNING_MODE
    }

    private Mode mode;

    @NotNull
    private MotionProfile generateProfile(boolean movingForward) {
        MotionState start = new MotionState(movingForward ? 0 : DISTANCE, 0, 0, 0);
        MotionState goal = new MotionState(movingForward ? DISTANCE : 0, 0, 0, 0);
        return MotionProfileGenerator.generateSimpleMotionProfile(start, goal, robot.drive.rrConfig.MAX_VEL, robot.drive.rrConfig.MAX_ACCEL);
    }

    @Override
    protected void onInit() {
        if (robot.drive.rrConfig.USE_DRIVE_ENCODERS) {
            robot.telemetry.addData("", "Feedforward constants usually don't need to be tuned when using the built-in drive motor velocity PID.");
            robot.telemetry.update();
        }

        mode = Mode.TUNING_MODE;
        clock = NanoClock.system();

        gamepad = new CustomizableGamepad(robot);
        gamepad.addButton(SWITCH_MODE, SWITCH_MODE_BUTTON);

        telemetry.addData("", "Ready!");
        telemetry.update();
    }


    @Override
    public void main() {
        boolean movingForwards = true;
        MotionProfile activeProfile = generateProfile(true);
        double profileStart = clock.seconds();

        while (!isStopRequested()) {
            telemetry.addData("mode", mode);

            switch (mode) {
                case TUNING_MODE:
                    if (gamepad.getInput(SWITCH_MODE_BUTTON)) {
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
                    double targetPower = Kinematics.calculateMotorFeedforward(motionState.getV(), motionState.getA(), robot.drive.rrConfig.kV, robot.drive.rrConfig.kA, robot.drive.rrConfig.kStatic);

                    robot.drive.movePower(new Vector2D(0, targetPower));
                    robot.drive.getLocalizer().update();

                    Pose2d poseVelo = Objects.requireNonNull(robot.drive.getLocalizer().getPoseVelocity(), "poseVelocity() must not be null. Ensure that the getWheelVelocities() method has been overridden in your localizer.");
                    double currentVelo = poseVelo.getY();

                    // update telemetry
                    telemetry.addData("targetVelocity", motionState.getV());
                    telemetry.addData("measuredVelocity", currentVelo);
                    telemetry.addData("error", motionState.getV() - currentVelo);
                    break;
                case DRIVER_MODE:
                    if (gamepad.getInput(SWITCH_MODE_BUTTON)) {
                        mode = Mode.TUNING_MODE;
                        movingForwards = true;
                        activeProfile = generateProfile(true);
                        profileStart = clock.seconds();
                    }

                    robot.drive.handle();
                    break;
            }

            telemetry.update();
        }
    }
}
