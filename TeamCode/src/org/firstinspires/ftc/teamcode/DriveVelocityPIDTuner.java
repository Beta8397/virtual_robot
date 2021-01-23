package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.jetbrains.annotations.NotNull;
import system.config.StandAlone;
import system.robot.BaseAutonomous;
import system.robot.MainRobot;
import util.control.Button;
import util.control.CustomizableGamepad;
import util.math.geometry.Vector2D;

import java.util.List;

@StandAlone
@Autonomous(name = "Drive Velocity PID Tuner")
public class DriveVelocityPIDTuner extends BaseAutonomous {
    public @MainRobot RoadrunnerCalibBot robot;

    private static final String SWITCH_MODE = "swtich mode";
    private static final double DISTANCE = 72; // in

    private static final Button<Boolean> SWITCH_MODE_BUTTON = new Button<>(1, Button.BooleanInputs.a);

    private CustomizableGamepad gamepad;

    private Mode mode;
    private double lastKp, lastKi, lastKd, lastKf;
    private NanoClock clock;

    private enum Mode {
        DRIVER_MODE,
        TUNING_MODE
    }

    @NotNull
    private MotionProfile generateProfile(boolean movingForward) {
        MotionState start = new MotionState(movingForward ? 0 : DISTANCE, 0, 0, 0);
        MotionState goal = new MotionState(movingForward ? DISTANCE : 0, 0, 0, 0);
        return MotionProfileGenerator.generateSimpleMotionProfile(start, goal, robot.drive.rrConfig.MAX_VEL, robot.drive.rrConfig.MAX_ACCEL);
    }

    @Override
    protected void onInit() {
        if(!robot.drive.rrConfig.USE_DRIVE_ENCODERS) {
            robot.telemetry.addData("", "DriveVelocityPIDTuner does not need to be run if the built-in motor velocity PID is not in use.");
        }

        mode = Mode.TUNING_MODE;

        lastKp = robot.drive.rrConfig.MOTOR_VELO_PID.p;
        lastKi = robot.drive.rrConfig.MOTOR_VELO_PID.i;
        lastKd = robot.drive.rrConfig.MOTOR_VELO_PID.d;
        lastKf = robot.drive.rrConfig.MOTOR_VELO_PID.f;

        robot.drive.setMotorPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, robot.drive.rrConfig.MOTOR_VELO_PID);

        clock = NanoClock.system();

        gamepad = new CustomizableGamepad(robot);
        gamepad.addButton(SWITCH_MODE, SWITCH_MODE_BUTTON);

        robot.telemetry.addData("","Ready!");
        robot.telemetry.update();
    }

    @Override
    public void main() {
        boolean movingForwards = true;
        MotionProfile activeProfile = generateProfile(true);
        double profileStart = clock.seconds();

        while(opModeIsActive()) {
            telemetry.addData("Mode", mode);

            switch (mode) {
                case TUNING_MODE:
                    if (gamepad.getInput(SWITCH_MODE)) {
                        mode = Mode.DRIVER_MODE;
                        robot.drive.setAllMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
                    double targetPower = robot.drive.rrConfig.kV * motionState.getV();
                    robot.drive.movePower(new Vector2D(0, targetPower));

                    List<Double> velocities = robot.drive.getWheelVelocities();

                    // update telemetry
                    telemetry.addData("targetVelocity", motionState.getV());
                    for (int i = 0; i < velocities.size(); i++) {
                        telemetry.addData("measuredVelocity" + i, velocities.get(i));
                        telemetry.addData(
                                "error" + i,
                                motionState.getV() - velocities.get(i)
                        );
                    }
                    break;
                case DRIVER_MODE:
                    if (gamepad.getInput(SWITCH_MODE)) {
                        robot.drive.setAllMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);

                        mode = Mode.TUNING_MODE;
                        movingForwards = true;
                        activeProfile = generateProfile(true);
                        profileStart = clock.seconds();
                    }

                    robot.drive.handle();
                    break;
            }

            if (lastKp != robot.drive.rrConfig.MOTOR_VELO_PID.p || lastKd != robot.drive.rrConfig.MOTOR_VELO_PID.d
                    || lastKi != robot.drive.rrConfig.MOTOR_VELO_PID.i || lastKf != robot.drive.rrConfig.MOTOR_VELO_PID.f) {
                robot.drive.setMotorPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, robot.drive.rrConfig.MOTOR_VELO_PID);

                lastKp = robot.drive.rrConfig.MOTOR_VELO_PID.p;
                lastKi = robot.drive.rrConfig.MOTOR_VELO_PID.i;
                lastKd = robot.drive.rrConfig.MOTOR_VELO_PID.d;
                lastKf = robot.drive.rrConfig.MOTOR_VELO_PID.f;
            }

            robot.telemetry.update();
        }
    }
}
