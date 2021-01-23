package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import system.config.StandAlone;
import system.robot.BaseAutonomous;
import system.robot.MainRobot;
import system.robot.roadrunner_util.LoggingUtil;
import system.robot.roadrunner_util.RegressionUtil;
import util.control.Button;
import util.control.CustomizableGamepad;
import util.control.Toggle;
import util.math.geometry.Vector2D;

import java.util.ArrayList;
import java.util.List;

@StandAlone
@Autonomous(name = "Automatic Feed Forward Tuner")
public class AutomaticFeedforwardTuner extends BaseAutonomous {

    private static final String YES = "yes", NO = "no";
    private static final Toggle
            yesToggle = new Toggle(Toggle.ToggleTypes.trueOnceToggle, false),
            noToggle = new Toggle(Toggle.ToggleTypes.trueOnceToggle, false);
    private final NanoClock clock = NanoClock.system();

    private CustomizableGamepad gamepad;
    public @MainRobot RoadrunnerCalibBot robot;

    public boolean fitKStatic = false, fitKa = false, fitKv = false;

    public static double MAX_POWER = 0.7;
    public static double DISTANCE = 75; // in
    public static long SIMULATED_SAMPLE_TIME_MS = 250;

    @Override
    protected void onInit() {
        gamepad = new CustomizableGamepad(robot);
        gamepad.addButton(YES, new Button<>(1, Button.BooleanInputs.a));
        gamepad.addButton(NO, new Button<>(1, Button.BooleanInputs.b));

        if(robot.drive.rrConfig.USE_DRIVE_ENCODERS) {
            robot.telemetry.addData("", "Feedforward constants usually don't need to be tuned when using the built-in drive motor velocity PID.");
            robot.telemetry.update();
        }
    }

    @Override
    public void main() {
        robot.telemetry.addData("","Press (A) to begin the feedforward tuning routine");
        robot.telemetry.update();

        waitUntil(() -> {
            yesToggle.updateToggle(gamepad.getInput(YES));
            return yesToggle.getCurrentState();
        });

        telemetry.addData("","Would you like to fit kStatic?");
        telemetry.addData("","Press (A) for yes, (B) for no");
        telemetry.update();

        waitUntil(() -> {
            yesToggle.updateToggle(gamepad.getInput(YES));
            noToggle.updateToggle(gamepad.getInput(NO));

            fitKStatic = yesToggle.getCurrentState();
            return fitKStatic || noToggle.getCurrentState();
        });

        telemetry.addData("","Place your robot on the field with at least "+DISTANCE+" inches of room in front");
        telemetry.addData("","Press (A) to begin");
        telemetry.update();

        waitUntil(() -> {
            yesToggle.updateToggle(gamepad.getInput(YES));
            return yesToggle.getCurrentState();
        });

        telemetry.addData("","Running...");
        telemetry.update();

        double maxVel = robot.drive.driveConfig.rpmToVelocity(robot.drive.driveConfig.MAX_RPM);
        double finalVel = MAX_POWER * maxVel;
        double accel = (finalVel * finalVel) / (2.0 * DISTANCE);
        double rampTime = Math.sqrt(2.0 * DISTANCE / accel);

        List<Double> timeSamples = new ArrayList<>();
        List<Double> positionSamples = new ArrayList<>();
        List<Double> powerSamples = new ArrayList<>();

        robot.drive.setPoseEstimate(new Pose2d());

        double startTime = clock.seconds();
        while (!robot.isStopRequested()) {
            double elapsedTime = clock.seconds() - startTime;
            if (elapsedTime > rampTime) {
                break;
            }
            double vel = accel * elapsedTime;
            double power = vel / maxVel;

            timeSamples.add(elapsedTime);
            positionSamples.add(robot.drive.getPoseEstimate().getY());
            powerSamples.add(power);

            robot.drive.movePower(new Vector2D(0, power));
            robot.drive.updateLocalizer();

            waitTime(SIMULATED_SAMPLE_TIME_MS);
        }
        robot.drive.stopAllMotors();

        RegressionUtil.RampResult rampResult = RegressionUtil.fitRampData(
                timeSamples, positionSamples, powerSamples, fitKStatic,
                LoggingUtil.getLogFile("DriveRampRegression-" + System.currentTimeMillis() + ".csv"));

        if(fitKStatic) {
            robot.telemetry.addData("", "Quasi-static ramp up test complete");
            robot.telemetry.addData("", "kV = " + rampResult.kV + ", kStatic = " + rampResult.kStatic + " (R^2 = " + rampResult.rSquare + ")");
        }
        else {
            robot.telemetry.addData("","kV = "+ rampResult.kStatic+" (R^2 = "+ rampResult.rSquare+")");
        }
        robot.telemetry.addData("","Press (A) to continue.");
        robot.telemetry.update();
        waitUntil(() -> {
            yesToggle.updateToggle(gamepad.getInput(YES));
            return yesToggle.getCurrentState();
        });

        telemetry.addData("","Would you like to fit kA?");
        telemetry.addData("","Press (A) for yes, (B) for no");
        telemetry.update();

        waitUntil(() -> {
            yesToggle.updateToggle(gamepad.getInput(YES));
            noToggle.updateToggle(gamepad.getInput(NO));

            fitKa = yesToggle.getCurrentState();
            return fitKa || noToggle.getCurrentState();
        });

        robot.telemetry.addData("", "Place the robot back in its starting position");
        robot.telemetry.addData("", "Press (A) to continue.");
        robot.telemetry.update();

        waitUntil(() -> {
            yesToggle.updateToggle(gamepad.getInput(YES));
            return yesToggle.getCurrentState();
        });

        telemetry.addData("", "Running...");
        telemetry.update();


        double maxPowerTime = DISTANCE / maxVel;

        timeSamples.clear();
        positionSamples.clear();
        powerSamples.clear();

        robot.drive.setPoseEstimate(new Pose2d());
        robot.drive.movePower(new Vector2D(0, MAX_POWER));

        startTime = clock.seconds();
        while (!isStopRequested()) {
            double elapsedTime = clock.seconds() - startTime;
            if (elapsedTime > maxPowerTime) {
                break;
            }

            System.out.println(robot.drive.getPoseEstimate());

            timeSamples.add(elapsedTime);
            positionSamples.add(robot.drive.getPoseEstimate().getY());
            powerSamples.add(MAX_POWER);

            robot.drive.updateLocalizer();

            waitTime(SIMULATED_SAMPLE_TIME_MS);
        }
        robot.drive.stopAllMotors();

        RegressionUtil.AccelResult accelResult = RegressionUtil.fitAccelData(
                timeSamples, positionSamples, powerSamples, rampResult,
                LoggingUtil.getLogFile("DriveAccelRegression-"+System.currentTimeMillis()+".csv"));

        telemetry.addData("","Constant power test complete");
        telemetry.addData("","kA = " + accelResult.kA +" (R^2 = " + accelResult.rSquare+")");
        telemetry.update();

        waitUntil(() -> robot.isStopRequested());
    }
}
