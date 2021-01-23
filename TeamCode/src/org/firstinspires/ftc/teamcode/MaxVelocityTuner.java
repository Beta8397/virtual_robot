package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import system.config.StandAlone;
import system.robot.BaseAutonomous;
import system.robot.MainRobot;
import system.robot.roadrunner_util.RoadrunnerConfig;
import util.math.geometry.Vector2D;

import java.util.Objects;

@StandAlone
@Autonomous(name = "Max Velocity Tuner")
public class MaxVelocityTuner extends BaseAutonomous {
    public @MainRobot RoadrunnerCalibBot robot;

    public static double RUNTIME = 2.0;

    private double maxVelocity = 0.0;

    //private VoltageSensor batteryVoltageSensor;

    @Override
    protected void onInit() {
        robot.drive.setAllMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.telemetry.addData("","Your bot will go at full speed for " + RUNTIME + " seconds.");
        robot.telemetry.addData("","Please ensure you have enough space cleared.");
        robot.telemetry.addData("","");
        robot.telemetry.addData("","Press start when ready.");
        robot.telemetry.update();
    }

    @Override
    public void main() {
        robot.drive.movePower(new Vector2D(0,1));
        ElapsedTime timer = new ElapsedTime();

        while (!isStopRequested() && timer.seconds() < RUNTIME) {
            robot.drive.getLocalizer().update();

            Pose2d poseVelo = Objects.requireNonNull(robot.drive.getLocalizer().getPoseVelocity(), "poseVelocity() must not be null. Ensure that the getWheelVelocities() method has been overridden in your localizer.");

            maxVelocity = Math.max(poseVelo.vec().norm(), maxVelocity);
        }

        robot.drive.stopAllMotors();

        double effectiveKf = RoadrunnerConfig.getMotorVelocityF(veloInchesToTicks(maxVelocity));

        telemetry.addData("Max Velocity", maxVelocity);
        telemetry.addData("Voltage Compensated kF", effectiveKf); //TODO * batteryVoltageSensor.getVoltage() / 12);
        telemetry.update();

        waitUntil(()->robot.isStopRequested());
    }

    private double veloInchesToTicks(double inchesPerSec) {
        return inchesPerSec / (2 * Math.PI * robot.drive.driveConfig.WHEEL_RADIUS) / robot.drive.driveConfig.GEAR_RATIO * robot.drive.driveConfig.TICKS_PER_REV;
    }
}
