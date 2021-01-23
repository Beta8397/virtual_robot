package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import system.config.StandAlone;
import system.robot.BaseAutonomous;
import system.robot.MainRobot;

import java.util.Objects;

@StandAlone
@Autonomous(name = "Max Angular Velocity Tuner")
public class MaxAngularVeloTuner extends BaseAutonomous {
    public @MainRobot RoadrunnerCalibBot robot;

    public static double RUNTIME = 4.0;

    private double maxAngVelocity = 0.0;

    @Override
    protected void onInit() {
        robot.drive.setAllMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.telemetry.addData("","Your bot will turn at full speed for " + RUNTIME + " seconds.");
        robot.telemetry.addData("","Please ensure you have enough space cleared.");
        robot.telemetry.addData("","");
        robot.telemetry.addData("","Press start when ready.");
        robot.telemetry.update();
    }

    @Override
    public void main() {
        robot.drive.turnPower(1);
        ElapsedTime timer = new ElapsedTime();

        while (!robot.isStopRequested() && timer.seconds() < RUNTIME) {
            robot.drive.getLocalizer().update();

            Pose2d poseVelo = Objects.requireNonNull(robot.drive.getLocalizer().getPoseVelocity(), "poseVelocity() must not be null. Ensure that the getWheelVelocities() method has been overridden in your localizer.");

            maxAngVelocity = Math.max(poseVelo.getHeading(), maxAngVelocity);
        }

        robot.drive.stopAllMotors();

        robot.telemetry.addData("Max Angular Velocity (rad)", maxAngVelocity);
        robot.telemetry.addData("Max Angular Velocity (deg)", Math.toDegrees(maxAngVelocity));
        robot.telemetry.update();

        waitUntil(() -> robot.isStopRequested());
    }
}
