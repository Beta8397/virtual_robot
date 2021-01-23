package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import system.config.StandAlone;
import system.robot.BaseTeleop;
import system.robot.MainRobot;

@StandAlone
@TeleOp(name = "Localization Test")
public class LocalizationTest extends BaseTeleop {
    public @MainRobot RoadrunnerCalibBot robot;

    @Override
    protected void onInit() {
        robot.drive.setAllMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    protected void onUpdate() {
        Pose2d poseEstimate = robot.drive.getPoseEstimate();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.update();
    }
}
