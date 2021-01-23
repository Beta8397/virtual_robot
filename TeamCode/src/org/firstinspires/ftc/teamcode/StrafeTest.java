package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import system.config.StandAlone;
import system.robot.BaseAutonomous;
import system.robot.MainRobot;
import system.robot.roadrunner_util.HALTrajectory;

@StandAlone
@Autonomous(name = "Strafe Test")
public class StrafeTest extends BaseAutonomous {
    public @MainRobot RoadrunnerCalibBot robot;

    public static double DISTANCE = 60; // in

    @Override
    public void main() {
        HALTrajectory trajectory = robot.drive.trajectoryBuilder(new Pose2d())
                .strafeRight(DISTANCE)
                .build();

        robot.drive.followTrajectory(trajectory);

        Pose2d poseEstimate = robot.drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        waitUntil(()->robot.isStopRequested());
    }
}
