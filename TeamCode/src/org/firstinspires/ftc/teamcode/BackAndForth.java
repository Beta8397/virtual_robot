package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import system.config.StandAlone;
import system.robot.BaseAutonomous;
import system.robot.MainRobot;
import system.robot.roadrunner_util.HALTrajectory;

@StandAlone
@Autonomous(name = "Back and Forth")
public class BackAndForth extends BaseAutonomous {
    public @MainRobot RoadrunnerCalibBot robot;

    private static final double DISTANCE = 50;

    @Override
    public void main() {
        HALTrajectory forwardTrajectory = robot.drive.trajectoryBuilder(new Pose2d())
                .forward(DISTANCE)
                .build();

        HALTrajectory backwardTrajectory = robot.drive.trajectoryBuilder(forwardTrajectory.end())
                .back(DISTANCE)
                .build();

        robot.drive.followTrajectory(forwardTrajectory);
        robot.drive.followTrajectory(backwardTrajectory);
    }
}
