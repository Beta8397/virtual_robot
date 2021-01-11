package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import system.config.StandAlone;
import system.robot.BaseAutonomous;
import system.robot.roadrunner_util.HALTrajectory;
import system.robot.MainRobot;
import util.math.geometry.Point2D;

import static java.lang.Math.PI;

@StandAlone
@Autonomous(name = "Example HAL Autonomous4")
public class ExampleAuto4 extends BaseAutonomous {
    public @MainRobot ExampleBot4 robot;

    @Override
    public void main() {
        HALTrajectory forward = robot.drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Point2D(30,30),0)
                .build();

        HALTrajectory backward = robot.drive.trajectoryBuilder(forward.end(), true)
                .splineTo(new Point2D(0,0), PI)
                .build();

        robot.drive.followTrajectory(forward);
        robot.drive.followTrajectory(backward);
    }
}
