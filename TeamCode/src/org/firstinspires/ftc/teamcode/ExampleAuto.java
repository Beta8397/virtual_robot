package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import system.config.StandAlone;
import system.robot.BaseAutonomous;
import system.robot.roadrunner_util.HALTrajectory;
import system.robot.MainRobot;
import util.math.geometry.Point2D;
import util.math.units.HALDistanceUnit;

import static java.lang.Math.PI;

@StandAlone
@Autonomous(name = "Example HAL Autonomous")
public class ExampleAuto extends BaseAutonomous {
    public @MainRobot ExampleBot robot;

    @Override
    public void main() {
        HALTrajectory forward = robot.drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Point2D(24,24),0)
                .build();
        
        robot.drive.followTrajectory(forward);

        HALTrajectory backward = robot.drive.trajectoryBuilder(forward.end(), true)
                .splineTo(new Point2D(0,0), PI)
                .build();

        robot.drive.followTrajectory(backward);
    }
}
