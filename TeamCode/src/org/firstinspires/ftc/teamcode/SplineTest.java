package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import system.config.StandAlone;
import system.robot.BaseAutonomous;
import system.robot.MainRobot;
import system.robot.roadrunner_util.HALTrajectory;
import util.math.geometry.Point2D;
import util.math.units.HALAngleUnit;

@StandAlone
@Autonomous(name = "Spline Test")
public class SplineTest extends BaseAutonomous {
    public @MainRobot RoadrunnerCalibBot robot;

    @Override
    public void main() {
        HALTrajectory traj = robot.drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Point2D(30, 30), 0)
                .build();

        robot.drive.followTrajectory(traj);

        waitTime(2000);

        robot.drive.followTrajectory(
                robot.drive.trajectoryBuilder(traj.end(), HALAngleUnit.DEGREES,true)
                        .splineTo(new Point2D(0, 0), 180)
                        .build()
        );
    }
}
