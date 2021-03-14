package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import system.robot.BaseAutonomous;
import system.robot.MainRobot;
import system.robot.roadrunner_util.HALPath;
import system.robot.roadrunner_util.HALTrajectory;
import system.robot.subsystems.drivetrain.Drivetrain;
import system.robot.subsystems.drivetrain.HolonomicDrivetrain;
import util.control.Button;
import util.math.geometry.Point2D;
import util.math.geometry.Vector2D;
import util.math.units.HALAngleUnit;
import util.math.units.HALDistanceUnit;

@Autonomous(name = "Test Program")
public class TestProgram extends BaseAutonomous {
    private @MainRobot RoadrunnerCalibBot robot;

    @Override
    public void main() {

        HALTrajectory trajectory = robot.drive.trajectoryBuilder(new Pose2d(0,0,0), HALDistanceUnit.INCHES, HALAngleUnit.RADIANS)
                .splineTo(new Point2D(24,24),0)
                .build();

        robot.drive.followTrajectory(trajectory);
    }
}
