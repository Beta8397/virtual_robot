package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import system.config.StandAlone;
import system.robot.BaseAutonomous;
import system.robot.MainRobot;
import system.robot.roadrunner_util.CoordinateMode;
import system.robot.roadrunner_util.HALTrajectory;
import util.math.geometry.Point2D;
import util.math.geometry.Vector2D;
import util.math.units.HALAngleUnit;
import util.math.units.HALDistanceUnit;

import static java.lang.Math.PI;

@StandAlone
@Autonomous(name = "test")
public class TestAuto extends BaseAutonomous {
    public @MainRobot RoadrunnerCalibBot robot;

    @Override
    public void main() {
        //robot.drive.moveSimple(new Vector2D(1,0, HALAngleUnit.DEGREES), HALDistanceUnit.TILES, 1);

        //robot.drive.setCoordinateMode(CoordinateMode.HAL);
        //robot.drive.moveSimple(new Vector2D(0.5, 0, HALAngleUnit.DEGREES), HALDistanceUnit.TILES, 0.5);


        HALTrajectory trajectory = robot.drive.trajectoryBuilder(new Pose2d(), HALDistanceUnit.TILES, HALAngleUnit.DEGREES)
                .splineToConstantHeading(new Point2D(-0.75, 2.5), 0)
                .splineTo(new Point2D(0,2.5), 180)
                .splineTo(new Point2D(0,1.5),-90)
                .splineTo(new Point2D(0.5, 1.5), 0)
                .splineToConstantHeading(new Point2D(1, 4.5), 90)
                .splineToConstantHeading(new Point2D(0.5, 3), 180)
                .build();
        robot.drive.followTrajectory(trajectory);

    }
}
