package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import system.config.StandAlone;
import system.robot.BaseAutonomous;
import system.robot.MainRobot;
import util.math.geometry.Vector2D;
import util.math.units.HALAngleUnit;
import util.math.units.HALDistanceUnit;

@StandAlone
@Autonomous(name = "Example HAL Autonomous2")
public class ExampleAuto2 extends BaseAutonomous {
    public @MainRobot ExampleBot2 robot;

    @Override
    public void main() {

        //robot.drive.turnPID(90, HALAngleUnit.DEGREES, 0.01);

        //robot.drive.turnSimple(0.5,-90, HALAngleUnit.DEGREES);
        //waitTime(1000);
        //robot.drive.turnSimple(0.5,90, HALAngleUnit.DEGREES);
        robot.drive.turnPID(90, HALAngleUnit.DEGREES);
        waitTime(1000);
        robot.drive.moveSimple(new Vector2D(0,1), HALDistanceUnit.TILES, 1);
        waitTime(1000);
        robot.drive.moveSimple(new Vector2D(0,-1), HALDistanceUnit.TILES,1);
        waitTime(1000);
        robot.drive.moveSimple(new Vector2D(1,0), HALDistanceUnit.TILES, 1);
        waitTime(1000);
        robot.drive.moveSimple(new Vector2D(-1,0), HALDistanceUnit.TILES, 1);
        waitTime(1000);
        robot.drive.moveSimple(new Vector2D(0,-1), HALDistanceUnit.TILES, 1);
        waitTime(1000);
        robot.drive.moveSimple(new Vector2D(0,1), HALDistanceUnit.TILES, 1);
        waitTime(1000);
        robot.drive.moveSimple(new Vector2D(-1,0), HALDistanceUnit.TILES, 1);
        waitTime(1000);
        robot.drive.moveSimple(new Vector2D(1,0), HALDistanceUnit.TILES, 1);
    }
}
