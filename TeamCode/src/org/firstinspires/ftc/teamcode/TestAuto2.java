package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import system.config.StandAlone;
import system.robot.BaseAutonomous;
import system.robot.MainRobot;
import system.robot.subsystems.drivetrain.HolonomicDrivetrain;
import util.math.geometry.Vector2D;
import util.math.units.HALAngleUnit;
import util.math.units.HALDistanceUnit;

import static java.lang.Math.PI;

@StandAlone
@Autonomous(name = "testing 2")
public class TestAuto2 extends BaseAutonomous {
    public @MainRobot TestRobot robot;

    @Override
    public void main() {
        robot.drive.setDriveMode(HolonomicDrivetrain.DriveMode.FIELD_CENTRIC);
        //robot.drive.setDriveMode(HolonomicDrivetrain.DriveMode.FIELD_CENTRIC);
        //robot.drive.moveTime(new Vector2D(0,1), 1000);
        //robot.drive.setDriveMode(HolonomicDrivetrain.DriveMode.FIELD_CENTRIC);
        robot.drive.moveSimple(1,0, HALDistanceUnit.TILES, 1);
        robot.drive.turnSimple(0.7, 90, HALAngleUnit.DEGREES);
        //robot.drive.moveTime(new Vector2D(0,1), 1000);

        robot.drive.moveSimple(1, HALDistanceUnit.TILES, 45, HALAngleUnit.DEGREES, 1);
    }
}
