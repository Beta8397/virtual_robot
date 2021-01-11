package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import system.config.StandAlone;
import system.robot.BaseAutonomous;
import system.robot.MainRobot;
import util.math.geometry.Vector2D;
import util.math.units.HALAngleUnit;
import util.math.units.HALDistanceUnit;

@StandAlone
@Autonomous(name = "Example HAL Autonomous3")
public class ExampleAuto3 extends BaseAutonomous {
    public @MainRobot ExampleBot3 robot;

    @Override
    public void main() {
        robot.drive.turnSimple(0.1,90, HALAngleUnit.DEGREES);
        robot.drive.moveSimple(1, HALDistanceUnit.TILES,0.7);
    }
}
