package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import system.config.StandAlone;
import system.robot.BaseAutonomous;
import system.robot.MainRobot;
import util.math.geometry.Vector2D;
import util.math.units.HALAngleUnit;

@StandAlone
@Autonomous(name = "Example HAL Autonomous")
public class ExampleAuto extends BaseAutonomous {
    public @MainRobot ExampleBot robot;

    @Override
    public void main() {
        robot.drive.driveTime(new Vector2D(1,0,HALAngleUnit.DEGREES), 1000);
    }
}
