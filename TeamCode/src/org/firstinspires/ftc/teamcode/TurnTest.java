package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import system.config.StandAlone;
import system.robot.BaseAutonomous;
import system.robot.MainRobot;

import static java.lang.Math.toRadians;

@StandAlone
@Autonomous(name = "Turn Test")
public class TurnTest extends BaseAutonomous {
    public @MainRobot RoadrunnerCalibBot robot;

    public static double ANGLE = 90; // deg

    @Override
    public void main() {
        robot.drive.turn(toRadians(ANGLE));
    }
}
