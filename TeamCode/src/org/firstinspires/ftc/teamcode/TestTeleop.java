package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import system.config.StandAlone;
import system.robot.BaseTeleop;
import system.robot.MainRobot;
import system.robot.subsystems.drivetrain.HolonomicDrivetrain;

@StandAlone
@TeleOp(name = "test teleop", group = "testing")
public class TestTeleop extends BaseTeleop {
    public @MainRobot TestRobot robot;

    @Override
    protected void onStart() {
        robot.drive.setDriveMode(HolonomicDrivetrain.DriveMode.FIELD_CENTRIC);
    }
}
