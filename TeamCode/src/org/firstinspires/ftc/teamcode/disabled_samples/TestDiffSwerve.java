package org.firstinspires.ftc.teamcode.disabled_samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name = "Test Differential Swerve", group = "Test")
public class TestDiffSwerve extends OpMode {

    DiffSwerveDrive bot = new DiffSwerveDrive();

    public void init(){

        bot.init(hardwareMap);

    }

    public void loop(){

        float vx = gamepad1.left_stick_x * (float)DiffSwerveDrive.MAX_DRIVE_SPEED;
        float vy = -gamepad1.left_stick_y * (float)DiffSwerveDrive.MAX_DRIVE_SPEED;
        float va = -gamepad1.right_stick_x * (float)DiffSwerveDrive.MAX_ANGULAR_SPEED;

        bot.setDriveSpeed(vx, vy, va);
    }

}
