package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * TeleOp op mode to test odometry with three "dead-wheel" encoders.
 */
@TeleOp(name = "TestOdom", group = "OdomBot")
public class TestOdom extends LinearOpMode {

    EncBot bot = new EncBot();
    double[] pose;

    public void runOpMode(){
        bot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()){
            pose = bot.updateOdometry();
            telemetry.addData("POSE", "x = %.1f  y = %.1f  h = %.1f", pose[0], pose[1],
                    Math.toDegrees(pose[2]));
            telemetry.update();
            double px = gamepad1.left_stick_x;
            double py = -gamepad1.left_stick_y;
            double pa = gamepad1.left_trigger - gamepad1.right_trigger;
            bot.setDrivePower(px, py, pa);
        }
    }
}
