package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Kiwi Drive Teleop", group = "Kiwi")
public class KiwiDriveTele extends LinearOpMode {

    DcMotorEx mFront, mBackLeft, mBackRight;

    public void runOpMode(){
        mFront = hardwareMap.get(DcMotorEx.class, "front_motor");
        mBackLeft = hardwareMap.get(DcMotorEx.class, "back_left_motor");
        mBackRight = hardwareMap.get(DcMotorEx.class, "back_right_motor");

        waitForStart();

        while (opModeIsActive()){
            double px = gamepad1.left_stick_x;
            double py = -gamepad1.left_stick_y;
            double pa = -gamepad1.right_stick_x;

            double pF = -px + 0*py + pa;
            double pBL = 0.5*px - Math.sqrt(3)*py/2 + pa;
            double pBR = 0.5*px + Math.sqrt(3)*py/2 + pa;

            double max = Math.max(1, Math.max(Math.abs(pF), Math.max(Math.abs(pBL), Math.abs(pBR))));

            if (max > 1) {
                pF /= max;
                pBL /= max;
                pBR /= max;
            }

            mFront.setPower(pF);
            mBackLeft.setPower(pBL);
            mBackRight.setPower(pBR);
        }
    }
}
