package org.firstinspires.ftc.teamcode.disabled_samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Disabled
@TeleOp(name = "Square Omni Demo", group = "Square Omni")
public class SquareOmniDemo extends LinearOpMode {

    private DcMotorEx left, right, front, back;
    private IMU imu;

    public void runOpMode(){
        left = hardwareMap.get(DcMotorEx.class, "left_motor");
        right = hardwareMap.get(DcMotorEx.class, "right_motor");
        front = hardwareMap.get(DcMotorEx.class, "front_motor");
        back = hardwareMap.get(DcMotorEx.class, "back_motor");
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        back.setDirection(DcMotorSimple.Direction.REVERSE);
        imu = hardwareMap.get(IMU.class, "imu");

        waitForStart();

        while (opModeIsActive()){

            float px = gamepad1.left_stick_x;
            float py = -gamepad1.left_stick_y;
            float pa = -gamepad1.right_stick_x;

            float pLeft = py - pa;
            float pRight = py + pa;
            float pFront = px - pa;
            float pBack = px + pa;

            float max = Math.max(1, Math.abs(pLeft));
            max = Math.max(max, Math.abs(pRight));
            max = Math.max(max, Math.abs(pFront));
            max = Math.max(max, Math.abs(pBack));

            if (max > 1){
                pLeft /= max;
                pRight /= max;
                pFront /= max;
                pBack /= max;
            }

            left.setPower(pLeft);
            right.setPower(pRight);
            front.setPower(pFront);
            back.setPower(pBack);

            YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
            telemetry.addData("YPR", "Y %.1f  P %.1f  R %.1f", ypr.getYaw(AngleUnit.DEGREES),
                    ypr.getPitch(AngleUnit.DEGREES), ypr.getRoll(AngleUnit.DEGREES));
            telemetry.update();
        }
    }

}
