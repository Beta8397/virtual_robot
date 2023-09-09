package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Kiwi Drive Teleop", group = "Kiwi")
public class KiwiDriveTele extends LinearOpMode {

    DcMotorEx mFront, mBackLeft, mBackRight;
    DistanceSensor dF, dR, dL, dB;

    public void runOpMode(){
        mFront = hardwareMap.get(DcMotorEx.class, "front_motor");
        mBackLeft = hardwareMap.get(DcMotorEx.class, "back_left_motor");
        mBackRight = hardwareMap.get(DcMotorEx.class, "back_right_motor");
        dF = hardwareMap.get(DistanceSensor.class, "front_distance");
        dL = hardwareMap.get(DistanceSensor.class, "left_distance");
        dR = hardwareMap.get(DistanceSensor.class, "right_distance");
        dB = hardwareMap.get(DistanceSensor.class, "back_distance");


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

            double fDist = dF.getDistance(DistanceUnit.INCH);
            double lDist = dL.getDistance(DistanceUnit.INCH);
            double rDist = dR.getDistance(DistanceUnit.INCH);
            double bDist = dB.getDistance(DistanceUnit.INCH);

            telemetry.addData("Dist","F %.1f  L %.1f  R %.1f  B %.1f", fDist, lDist, rDist, bDist);
            telemetry.update();


        }
    }
}
