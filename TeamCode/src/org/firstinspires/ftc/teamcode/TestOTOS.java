package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D;

@TeleOp
public class TestOTOS extends LinearOpMode {

    SparkFunOTOS otos;
    DcMotorEx bl, fl, fr, br;

    public void runOpMode(){

        otos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");

        bl = hardwareMap.get(DcMotorEx.class, "back_left_motor");
        fl = hardwareMap.get(DcMotorEx.class, "front_left_motor");
        fr = hardwareMap.get(DcMotorEx.class, "front_right_motor");
        br = hardwareMap.get(DcMotorEx.class, "back_right_motor");
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);

        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);
        otos.setAngularScalar(1.0);
        otos.setLinearScalar(1.0);
        otos.setOffset(new Pose2D(0, 0, 0));
        otos.calibrateImu();

        Pose2D otosPos = otos.getPosition();
        Pose2D otosVel = otos.getVelocity();
        Pose2D otosAccel = otos.getAcceleration();

        telemetry.addData("Otos Pose", "x: %.1f  y: %.1f  h: %.1f", otosPos.x, otosPos.y, otosPos.h);
        telemetry.addData("Otos Vel", "vx: %.1f  vy: %.1f  vh: %.1f", otosVel.x, otosVel.y, otosVel.h);
        telemetry.addData("Otos Accel", "ax: %.1f  ay: %.1f  ah: %.1f", otosAccel.x, otosAccel.y, otosAccel.h);

        telemetry.update();


        waitForStart();

        while (opModeIsActive()){

            if (gamepad1.a){
                otos.setPosition(new Pose2D(0, 0, 0));
            } else if (gamepad1.b){
                otos.resetTracking();
            }

            otosPos = otos.getPosition();
            otosVel = otos.getVelocity();
            otosAccel = otos.getAcceleration();

            telemetry.addData("Otos Pose", "x: %.1f  y: %.1f  h: %.1f", otosPos.x, otosPos.y, otosPos.h);
            telemetry.addData("Otos Vel", "vx: %.1f  vy: %.1f  vh: %.1f", otosVel.x, otosVel.y, otosVel.h);
            telemetry.addData("Otos Accel", "ax: %.1f  ay: %.1f  ah: %.1f", otosAccel.x, otosAccel.y, otosAccel.h);

            telemetry.update();


            double pFwd = -gamepad1.left_stick_y;
            double pRight = gamepad1.left_stick_x;
            double pTurn = -gamepad1.right_stick_x;
            double pBL = pFwd - pRight - pTurn;
            double pFL = pFwd + pRight - pTurn;
            double pFR = pFwd - pRight + pTurn;
            double pBR = pFwd + pRight + pTurn;
            double max = Math.max(1, Math.max(Math.abs(pFwd), Math.max(Math.abs(pRight), Math.abs(pTurn))));
            if (max > 1){
                pBL /= max;
                pFL /= max;
                pFR /= max;
                pBR /= max;
            }
            bl.setPower(pBL);
            fl.setPower(pFL);
            fr.setPower(pFR);
            br.setPower(pBR);

        }

    }

}
