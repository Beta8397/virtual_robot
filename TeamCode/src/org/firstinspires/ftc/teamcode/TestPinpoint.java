package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

@TeleOp
public class TestPinpoint extends LinearOpMode {

    GoBildaPinpointDriver pinPoint;
    DcMotorEx bl, fl, fr, br;

    public void runOpMode(){

        pinPoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinPoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinPoint.setOffsets(100, 100);
        pinPoint.setEncoderResolution(336.88, DistanceUnit.INCH);

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

        Pose2D pinPose = pinPoint.getPosition();
        Pose2D pinVel = pinPoint.getVelocity();
        telemetry.addData("Pinpoint Offsets", "xOff: %.1f  yOff: %.1f",
                pinPoint.getXOffset(DistanceUnit.INCH), pinPoint.getYOffset(DistanceUnit.INCH));
        telemetry.addData("Pinpoint Pos", "x: %.1f  y: %.1f  h: %.1f", pinPose.getX(DistanceUnit.INCH),
                pinPose.getY(DistanceUnit.INCH), pinPose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Pinpoint Vel", "x: %.1f  y: %.1f  h: %.1f", pinVel.getX(DistanceUnit.INCH),
                pinVel.getY(DistanceUnit.INCH), pinVel.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Pinpoint Encoders", "X: %d  Y: %d", pinPoint.getEncoderX(), pinPoint.getEncoderY());
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){

            if (gamepad1.a){
                pinPoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0));
            }

            pinPoint.update();

            pinPose = pinPoint.getPosition();
            pinVel = pinPoint.getVelocity();
            double pinX = pinPoint.getPosX(DistanceUnit.INCH);
            double pinY = pinPoint.getPosY(DistanceUnit.INCH);
            double pinH = pinPoint.getHeading(AngleUnit.DEGREES);
            double pinVX = pinPoint.getVelX(DistanceUnit.INCH);
            double pinVY = pinPoint.getVelY(DistanceUnit.INCH);
            double pinVH = pinPoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);


            telemetry.addData("Pinpoint Pos", "x: %.1f  y: %.1f  h: %.1f", pinPose.getX(DistanceUnit.INCH),
                    pinPose.getY(DistanceUnit.INCH), pinPose.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Pinpoint Vel", "x: %.1f  y: %.1f  h: %.1f", pinVel.getX(DistanceUnit.INCH),
                    pinVel.getY(DistanceUnit.INCH), pinVel.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Pinpoint Pos", "x: %.1f  y: %.1f  h: %.1f", pinX, pinY, pinH);
            telemetry.addData("Pinpoint Vel", "x: %.1f  y: %.1f  h: %.1f", pinVX, pinVY, pinVH);
            telemetry.addData("Pinpoint Encoders", "X: %d  Y: %d", pinPoint.getEncoderX(), pinPoint.getEncoderY());

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
