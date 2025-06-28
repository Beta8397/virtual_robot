package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

@Autonomous
public class TestDynMecBot extends LinearOpMode {

    DcMotorEx bL, fL, bR, fR;
    GoBildaPinpointDriver pinPoint;

    public void runOpMode(){

        bL = hardwareMap.get(DcMotorEx.class, "back_left_motor");
        fL = hardwareMap.get(DcMotorEx.class, "front_left_motor");
        fR = hardwareMap.get(DcMotorEx.class, "front_right_motor");
        bR = hardwareMap.get(DcMotorEx.class, "back_right_motor");

        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        bL.setDirection(DcMotorSimple.Direction.REVERSE);
        fL.setDirection(DcMotorSimple.Direction.REVERSE);

        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

//        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        bL.setTargetPosition(4000);
//        fL.setTargetPosition(4000);
//        fR.setTargetPosition(4000);
//        bR.setTargetPosition(4000);
//        bL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        fL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        fR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        bR.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        pinPoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        waitForStart();

        pinPoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0));

        bL.setPower(0.5);
        fL.setPower(0.5);
        fR.setPower(0.5);
        bR.setPower(0.5);

        while (opModeIsActive() && pinPoint.getPosX(DistanceUnit.INCH) < 48){
            pinPoint.update();
            System.out.printf("\nVel: vx = %.2f  vy = %.2f  vh = %.2f",
                    pinPoint.getVelX(DistanceUnit.INCH), pinPoint.getVelY(DistanceUnit.INCH),
                    pinPoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
        }

        bL.setPower(0);
        fL.setPower(0);
        fR.setPower(0);
        bR.setPower(0);

        while(opModeIsActive()) continue;

    }

}
