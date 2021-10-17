package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.*;

/**
 * Example OpMode. Demonstrates use of gyro, color sensor, encoders, and telemetry.
 *
 */
@TeleOp(name = "beta bot demo", group = "BetaBot")
public class BetaBotDemo extends LinearOpMode {

    private DcMotorEx m1, m2, m3, m4;
    private Servo kickerServo;
    private DcMotorEx intakeMotor;
    private DcMotorEx shooterMotor;
    private DcMotorEx scoopMotor;

    public void runOpMode(){
        m1 = hardwareMap.get(DcMotorEx.class,"back_left_motor");
        m2 = hardwareMap.get(DcMotorEx.class,"front_left_motor");
        m3 = hardwareMap.get(DcMotorEx.class,"front_right_motor");
        m4 = hardwareMap.get(DcMotorEx.class,"back_right_motor");
        m1.setDirection(DcMotor.Direction.REVERSE);
        m2.setDirection(DcMotor.Direction.REVERSE);
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        kickerServo = hardwareMap.get(Servo.class, "kicker_servo");
        kickerServo.setPosition(0);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        scoopMotor = hardwareMap.get(DcMotorEx.class, "scoop_motor");

        intakeMotor.setPower(0.75);
        shooterMotor.setPower(0.75);

        telemetry.addData("Drive: ", "Lt and Rt Sticks");
        telemetry.addData("Shoot: ", "'A' (250ms on, 250ms off)");
        telemetry.addData("Scoop: ", "Dpad up/down");
        telemetry.addData("Intake: ", "Back of bot");
        telemetry.addData("Press 'START' to start.","");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()){

            telemetry.addData("Drive: ", "Lt and Rt Sticks");
            telemetry.addData("Shoot: ", "'A' (250ms on, 250ms off)");
            telemetry.addData("Scoop: ", "Dpad up/down");
            telemetry.addData("Intake: ","Back of bot");
            telemetry.update();

            double px = gamepad1.left_stick_x;
            double py = -gamepad1.left_stick_y;
            double pa = -gamepad1.right_stick_x;
            double p1 = -px + py - pa;
            double p2 = px + py + -pa;
            double p3 = -px + py + pa;
            double p4 = px + py + pa;
            double max = Math.max(1.0, Math.abs(p1));
            max = Math.max(max, Math.abs(p2));
            max = Math.max(max, Math.abs(p3));
            max = Math.max(max, Math.abs(p4));
            p1 /= max;
            p2 /= max;
            p3 /= max;
            p4 /= max;
            m1.setPower(p1);
            m2.setPower(p2);
            m3.setPower(p3);
            m4.setPower(p4);

            if (gamepad1.a) kickerServo.setPosition(1);
            else kickerServo.setPosition(0);

            if (gamepad1.dpad_up) scoopMotor.setPower(0.5);
            else if (gamepad1.dpad_down) scoopMotor.setPower(-0.5);
            else scoopMotor.setPower(0);
        }

        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
    }
}
