package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.*;

/**
 * Example OpMode. Demonstrates use of gyro, color sensor, encoders, and telemetry.
 *
 */
@TeleOp(name = "mecanum bot demo", group = "MecanumBot")
public class MecanumDemo extends LinearOpMode {

    public void runOpMode(){
        DcMotor m1 = hardwareMap.dcMotor.get("back_left_motor");
        DcMotor m2 = hardwareMap.dcMotor.get("front_left_motor");
        DcMotor m3 = hardwareMap.dcMotor.get("front_right_motor");
        DcMotor m4 = hardwareMap.dcMotor.get("back_right_motor");
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

        DistanceSensor frontDistance = hardwareMap.get(DistanceSensor.class, "front_distance");
        DistanceSensor leftDistance = hardwareMap.get(DistanceSensor.class, "left_distance");
        DistanceSensor rightDistance = hardwareMap.get(DistanceSensor.class, "right_distance");
        DistanceSensor backDistance = hardwareMap.get(DistanceSensor.class, "back_distance");

        IMU imu = hardwareMap.get(IMU.class, "imu");

        ColorSensor colorSensor = hardwareMap.colorSensor.get("color_sensor");
        OctoQuad octoQuad = hardwareMap.get(OctoQuad.class, "octoquad");

        telemetry.addData("Press Start When Ready","");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()){
            double px = gamepad1.left_stick_x;
            double py = -gamepad1.left_stick_y;
            double pa = gamepad1.left_trigger - gamepad1.right_trigger;

            if (Math.abs(pa) < 0.05) pa = 0;
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
            telemetry.addData("Color","R %d  G %d  B %d", colorSensor.red(), colorSensor.green(), colorSensor.blue());
//            Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            Orientation orientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            telemetry.addData("Heading", " %.1f", orientation.firstAngle * 180.0 / Math.PI);
            telemetry.addData("Angular Velocity", "%.1f", imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate);
            telemetry.addData("Front Distance", " %.1f", frontDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Left Distance", " %.1f", leftDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Right Distance", " %.1f", rightDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Back Distance", " %.1f", backDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Encoders"," %d %d %d %d", m1.getCurrentPosition(), m2.getCurrentPosition(),
                    m3.getCurrentPosition(), m4.getCurrentPosition());
            telemetry.addData("Octoquad", "%d %d %d %d", octoQuad.readSinglePosition(0),
                    octoQuad.readSinglePosition(1), octoQuad.readSinglePosition(2),
                    octoQuad.readSinglePosition(3));
            telemetry.update();
        }
        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
    }
}
