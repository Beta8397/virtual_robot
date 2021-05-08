package org.firstinspires.ftc.teamcode.disabled_samples;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.*;

/**
 * Example OpMode. Demonstrates use of gyro, color sensor, encoders, and telemetry.
 *
 */
@Disabled
@TeleOp(name = "two wheel demo linear", group = "TwoWheel")
public class TwoWheelDemoLinear extends LinearOpMode {

    DcMotorEx left;
    DcMotorEx right;
    BNO055IMU imu;
    Servo backServo;
    ColorSensor colorSensor;
    DistanceSensor frontDistance, leftDistance, backDistance, rightDistance;

    public void runOpMode() {
        left = (DcMotorEx)hardwareMap.dcMotor.get("left_motor");
        right = (DcMotorEx)hardwareMap.dcMotor.get("right_motor");
        left.setDirection(DcMotor.Direction.REVERSE);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        backServo = hardwareMap.servo.get("back_servo");
        colorSensor = hardwareMap.colorSensor.get("color_sensor");
        frontDistance = hardwareMap.get(DistanceSensor.class, "front_distance");
        leftDistance = hardwareMap.get(DistanceSensor.class, "left_distance");
        backDistance = hardwareMap.get(DistanceSensor.class, "back_distance");
        rightDistance = hardwareMap.get(DistanceSensor.class, "right_distance");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelerationIntegrationAlgorithm = null;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationData = null;
        parameters.calibrationDataFile = "";
        parameters.loggingEnabled = false;
        parameters.loggingTag = "Who cares.";
        imu.initialize(parameters);

        telemetry.addData("Press Start to Continue","");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.a){
                telemetry.addData("a pressed","");
                left.setPower(-.5);
                right.setPower(-.5);
            } else if (gamepad1.y) {
                telemetry.addData("y pressed", "");
                left.setVelocity(401.8, AngleUnit.DEGREES);
                right.setVelocity(401.8, AngleUnit.DEGREES);
            } else if (gamepad1.b){
                telemetry.addData("b pressed", "");
                left.setPower(0.5);
                right.setPower(-0.5);
            } else if (gamepad1.x){
                telemetry.addData("x pressed", "");
                left.setPower(-0.5);
                right.setPower(0.5);
            } else {
                left.setPower(0);
                right.setPower(0);
            }
            backServo.setPosition(0.5 - 0.5* gamepad1.left_stick_y);
            telemetry.addData("Press", "Y-fwd, A-rev, B-Rt, X-Lt");
            telemetry.addData("Left Gamepad stick controls back servo","");
            telemetry.addData("Color","R %d  G %d  B %d", colorSensor.red(), colorSensor.green(), colorSensor.blue());
            Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Heading"," %.1f", orientation.firstAngle);
            telemetry.addData("Encoders","Left %d  Right %d", left.getCurrentPosition(), right.getCurrentPosition());
            telemetry.addData("Vel, TPS", "Left %.0f  Right %.0f", left.getVelocity(), right.getVelocity());
            telemetry.addData("Vel, DPS", "Left %.1f  Right %.1f", left.getVelocity(AngleUnit.DEGREES), right.getVelocity(AngleUnit.DEGREES));
            telemetry.addData("Distance", " Fr %.1f  Lt %.1f  Rt %.1f  Bk %.1f  ",
                    frontDistance.getDistance(DistanceUnit.CM), leftDistance.getDistance(DistanceUnit.CM),
                    rightDistance.getDistance(DistanceUnit.CM), backDistance.getDistance(DistanceUnit.CM)
                    );
            telemetry.update();
        }
        left.setPower(0);
        right.setPower(0);
    }
}
