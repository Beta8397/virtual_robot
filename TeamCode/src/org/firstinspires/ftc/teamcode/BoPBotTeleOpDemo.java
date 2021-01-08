package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.MotorType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import Misc.Log;
import UserControlled.GamepadController;

/**
 * Example OpMode for BoPBot. Demonstrates use of XDrive drive system, IMU, shooter system,
 * and intake system.
 */
@TeleOp(name = "BoP Bot TeleOp", group = "Linear Opmode")
public class BoPBotTeleOpDemo extends LinearOpMode {

    private static final double WOBBLE_GOAL_GRAB_ANGLE_DEG = 180.0;
    private static final double WOBBLE_GOAL_CARRY_ANGLE_DEG = 155.0;
    private static final double WOBBLE_GOAL_DROP_ANGLE_DEG = 120.0;
    private static final double WOBBLE_GOAL_HOME_ANGLE_DEG = 0.0;

    public void runOpMode() {

        GamepadController controller = new GamepadController(gamepad1);

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

        DcMotor shooterMotor = hardwareMap.dcMotor.get("shooter_motor");
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        boolean runShooterMotor = false;

        DcMotor intakeMotor = hardwareMap.dcMotor.get("intake_motor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        boolean runIntakeMotor = false;

        DcMotor armMotor = hardwareMap.dcMotor.get("arm_motor");
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition(0); // start in home position
        armMotor.setPower(1.0); // turn power on to the arm motor to always hold its position

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");

        Servo shooterServo = hardwareMap.servo.get("shooter_servo");
        Servo handServo = hardwareMap.servo.get("hand_servo");

        DistanceSensor frontDistance = hardwareMap.get(DistanceSensor.class, "front_distance");
        DistanceSensor leftDistance = hardwareMap.get(DistanceSensor.class, "left_distance");
        DistanceSensor rightDistance = hardwareMap.get(DistanceSensor.class, "right_distance");
        DistanceSensor backDistance = hardwareMap.get(DistanceSensor.class, "back_distance");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelerationIntegrationAlgorithm = null;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationData = null;
        parameters.calibrationDataFile = "";
        parameters.loggingEnabled = false;
        parameters.loggingTag = "Who cares.";

        imu.initialize(parameters);

        telemetry.addData("Press Start When Ready", "");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            controller.update();

            double px = gamepad1.left_stick_x;
            if (Math.abs(px) < 0.05) px = 0;
            double py = -gamepad1.left_stick_y;
            if (Math.abs(py) < 0.05) py = 0;
            double pa = -gamepad1.right_stick_x;
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

            if (controller.aPressed) {
                runShooterMotor = !runShooterMotor;
            }

            if (controller.xPressed) {
                runIntakeMotor = !runIntakeMotor;
            }

            shooterMotor.setPower(runShooterMotor ? 1.0 : 0.0);
            shooterServo.setPosition(gamepad1.right_trigger);
            intakeMotor.setPower((runIntakeMotor ? -1.0 : 0.0));
            handServo.setPosition(gamepad1.left_trigger);

            Double armPositionDeg = null;

            if (gamepad1.dpad_down) { // grab position
                armPositionDeg = WOBBLE_GOAL_GRAB_ANGLE_DEG;
            }
            else if (gamepad1.dpad_right) { // carry position
                armPositionDeg = WOBBLE_GOAL_CARRY_ANGLE_DEG;
            }
            else if (gamepad1.dpad_up) { // drop position
                armPositionDeg = WOBBLE_GOAL_DROP_ANGLE_DEG;
            }
            else if (gamepad1.dpad_left) { // home position
                armPositionDeg = WOBBLE_GOAL_HOME_ANGLE_DEG;
            }

            if (armPositionDeg != null) {
                armMotor.setTargetPosition(((int) (armPositionDeg / 360.0 * MotorType.Neverest60.TICKS_PER_ROTATION)));
                armMotor.setPower(1.0);
            }

            Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            telemetry.addData("Heading", " %.1f", orientation.firstAngle * 180.0 / Math.PI);
            telemetry.addData("Front Distance", " %.1f", frontDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Left Distance", " %.1f", leftDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Right Distance", " %.1f", rightDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Back Distance", " %.1f", backDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Encoders", " %d %d %d %d", m1.getCurrentPosition(), m2.getCurrentPosition(),
                    m3.getCurrentPosition(), m4.getCurrentPosition());
            telemetry.update();
        }

        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
        shooterMotor.setPower(0);
        intakeMotor.setPower(0);
    }
}
