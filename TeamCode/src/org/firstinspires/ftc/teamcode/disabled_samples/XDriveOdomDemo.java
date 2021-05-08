package org.firstinspires.ftc.teamcode.disabled_samples;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeadWheelEncoder;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import virtual_robot.util.Vector2D;

/**
 * Example OpMode. Demonstrates use of gyro, color sensor, encoders, and telemetry.
 *
 */
@Disabled
@TeleOp(name = "XDriveOdom demo", group = "XBot")
public class XDriveOdomDemo extends LinearOpMode {
    private double wheelCircumference;
    private double wheelBaseRadius;

    private final double ENCODER_WHEEL_DIAMETER = 2.0;
    //Distances of right and left encoder wheels from robot centerline (i.e., the robot-X coordinates of the wheels)
    private final double Y_ENCODER_X = 6.0;
    //Distance of X-Encoder wheel from robot-X axis (i.e., the robot-Y coordinate of the wheel)
    private final double X_ENCODER_Y = -6.0;
    private double yEncoderX;
    private double xEncoderY;
    private double encoderWheelRadius;
    private final double ENCODER_TICKS_PER_REVOLUTION = 1120;
    private final double ENCODER_WHEEL_CIRCUMFERENCE = Math.PI * 1.5;
    public double[] pose = new double[2];
    public int[] prevTicks = new int[3];
    private DeadWheelEncoder encX;
    private DeadWheelEncoder encY;

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
//
//        //GyroSensor gyro = hardwareMap.gyroSensor.get("gyro_sensor");
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
//
//        CRServo backServo = hardwareMap.crservo.get("back_crservo");
        DistanceSensor frontDistance = hardwareMap.get(DistanceSensor.class, "front_distance");
        DistanceSensor leftDistance = hardwareMap.get(DistanceSensor.class, "left_distance");
        DistanceSensor rightDistance = hardwareMap.get(DistanceSensor.class, "right_distance");
        DistanceSensor backDistance = hardwareMap.get(DistanceSensor.class, "back_distance");
//        //gyro.init();
//


         encY = hardwareMap.get(DeadWheelEncoder.class, "y_enc");
         encX = hardwareMap.get(DeadWheelEncoder.class, "x_enc");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelerationIntegrationAlgorithm = null;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationData = null;
        parameters.calibrationDataFile = "";
        parameters.loggingEnabled = false;
        parameters.loggingTag = "Who cares.";

        imu.initialize(parameters);

        telemetry.addData("Press Start When Ready","");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()){
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
            double psrv = -gamepad2.left_stick_y;
            if (Math.abs(psrv) < 0.05) psrv = 0.0;
//            backServo.setPower(psrv);
            //telemetry.addData("Heading"," %.1f", gyro.getHeading());
            Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            telemetry.addData("Heading", " %.1f", orientation.firstAngle * 180.0 / Math.PI);
            telemetry.addData("Motor 1 Power", p1);
            telemetry.addData("Motor 2 Power", p2);
            telemetry.addData("Motor 3 Power", p3);
            telemetry.addData("Motor 4 Power", p4);
            telemetry.addData("Front Distance", " %.1f", frontDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Left Distance", " %.1f", leftDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Right Distance", " %.1f", rightDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Back Distance", " %.1f", backDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Encoders"," %d %d %d %d", m1.getCurrentPosition(), m2.getCurrentPosition(),
                    m3.getCurrentPosition(), m4.getCurrentPosition());
            updateOdometry(orientation);
            telemetry.addData("X Odom", pose[0]);
            telemetry.addData("Y Odom", pose[1]);

            telemetry.update();
        }
        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
    }

    public double[] updateOdometry(Orientation orientation){
        int[] ticks = new int[2];
        ticks[0] = encX.getCurrentPosition();
        ticks[1] = -encY.getCurrentPosition();
        int newXTicks = ticks[0] - prevTicks[0];
        int newYTicks = ticks[1] - prevTicks[1];
        prevTicks = ticks;
        Vector2D dr = new Vector2D(newXTicks, newYTicks);
        Vector2D df = dr.rotated(orientation.firstAngle).multiplied(ENCODER_WHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REVOLUTION);
        pose[0] += df.x;
        pose[1] += df.y;
//        double xDist = -newXTicks * ENCODER_WHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REVOLUTION;
//        double yDist = newYTicks * ENCODER_WHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REVOLUTION;
////        double curHeadingRad =
////        double cos = Math.cos(avgHeadingRadians);
////        double sin = Math.sin(avgHeadingRadians);
////        pose[0] += xDist*sin + yDist*cos;
////        pose[1] += -xDist*cos + yDist*sin;
////        pose[2] = AngleUtils.normalizeRadians(pose[2] + headingChangeRadians);
//        pose[0] += xDist;
//        pose[1] += yDist;
        return pose;
    }

}
