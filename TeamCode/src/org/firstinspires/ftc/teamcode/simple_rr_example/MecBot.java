package org.firstinspires.ftc.teamcode.simple_rr_example;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.*;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * A simple mecanum bot class for use with Acme Robotics RoadRunner. This class uses the mecanum drive wheels for
 * odometry (localization).
 */
public class MecBot extends MecanumDrive {

    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(1, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(1, 0, 0);
    public static final double TICKS_PER_REV = 1120;
    public static final double MAX_RPM = 133.9;
    public static double WHEEL_DIAM = 4;
    public static double TRACK_WIDTH = 18;         //Effective Track Width
    public static double MAX_VELOCITY = MAX_RPM * Math.PI * WHEEL_DIAM / 60.0;
    public static double kV = 1.0 / MAX_VELOCITY;
    public static double kA = 0;
    public static double kStatic = 0;

    public TrajectoryVelocityConstraint velocityConstraint = new MinVelocityConstraint(Arrays.asList(
            new AngularVelocityConstraint(60),
            new TranslationalVelocityConstraint(25)
    ));
    public TrajectoryAccelerationConstraint accelConstraint = new ProfileAccelerationConstraint(25);
    private PIDFController turnController;
    public TrajectoryFollower follower;

    BNO055IMU imu;
    DcMotor leftFront, leftRear, rightRear, rightFront;


    public MecBot(HardwareMap hardwareMap){
        super(kV, kA, kStatic, TRACK_WIDTH);

        turnController = new PIDFController(HEADING_PID);
        turnController.setInputBounds(0, 2 * Math.PI);
        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        leftFront = hardwareMap.get(DcMotor.class, "front_left_motor");
        leftRear = hardwareMap.get(DcMotor.class, "back_left_motor");
        rightRear = hardwareMap.get(DcMotor.class, "back_right_motor");
        rightFront = hardwareMap.get(DcMotor.class, "front_right_motor");

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

    }



    public static double ticksToInches(double ticks) {
        return WHEEL_DIAM * Math.PI * ticks / TICKS_PER_REV;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        wheelPositions.add(ticksToInches(leftFront.getCurrentPosition()));
        wheelPositions.add(ticksToInches(leftRear.getCurrentPosition()));
        wheelPositions.add(ticksToInches(rightRear.getCurrentPosition()));
        wheelPositions.add(ticksToInches(rightFront.getCurrentPosition()));
        return wheelPositions;
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }
}
