package org.firstinspires.ftc.teamcode.ftc16072;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.ftc16072.Util.Polar;
import org.firstinspires.ftc.teamcode.ftc16072.Util.RobotPosition;


public class Navigation {
    static double DISTANCE_TOLARANCE = 2;
    static double ANGLE_TOLARANCE = AngleUnit.RADIANS.fromDegrees(1);
    static double KP_DISTANCE = 0.02;
    static double KP_ANGLE = 1;
    MecanumDrive mecanumDrive = new MecanumDrive();
    BNO055IMU imu;
    private RobotPosition lastSetPosition;

    void init(HardwareMap hwMap) {
        imu = hwMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        imu.initialize(params);
        mecanumDrive.init(hwMap);
        setPosition(0, 0, DistanceUnit.CM);
    }

    public double getHeading(AngleUnit angleUnit) {
        Orientation angles;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, angleUnit);
        return angles.firstAngle;

    }

    public void driveFieldRelative(double x, double y, double rotate) {
        Polar drive = Polar.fromCartesian(x, y);
        double heading = getHeading(AngleUnit.RADIANS);

        drive.subtractAngle(heading);
        mecanumDrive.driveMecanum(drive.getX(), -drive.getY(), rotate);
    }

    public void strafe(double speed) {
        mecanumDrive.driveMecanum(0, speed, 0);
    }

    public void driveFieldRelativeAngle(double x, double y, double angle) {
        double delta = angle - getHeading(AngleUnit.RADIANS);
        if (delta >= Math.PI) {
            delta = delta - (2 * Math.PI);
        } else if (delta <= -Math.PI) {
            delta = delta + (2 * Math.PI);
        }
        double MAX_ROTATE = 0.7; //This is to shrink how fast we can rotate so we don't fly past the angle
        delta = Range.clip(delta, -MAX_ROTATE, MAX_ROTATE);
        driveFieldRelative(x, y, delta);
    }


    public void setMecanumDriveMaxSpeed(double speed) {
        mecanumDrive.setMaxSpeed(speed);
    }

    public double getMecanumDriveMaxSpeed() {
        return mecanumDrive.getMaxSpeed();
    }

    public RobotPosition getEstimatedPosition() {
        double[] distanceDriven = mecanumDrive.getDistanceCm();
        Polar translation = Polar.fromCartesian(distanceDriven[1], distanceDriven[0]);
        double rotate = getHeading(AngleUnit.RADIANS);
        translation.subtractAngle(-rotate);

        double estX = lastSetPosition.getX(DistanceUnit.CM) + translation.getX();
        double estY = lastSetPosition.getY(DistanceUnit.CM) + translation.getY();

        return new RobotPosition(estX, estY, DistanceUnit.CM, rotate, AngleUnit.RADIANS);
    }

    public void setPosition(double x, double y, DistanceUnit distanceUnit) {
        lastSetPosition = new RobotPosition(x, y, distanceUnit, getHeading(AngleUnit.RADIANS), AngleUnit.RADIANS);
        mecanumDrive.setEncoderOffsets();
    }

    public boolean rotateTo(double angle, AngleUnit angleUnit) {
        double rotateSpeed = 0.0;

        double rotateDiff = angleUnit.toRadians(angle) - getHeading(AngleUnit.RADIANS);
        if (Math.abs(rotateDiff) <= ANGLE_TOLARANCE) {
            mecanumDrive.driveMecanum(0, 0, 0);
            return true;
        } else {
            rotateSpeed = KP_ANGLE * rotateDiff;
        }
        mecanumDrive.driveMecanum(0, 0, rotateSpeed);
        return false;
    }

    public boolean driveTo(double x, double y, DistanceUnit distanceUnit) {
        RobotPosition estimatedPosition = getEstimatedPosition();
        double xDiff = distanceUnit.toCm(x) - estimatedPosition.getX(DistanceUnit.CM);
        double yDiff = distanceUnit.toCm(y) - estimatedPosition.getY(DistanceUnit.CM);

        double xSpeed = 0.0;
        double ySpeed = 0.0;

        if ((Math.abs(xDiff) <= DISTANCE_TOLARANCE) &&
                (Math.abs(yDiff) <= DISTANCE_TOLARANCE)) {
            mecanumDrive.driveMecanum(0, 0, 0);
            setPosition(distanceUnit.toCm(x), distanceUnit.toCm(y), DistanceUnit.CM);
            return true;
        }
        if (Math.abs(xDiff) > DISTANCE_TOLARANCE) {
            xSpeed = KP_DISTANCE * xDiff;
        }
        if (Math.abs(yDiff) > DISTANCE_TOLARANCE) {
            ySpeed = KP_DISTANCE * yDiff;
        }
        System.out.printf("Diff: %.02f %.02f\n", yDiff, xDiff);
        System.out.printf("--Speed: %.02f %.02f\n", ySpeed, xSpeed);
        driveFieldRelative(xSpeed, ySpeed, 0.0);

        return false;
    }

}
