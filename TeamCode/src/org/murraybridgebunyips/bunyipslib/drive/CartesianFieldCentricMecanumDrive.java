package org.murraybridgebunyips.bunyipslib.drive;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.murraybridgebunyips.bunyipslib.Controller;
import org.murraybridgebunyips.bunyipslib.IMUOp;
import org.murraybridgebunyips.bunyipslib.RelativePose2d;

/**
 * Variant of the Cartesian MecanumDrive that uses field-centric controls, accounting for robot heading.
 * This system is deprecated by RoadRunner, but remains for legacy/non-RoadRunner purposes.
 *
 * @author Lucas Bubner, 2023
 * @see MecanumDrive
 */
public class CartesianFieldCentricMecanumDrive extends CartesianMecanumDrive {
    private final IMUOp imu;

    /**
     * Constructs a new CartesianFieldCentricMecanumDrive.
     *
     * @param frontLeft                 the front left motor
     * @param frontRight                the front right motor
     * @param backLeft                  the back left motor
     * @param backRight                 the back right motor
     * @param imu                       the IMU to use
     * @param invalidatePreviousHeading whether to invalidate the previous heading of the IMU
     * @param startingDirection         the starting direction of the robot
     */
    public CartesianFieldCentricMecanumDrive(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight, IMUOp imu, boolean invalidatePreviousHeading, RelativePose2d startingDirection) {
        super(frontLeft, frontRight, backLeft, backRight);
        assertParamsNotNull(imu);

        this.imu = imu;
        if (startingDirection == RelativePose2d.CLOCKWISE || startingDirection == RelativePose2d.ANTICLOCKWISE) {
            throw new IllegalArgumentException("FCMD: Cannot use rotational quantities as a starting direction");
        }

        // Invalidate any previous readings
        if (invalidatePreviousHeading)
            imu.resetHeading();

        // Current vector will be the robot's starting vector, must offset the IMU to align straight
        imu.setOffset(startingDirection.getDegrees());
    }

    /**
     * Set a speed at which the Mecanum drive assembly should move using controller input.
     * This method will also account for the robot's current heading and adjust the vectors
     * accordingly. Note that the IMU will be internally ticked by this method, so imu.tick()
     * does not need to be called.
     * The difference is that the vectors will be rotated by 90 degrees clockwise to account for
     * the strange non-Cartesian coordinate system of the gamepad.
     *
     * @param left_stick_x  X value of the controller, will be interpreted as relative to the field
     * @param left_stick_y  Y value of the controller, will be interpreted as relative to the field
     * @param right_stick_x R value of the controller, will be interpreted as relative to the field
     * @see Controller#Companion
     */
    @Override
    public CartesianFieldCentricMecanumDrive setSpeedUsingController(double left_stick_x, double left_stick_y, double right_stick_x) {
        imu.update();

        // Account for the rotated vector of the gamepad
        double heading = imu.getRawHeading() - 90;
        double x = -left_stick_x;

        double sin = Math.sin(Math.toRadians(heading));
        double cos = Math.cos(Math.toRadians(heading));

        // Transform the x and y values to be relative to the field
        // This is done by calculating the current heading to the field then rotating the x
        // and y vectors to be relative to the field, then updating the motor powers as normal
        speedY = x * cos + left_stick_y * sin;
        speedX = x * sin - left_stick_y * cos;
        speedR = right_stick_x;

        return this;
    }

    /**
     * Set a speed at which the Mecanum drive assembly should move.
     * This method will also account for the robot's current heading and adjust the vectors
     * accordingly. Note that the IMU will be internally ticked by this method, so imu.tick()
     * does not need to be called.
     *
     * @param x The speed at which the robot should move in the x direction relative to the field.
     *          Positive is right, negative is left.
     *          Range: -1.0 to 1.0
     * @param y The speed at which the robot should move in the y direction relative to the field.
     *          Positive is forward, negative is backward.
     *          Range: -1.0 to 1.0
     * @param r The speed at which the robot will rotate.
     *          Positive is clockwise, negative is anti-clockwise.
     *          Range: -1.0 to 1.0
     */
    @Override
    public CartesianFieldCentricMecanumDrive setSpeedXYR(double x, double y, double r) {
        imu.update();

        double heading = imu.getRawHeading();
        double xn = -x;

        double sin = Math.sin(Math.toRadians(heading));
        double cos = Math.cos(Math.toRadians(heading));

        speedX = -(xn * cos + y * sin);
        speedY = -(xn * sin - y * cos);
        speedR = r;

        return this;
    }

    /**
     * Set a polar speed at which the Mecanum drive assembly should move.
     * This method will also account for the robot's current heading and adjust the vectors
     * accordingly. Note that the IMU will be internally ticked by this method, so imu.tick()
     * does not need to be called.
     *
     * @param speed             speed at which the motors will operate
     * @param direction_degrees direction at which the motors will move toward relative to the field
     * @param rSpeed            rotation speed - positive: clockwise
     */
    @Override
    public CartesianFieldCentricMecanumDrive setSpeedPolarR(double speed, double direction_degrees, double rSpeed) {
        imu.update();

        double heading = imu.getRawHeading();
        double direction = Math.toRadians(direction_degrees);

        double sin = Math.sin(Math.toRadians(heading));
        double cos = Math.cos(Math.toRadians(heading));

        speedX = -(speed * Math.cos(direction) * cos + speed * Math.sin(direction) * sin);
        speedY = -(speed * Math.cos(direction) * sin - speed * Math.sin(direction) * cos);
        speedR = rSpeed;

        return this;
    }
}
