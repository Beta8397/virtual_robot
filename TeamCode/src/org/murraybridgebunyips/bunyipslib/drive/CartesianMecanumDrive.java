package org.murraybridgebunyips.bunyipslib.drive;

import static org.murraybridgebunyips.bunyipslib.external.units.Units.Radians;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.Controls;
import org.murraybridgebunyips.bunyipslib.external.units.Angle;
import org.murraybridgebunyips.bunyipslib.external.units.Measure;

import java.util.Locale;

/**
 * Class for a Cartesian drive system that uses Mecanum wheels.
 * Includes all the common math done across all Mecanum drive systems.
 * <p>
 * This system is deprecated by RoadRunner, but remains for legacy/non-RoadRunner purposes.
 * Coordinates used in Cartesian drives include a Cartesian speed mapping, where X is the horizontal axis, and
 * Y is the forward axis. This is rotated differently within RoadRunner-derived drive bases.
 *
 * @author Lucas Bubner, 2023
 * @see MecanumDrive
 */
public class CartesianMecanumDrive extends BunyipsSubsystem {
    // Fields left public for legacy reasons
    /**
     * Horizontal speed of the robot.
     */
    public double speedX;
    /**
     * Vertical speed of the robot.
     */
    public double speedY;
    /**
     * Rotational speed of the robot.
     */
    public double speedR;

    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;

    private Priority priority = Priority.NORMALISED;

    /**
     * @param frontLeft  the front left motor
     * @param frontRight the front right motor
     * @param backLeft   the back left motor
     * @param backRight  the back right motor
     */
    public CartesianMecanumDrive(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight) {
        if (!assertParamsNotNull(frontLeft, frontRight, backLeft, backRight)) return;
        this.frontLeft = frontLeft;
        this.backLeft = backLeft;
        this.frontRight = frontRight;
        this.backRight = backRight;
        setToBrake();
    }

    // Setters for the prioritisation of the drive system
    public CartesianMecanumDrive setPriority(Priority priority) {
        this.priority = priority;
        return this;
    }

    /**
     * Swap the priority of the drive system.
     *
     * @return this
     */
    public CartesianMecanumDrive swapPriority() {
        priority = priority == Priority.NORMALISED ? Priority.ROTATIONAL : Priority.NORMALISED;
        return this;
    }

    /**
     * Set a speed at which the Mecanum drive assembly should move using controller input.
     * The difference is that the vectors will be negated properly for the gamepad.
     *
     * @param left_stick_x  X value of the controller
     * @param left_stick_y  Y value of the controller
     * @param right_stick_x R value of the controller
     * @return this
     * @see Controls#Companion
     */
    public CartesianMecanumDrive setSpeedUsingController(double left_stick_x, double left_stick_y, double right_stick_x) {
        setSpeedXYR(left_stick_x, -left_stick_y, right_stick_x);
        return this;
    }

    /**
     * Set a speed at which the Mecanum drive assembly should move.
     *
     * @param x The speed at which the robot should move in the x direction.
     *          Positive is right, negative is left.
     *          Range: -1.0 to 1.0
     * @param y The speed at which the robot should move in the y direction.
     *          Positive is forward, negative is backward.
     *          Range: -1.0 to 1.0
     * @param r The speed at which the robot will rotate.
     *          Positive is clockwise, negative is anti-clockwise.
     *          Range: -1.0 to 1.0
     * @return this
     */
    public CartesianMecanumDrive setSpeedXYR(double x, double y, double r) {
        speedX = Range.clip(x, -1.0, 1.0);
        speedY = Range.clip(y, -1.0, 1.0);
        speedR = Range.clip(r, -1.0, 1.0);
        return this;
    }

    /**
     * Set a polar speed at which the Mecanum drive assembly should move.
     *
     * @param speed     speed at which the motors will operate
     * @param direction direction at which the motors will move toward
     * @param rSpeed    rotation speed - positive: clockwise
     * @return this
     */
    public CartesianMecanumDrive setSpeedPolarR(double speed, Measure<Angle> direction, double rSpeed) {
        setSpeedXYR(
                speed * Math.cos(direction.in(Radians)),
                speed * Math.sin(direction.in(Radians)),
                rSpeed
        );
        return this;
    }

    /**
     * Update and reflect the speed of the drive system to the actual motors, and
     * calculate the motor powers based on these variables.
     */
    @Override
    protected void periodic() {
        if (priority == Priority.ROTATIONAL) {
            rotationalUpdate();
            opMode.telemetry.add(String.format(Locale.getDefault(), "Rotation-priority Mecanum Drive: Forward: %.2f, Strafe: %.2f, Rotate: %.2f", speedX, speedY, speedR));
            return;
        }

        // Calculate motor powers
        double frontLeftPower = speedY + speedX + speedR;
        double frontRightPower = speedY - speedX - speedR;
        double backLeftPower = speedY - speedX + speedR;
        double backRightPower = speedY + speedX - speedR;

        double maxPower = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower), Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));
        // If the maximum number is greater than 1.0, then normalise by that number
        if (maxPower > 1.0) {
            frontLeftPower = frontLeftPower / maxPower;
            frontRightPower = frontRightPower / maxPower;
            backLeftPower = backLeftPower / maxPower;
            backRightPower = backRightPower / maxPower;
        }

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        opMode.telemetry.add(String.format(Locale.getDefault(), "Mecanum Drive: X: %.2f, Y: %.2f, R: %.2f", speedX, speedY, speedR));
    }

    /**
     * Immediately stop the drive system.
     */
    public void stop() {
        speedX = 0.0;
        speedY = 0.0;
        speedR = 0.0;
        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
        update();
    }

    /**
     * Set the drive system to brake.
     *
     * @return this
     */
    public CartesianMecanumDrive setToBrake() {
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return this;
    }

    /**
     * Set the drive system to float.
     *
     * @return this
     */
    public CartesianMecanumDrive setToFloat() {
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        return this;
    }

    private CartesianMecanumDrive rotationalUpdate() {
        // Calculate translational speeds
        double[] translationValues = {
                speedY + speedX,
                speedY - speedX,
                speedY - speedX,
                speedY + speedX
        };

        double[] rotationValues = {
                -speedR,
                speedR,
                -speedR,
                speedR
        };

        double scaleFactor = 1.0;
        double tmpScale = 1.0;

        // Solve this equation backwards
        // MotorX = TranslationX * scaleFactor + RotationX
        // to find scaleFactor that ensures -1 <= MotorX <= 1 and 0 < scaleFactor <= 1
        for (int i = 0; i < 4; i++) {
            if (Math.abs(translationValues[i] + rotationValues[i]) > 1) {
                tmpScale = (1 - rotationValues[i]) / translationValues[i];
            } else if (translationValues[i] + rotationValues[i] < -1) {
                tmpScale = (rotationValues[i] - 1) / translationValues[i];
            }
            if (tmpScale < scaleFactor) {
                scaleFactor = tmpScale;
            }
        }

        double frontLeftPower = translationValues[0] * scaleFactor + rotationValues[0];
        double frontRightPower = translationValues[1] * scaleFactor + rotationValues[1];
        double backLeftPower = translationValues[2] * scaleFactor + rotationValues[2];
        double backRightPower = translationValues[3] * scaleFactor + rotationValues[3];

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
        return this;
    }

    /**
     * Mecanum control prioritisation.
     */
    public enum Priority {
        /**
         * Calculate translational speeds first.
         */
        NORMALISED,
        /**
         * Calculate rotational speeds first, and use the rest for translation.
         */
        ROTATIONAL
    }
}

