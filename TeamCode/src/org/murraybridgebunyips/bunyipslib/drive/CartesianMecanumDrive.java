package org.murraybridgebunyips.bunyipslib.drive;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.murraybridgebunyips.bunyipslib.BunyipsOpMode;
import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.Controller;

import java.util.Locale;

/**
 * Base class for a Cartesian drive system that uses Mecanum wheels.
 * Includes all the common math done across all Mecanum drive systems.
 * This system is deprecated by RoadRunner, but remains for legacy/non-RoadRunner purposes.
 *
 * @author Lucas Bubner, 2023
 * @see MecanumDrive
 */
public class CartesianMecanumDrive extends BunyipsSubsystem {

    private final DcMotor frontLeft;
    private final DcMotor backLeft;
    private final DcMotor frontRight;
    private final DcMotor backRight;

    // Axial translation speeds
    public double speedX;
    public double speedY;
    public double speedR;

    // Store and declare prioritisation when given instruction to calculate motor powers
    private Priority priority = Priority.NORMALISED;

    public CartesianMecanumDrive(@NonNull BunyipsOpMode opMode, DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight) {
        super(opMode);
        this.frontLeft = frontLeft;
        this.backLeft = backLeft;
        this.frontRight = frontRight;
        this.backRight = backRight;
        // Critical component, cannot be ignored if null
        assert frontLeft != null && backLeft != null && frontRight != null && backRight != null;
    }

    // Setters for the prioritisation of the drive system
    public void setPriority(Priority priority) {
        this.priority = priority;
    }

    public void swapPriority() {
        priority = priority == Priority.NORMALISED ? Priority.ROTATIONAL : Priority.NORMALISED;
    }

    /**
     * Set a speed at which the Mecanum drive assembly should move using controller input.
     * The difference is that the vectors will be negated properly for the gamepad.
     *
     * @param left_stick_x  X value of the controller
     * @param left_stick_y  Y value of the controller
     * @param right_stick_x R value of the controller
     * @see Controller#Companion
     */
    public void setSpeedUsingController(double left_stick_x, double left_stick_y, double right_stick_x) {
        speedX = Range.clip(left_stick_x, -1.0, 1.0);
        speedY = Range.clip(-left_stick_y, -1.0, 1.0);
        speedR = Range.clip(right_stick_x, -1.0, 1.0);
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
     */
    public void setSpeedXYR(double x, double y, double r) {
        speedX = Range.clip(x, -1.0, 1.0);
        speedY = Range.clip(y, -1.0, 1.0);
        speedR = Range.clip(r, -1.0, 1.0);
    }

    /**
     * Set a polar speed at which the Mecanum drive assembly should move.
     *
     * @param speed             speed at which the motors will operate
     * @param direction_degrees direction at which the motors will move toward
     * @param speedR            rotation speed - positive: clockwise
     */
    public void setSpeedPolarR(double speed, double direction_degrees, double speedR) {
        double radians = Math.toRadians(direction_degrees);
        speedX = Range.clip(speed * Math.cos(radians), -1.0, 1.0);
        speedY = Range.clip(speed * Math.sin(radians), -1.0, 1.0);
        this.speedR = Range.clip(speedR, -1.0, 1.0);
    }

    /**
     * Update and reflect the speed of the drive system to the actual motors, and
     * calculate the motor powers based on these variables.
     */
    public void update() {
        if (priority == Priority.ROTATIONAL) {
            rotationalUpdate();
            getOpMode().addTelemetry(String.format(Locale.getDefault(), "Rotation-priority Mecanum Drive: Forward: %.2f, Strafe: %.2f, Rotate: %.2f", speedX, speedY, speedR));
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

        getOpMode().addTelemetry(String.format(Locale.getDefault(), "Mecanum Drive: X: %.2f, Y: %.2f, R: %.2f", speedX, speedY, speedR));
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
     */
    public void setToBrake() {
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Set the drive system to float.
     */
    public void setToFloat() {
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    private void rotationalUpdate() {
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
    }

    enum Priority {
        NORMALISED, ROTATIONAL
    }
}

