package org.murraybridgebunyips.bunyipslib.example.examplerobot.components;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.RelativePose2d;

import java.util.Locale;

/**
 * Example code for a drive system using the BunyipsOpMode ecosystem.
 */
// Extend the `BunyipsSubsystem` class when making a new component, as shown below.
public class ExampleMecanumDrive extends BunyipsSubsystem {

    // View ExampleDrive.java for more information on how to make a component.

    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;

    // Store the desired power of the motors in variables
    private double speedX;
    private double speedY;
    private double speedR;
    // Store and declare prioritisation when given instruction to calculate motor powers
    private Priority priority = Priority.NORMALISED;

    public ExampleMecanumDrive(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight) {
        // Assign constructor parameters to the ExampleMecanumDrive instance
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
    }

    // Setters for the prioritisation of the drive system
    public void setPriority(Priority priority) {
        this.priority = priority;
    }

    public void swapPriority() {
        priority = priority == Priority.NORMALISED ? Priority.ROTATIONAL : Priority.NORMALISED;
    }

    /**
     * Set a speed at which the Mecanum drive assembly should move.
     *
     * @param x The speed at which the robot should move in the x direction.
     *          Positive is right, negative is left.
     *          Range: -1.0 to 1.0
     * @param y The speed at which the robot should move in the -y direction.
     *          Positive is backward, negative is forward.
     *          Range: -1.0 to 1.0
     * @param r The speed at which the robot will rotate.
     *          Positive is clockwise, negative is anti-clockwise.
     *          Range: -1.0 to 1.0
     */
    public void setSpeedXYR(double x, double y, double r) {
        // X and Y have been swapped, and X has been inverted
        // This rotates input vectors by 90 degrees clockwise and wil account for gamepad input.
        speedX = Range.clip(y, -1.0, 1.0);
        speedY = Range.clip(-x, -1.0, 1.0);
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
        }

        // Calculate motor powers
        double frontLeftPower = speedX + speedY - speedR;
        double frontRightPower = speedX - speedY + speedR;
        double backLeftPower = speedX - speedY - speedR;
        double backRightPower = speedX + speedY + speedR;

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

        opMode.addTelemetry(String.format(Locale.getDefault(), "Mecanum Drive: Forward: %.2f, Strafe: %.2f, Rotate: %.2f", speedX, speedY, speedR), false);
    }

    private void rotationalUpdate() {
        // Calculate translational speeds
        double[] translationValues = {
                speedX + speedY,
                speedX - speedY,
                speedX - speedY,
                speedX + speedY
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

    /**
     * Set motor speeds based on a Pose2d or RelativePose2d.
     */
    public <T> void setVector(T vector) {
        if (vector instanceof Pose2d) {
            Pose2d robotVector = (Pose2d) vector;
            setSpeedXYR(robotVector.getX(), -robotVector.getY(), robotVector.getHeading());
        } else if (vector instanceof RelativePose2d) {
            RelativePose2d relativeVector = (RelativePose2d) vector;
            setSpeedXYR(relativeVector.getVector().getX(), -relativeVector.getVector().getY(), relativeVector.getVector().getHeading());
        }
    }

    /**
     * Prioritisation of the drive system.
     * NORMALISED: The drive system calculate the motor powers with equal priority to each desired speed.
     * ROTATIONAL: The drive system will calculate rotational speed first, and use remaining headway for translation.
     */
    enum Priority {
        NORMALISED, ROTATIONAL
    }

}
