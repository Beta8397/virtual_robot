package com.qualcomm.robotcore.hardware;

/**
 * Represents a simple DC Motor
 */
public interface DcMotorSimple extends HardwareDevice {

    /**
     * DcMotors can be configured to internally reverse the values
     * to which, e.g., their motor power is set. This makes it easy
     * to have drive train motors on two sides of a robot: during initialization,
     * one would be set at at forward, the other at reverse, and the
     * difference between the two in that respect could be thereafter ignored.
     *
     */
    enum Direction { FORWARD, REVERSE;
        public Direction inverted() {
            return this==FORWARD ? REVERSE : FORWARD;
        }
    }

    /**
     * Sets the logical direction in which this motor operates.
     * @param direction the direction to set for this motor
     *
     */
    void setDirection(Direction direction);

    /**
     * Returns the current logical direction in which this motor is set as operating.
     * @return the current logical direction in which this motor is set as operating.
     */
    Direction getDirection();

    /**
     * Sets the power level of the motor, expressed as a fraction of the maximum
     * possible power / speed supported according to the run mode in which the
     * motor is operating.
     *
     * <p>Setting a power level of zero will brake the motor</p>
     *
     * @param power the new power level of the motor, a value in the interval [-1.0, 1.0]
     */
    void setPower(double power);

    /**
     * Returns the current configured power level of the motor.
     * @return the current level of the motor, a value in the interval [0.0, 1.0]
     */
    double getPower();
}
