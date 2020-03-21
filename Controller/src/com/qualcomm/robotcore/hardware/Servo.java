package com.qualcomm.robotcore.hardware;

import com.qualcomm.robotcore.hardware.HardwareDevice;

/**
 * Provides an subset of the functionality of the Servo interface in the FTC SDK.
 */
public interface Servo extends HardwareDevice {

    /**
     * The minimum and maximum positions to which a servo can be moved.
     */
    double MIN_POSITION = 0.0;
    double MAX_POSITION = 1.0;

    /**
     * Direction enum to allow software to internally reverse the values to which position is set.
     */
    enum Direction {FORWARD, REVERSE}

    /**
     * Method to set the direction
     * @param direction
     */
    void setDirection(Direction direction);

    /**
     * Method to get the direction
     * @return
     */
    Direction getDirection();

    /**
     * Set servo position
     * @param position Must be between 0 and 1
     */
    void setPosition(double position);

    /**
     * Get servo position
     * @return Current servo position
     */
    double getPosition();

    /**
     * Scale range to a pair of values between 0 and 1. After scaling the range, calls to setPosition
     * will still accept values between 0 and 1; getPosition will still return values between 0 and 1.
     * But the true position of the servo will be scaled into the range specified in the call to
     * scaleRange.
     */
    void scaleRange(double min, double max);

}
