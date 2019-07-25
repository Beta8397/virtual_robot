package com.qualcomm.robotcore.hardware;

import com.qualcomm.robotcore.hardware.HardwareDevice;

/**
 * Provides an subset of the functionality of the Servo interface in the FTC SDK.
 */
public interface Servo extends HardwareDevice {
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
}
