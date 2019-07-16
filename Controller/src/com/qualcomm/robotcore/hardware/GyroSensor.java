package com.qualcomm.robotcore.hardware;

import com.qualcomm.robotcore.hardware.HardwareDevice;

/**
 * Represents a simple Gyro Sensor.
 * The sensor must be initialized prior to use.
 */
public interface GyroSensor extends HardwareDevice {
    /**
     * Initialize the sensor.
     */
    public void init();

    /**
     * Get robot heading in degrees
     * @return robot heading (-180 to +180 degrees)
     */
    public double getHeading();
}
