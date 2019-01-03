package virtual_robot.hardware;

/**
 * Represents a simple Gyro Sensor.
 * The sensor must be initialized prior to use.
 */
public interface GyroSensor {
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
