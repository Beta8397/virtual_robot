package hardware;

/**
 * Provides an subset of the functionality of the Servo interface in the FTC SDK.
 */
public interface Servo {
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
