package hardware;

/**
 * Provides an subset of the functionality of the Servo interface in the FTC SDK.
 */
public interface Servo {
    void setPosition(double position);
    double getPosition();
}
