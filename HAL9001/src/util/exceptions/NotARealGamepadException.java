package util.exceptions;

/**
 * An exception that is thrown when someone tries to use an invalid gamepad to control the robot.
 * <p>
 * Creation Date: 7/21/19
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see util.control.Button
 * @see util.control.CustomizableGamepad
 * @see RuntimeException
 * @since 1.0.0
 */
public class NotARealGamepadException extends RuntimeException {

    /**
     * Constructor for NotARealGamepadException.
     *
     * @param message The error message to print to the screen.
     */
    public NotARealGamepadException(String message) {
        super(message);
    }
}