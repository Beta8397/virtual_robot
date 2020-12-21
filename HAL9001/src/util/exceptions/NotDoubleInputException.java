package util.exceptions;

/**
 * An exception thrown when a button meant to return double data is mapped to a button that does not return double data.
 * <p>
 * Creation Date: 7/20/19
 *
 * @author Dylan Zueck, Crow Force
 * @version 1.0.0
 * @see util.control.Button
 * @see util.control.CustomizableGamepad
 * @see RuntimeException
 * @since 1.0.0
 */
public class NotDoubleInputException extends RuntimeException {

    /**
     * Constructor for NotDoubleInputException.
     *
     * @param message The message to print to the screen.
     */
    public NotDoubleInputException(String message) {
        super(message);
    }
}