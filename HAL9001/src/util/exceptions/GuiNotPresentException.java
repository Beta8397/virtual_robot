package util.exceptions;

/**
 * An exception thrown if a GUI is needed, but is not present.
 * <p>
 * Creation Date: 8/31/19
 *
 * @author Dylan Zueck, Crow Force
 * @version 1.0.0
 * @see RuntimeException
 * @since 1.0.0
 */
public class GuiNotPresentException extends RuntimeException {

    /**
     * Constructor for GuiNotPresentException
     *
     * @param message The error message to print to the screen.
     */
    public GuiNotPresentException(String message) {
        super(message);
    }
}