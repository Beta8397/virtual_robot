package util.exceptions;

/**
 * An exception thrown when there is a general problem with something that was attempted in a HAL math utility.
 * <p>
 * Creation Date: 9/28/20
 *
 * @author Cole Savage, Level Up
 * @version 1.1.0
 * @see util.math
 * @see RuntimeException
 * @since 1.0.0
 */
public class HALMathException extends RuntimeException {

    /**
     * Constructor for HALMathException.
     *
     * @param message The message to print to the screen.
     */
    public HALMathException(String message) {
        super(message);
    }
}
