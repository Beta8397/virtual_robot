package util.exceptions;

/**
 * An exception thrown if something really bad happens.
 * <p>
 * Creation Date: 7/29/19
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see RuntimeException
 * @since 1.0.0
 */
public class DumpsterFireException extends RuntimeException {

    /**
     * Constructor for DumpsterFireException.
     *
     * @param message The error message to print to the screen.
     */
    public DumpsterFireException(String message) {
        super(message);
    }
}
