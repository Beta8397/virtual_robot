package util.exceptions;

/**
 * An exception thrown if a drive train is given an invalid movement command.
 * <p>
 * Creation Date: 9/5/19
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see system.robot.subsystems.MechanumDrive
 * @see system.robot.subsystems.OmniWheelDrive
 * @see system.robot.subsystems.TankDrive
 * @see system.robot.subsystems.QuadWheelDrive
 * @see RuntimeException
 * @since 1.0.0
 */
public class InvalidMoveCommandException extends RuntimeException {

    /**
     * Constructor for InvalidMoveCommandException.
     *
     * @param message The error message to print to the screen.
     */
    public InvalidMoveCommandException(String message) {
        super(message);
    }
}