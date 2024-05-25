package org.murraybridgebunyips.bunyipslib.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

/**
 * Alias for a {@link Cannon}, which is a generic servo switch.
 * May be more intuitive to use this alias in some cases.
 *
 * @see Cannon
 */
public class Switch extends Cannon {
    /**
     * Constructs a new Switch.
     *
     * @param servo         the servo to use
     * @param openPosition  the position to set the servo to when open
     * @param closePosition the position to set the servo to when closed
     */
    public Switch(Servo servo, double openPosition, double closePosition) {
        super(servo, openPosition, closePosition);
        withName("Switch");
    }

    /**
     * Constructs a new Switch with default open and close positions.
     *
     * @param servo the servo to use
     */
    public Switch(Servo servo) {
        super(servo);
        withName("Switch");
    }

    /**
     * Open the switch.
     */
    public void open() {
        fire();
    }

    /**
     * Close the switch.
     */
    public void close() {
        reset();
    }

    /**
     * Query whether the switch is open.
     *
     * @return whether the switch is open
     */
    public boolean isOpen() {
        return isFired();
    }

    /**
     * Query whether the switch is closed.
     *
     * @return whether the switch is closed
     */
    public boolean isClosed() {
        return isReset();
    }

    /**
     * Open the switch.
     *
     * @return a task that opens the switch
     */
    public Task openTask() {
        return fireTask();
    }

    /**
     * Close the switch.
     *
     * @return a task that closes the switch
     */
    public Task closeTask() {
        return resetTask();
    }
}
