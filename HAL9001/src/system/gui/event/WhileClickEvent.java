package system.gui.event;

import util.control.Button;

/**
 * An event that is injected while a boolean button is pressed.
 * <p>
 * Creation Date: 5/17/20
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see ClickEvent
 * @see Event
 * @see GamepadEventGenerator
 * @see Button
 * @since 1.1.0
 */
public class WhileClickEvent extends ClickEvent<Button<Boolean>> {

    /**
     * The constructor for WhileClickEvent.
     *
     * @param priority The event's priority.
     * @param button   The button being held down.
     */
    public WhileClickEvent(int priority, Button<Boolean> button) {
        super(priority, button);
    }
}