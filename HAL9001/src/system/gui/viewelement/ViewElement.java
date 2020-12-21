package system.gui.viewelement;

/**
 * The base interface for all ViewElements used in HAL Menus.
 * <p>
 * Creation Date: 4/29/20
 *
 * @author Cole Savage, Level Up
 * @version 1.1.0
 * @see system.gui.HALMenu
 * @see TextElement
 * @see system.gui.viewelement.eventlistener.EventListener
 * @since 1.1.0
 */
public interface ViewElement {

    /**
     * Gets the text associated with the ViewElement.
     *
     * @return The text associated with the ViewElement.
     */
    String getText();

    /**
     * Sets the text associated with the ViewElement.
     *
     * @param text The text to be associated with the ViewElement.
     */
    void setText(String text);
}
