package system.gui.viewelement.eventlistener;

/**
 * A special interface that allows event listeners that implement it to handle all events in the event heap.
 * <p>
 * Creation Date: 9/10/20
 *
 * @author Cole Savage, Level Up
 * @version 1.1.0
 * @see EventListener
 * @see system.gui.HALMenu
 * @since 1.1.0
 */
public interface UniversalUpdater {
    boolean updatesUniversally();
}