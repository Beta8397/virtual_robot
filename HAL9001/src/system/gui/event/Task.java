package system.gui.event;

/**
 * A functional interface used to allow EventListeners to run pieces of code when they are triggered.
 * <p>
 * Creation Date: 5/17/20
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see TaskPacket
 * @see DataPacket
 * @see system.gui.viewelement.eventlistener.EventListener
 * @see system.gui.HALMenu
 * @since 1.1.0
 */
@FunctionalInterface
public interface Task {

    /**
     * A method that will be overriden with code that the event listener should run.
     *
     * @param dataPacket A datapacket giving the code information about the event and event listener running the task.
     * @see DataPacket
     * @see Event
     * @see system.gui.viewelement.eventlistener.EventListener
     */
    void run(DataPacket dataPacket);
}
