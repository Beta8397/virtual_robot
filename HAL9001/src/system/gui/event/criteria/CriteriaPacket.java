package system.gui.event.criteria;

import system.gui.event.Event;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

/**
 * A packet of EventCriteria for various events.
 * <p>
 * Creation Date: 5/17/20
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see EventCriteria
 * @see Event
 * @see system.gui.HALMenu
 * @see system.gui.HALGUI
 * @see Iterable
 * @since 1.1.0
 */
public final class CriteriaPacket implements Iterable<EventCriteria<?>> {
    //A list containing all of the event criteria in the packet.
    private List<EventCriteria<? extends Event>> criteriaList;

    /**
     * A constructor for the criteria packet.
     */
    public CriteriaPacket() {
        criteriaList = new ArrayList<>();
    }

    /**
     * Adds an EventCriteria to the packet.
     *
     * @param criteria The EventCriteria to add.
     * @param <T>      The event the EventCriteria is associated with.
     * @see Event
     * @see EventCriteria
     */
    public final <T extends Event> void add(EventCriteria<T> criteria) {
        criteriaList.add(criteria);
    }

    @Override
    public Iterator<EventCriteria<?>> iterator() {
        return criteriaList.iterator();
    }
}