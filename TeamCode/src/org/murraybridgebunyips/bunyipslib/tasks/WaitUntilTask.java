package org.murraybridgebunyips.bunyipslib.tasks;

import org.murraybridgebunyips.bunyipslib.external.units.Measure;
import org.murraybridgebunyips.bunyipslib.external.units.Time;
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

import java.util.function.BooleanSupplier;

/**
 * A task that waits until a condition is true before finishing.
 *
 * @author Lucas Bubner, 2024
 */
public class WaitUntilTask extends Task {
    private final BooleanSupplier condition;

    /**
     * Create a new WaitUntilTask with a maximum timeout and given condition.
     *
     * @param timeout   max timeout
     * @param condition the condition to wait for true
     */
    public WaitUntilTask(Measure<Time> timeout, BooleanSupplier condition) {
        super(timeout);
        this.condition = condition;
        withName("Wait Until");
    }

    /**
     * Create a new WaitUntilTask with the given condition.
     *
     * @param condition the condition to wait for true
     */
    public WaitUntilTask(BooleanSupplier condition) {
        this.condition = condition;
        withName("Wait Until");
    }

    @Override
    protected void periodic() {
        // no-op
    }

    @Override
    protected boolean isTaskFinished() {
        return condition.getAsBoolean();
    }
}
