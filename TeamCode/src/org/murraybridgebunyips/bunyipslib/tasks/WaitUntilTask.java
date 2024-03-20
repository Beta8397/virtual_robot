package org.murraybridgebunyips.bunyipslib.tasks;

import org.murraybridgebunyips.bunyipslib.tasks.bases.NoTimeoutTask;

import java.util.function.BooleanSupplier;

/**
 * A task that waits until a condition is true before finishing.
 * @author Lucas Bubner, 2024
 */
public class WaitUntilTask extends NoTimeoutTask {
    private final BooleanSupplier condition;

    public WaitUntilTask(BooleanSupplier condition) {
        this.condition = condition;
    }

    @Override
    protected void init() {
        // no-op
    }

    @Override
    protected void periodic() {
        // no-op
    }

    @Override
    protected void onFinish() {
        // no-op
    }

    @Override
    protected boolean isTaskFinished() {
        return condition.getAsBoolean();
    }
}
