package org.murraybridgebunyips.bunyipslib.tasks;

import org.murraybridgebunyips.bunyipslib.tasks.bases.ForeverTask;

/**
 * A task to run continuously and will never finish.
 * <p>
 * {@code new ContinuousTask(() -> telemetry.add("I will run forever"));}
 */
public class ContinuousTask extends ForeverTask {
    private final Runnable callback;

    /**
     * Subsystem-independent continuous task.
     *
     * @param callback the task to run continuously
     */
    public ContinuousTask(Runnable callback) {
        this.callback = callback;
        withName("Continuous");
    }

    @Override
    protected void periodic() {
        callback.run();
    }
}
