package org.murraybridgebunyips.bunyipslib.tasks;

import org.murraybridgebunyips.bunyipslib.external.units.Measure;
import org.murraybridgebunyips.bunyipslib.external.units.Time;
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

/**
 * A task to run for a timeout.
 * <p>
 * {@code new RunForTask(5, () -> arm.spin());}
 */
public class RunForTask extends Task {
    private final Runnable callback;

    /**
     * Subsystem-independent task.
     *
     * @param timeout  The time to run the task for
     * @param callback The callback to run every loop
     */
    public RunForTask(Measure<Time> timeout, Runnable callback) {
        super(timeout);
        this.callback = callback;
        withName("Run For");
    }

    @Override
    protected void periodic() {
        // no-op
    }

    @Override
    protected boolean isTaskFinished() {
        // Timeout will handle this automatically
        return false;
    }
}
