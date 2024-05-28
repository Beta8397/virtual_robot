package org.murraybridgebunyips.bunyipslib.tasks;

import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
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

    /**
     * Subsystem-dependent task.
     *
     * @param timeout    The time to run the task for
     * @param callback   The callback to run every loop
     * @param dependency The subsystem to run this task on
     * @param override   Whether this task should override conflicting tasks
     */
    public RunForTask(Measure<Time> timeout, Runnable callback, BunyipsSubsystem dependency, boolean override) {
        super(timeout, dependency, override);
        this.callback = callback;
        withName("Run For");
    }

    @Override
    protected void init() {
        // no-op
    }

    @Override
    protected void periodic() {
        callback.run();
    }

    @Override
    protected void onFinish() {
        // no-op
    }

    @Override
    protected boolean isTaskFinished() {
        // Timeout will handle this automatically
        return false;
    }
}
