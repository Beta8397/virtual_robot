package org.murraybridgebunyips.bunyipslib.tasks;

import org.murraybridgebunyips.bunyipslib.external.units.Measure;
import org.murraybridgebunyips.bunyipslib.external.units.Time;
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

/**
 * Define a callback to run when this task is started, and when it is finished. Optionally mention a timeout.
 *
 * @author Lucas Bubner, 2024
 */
public class StartEndTask extends Task {
    private final Runnable onStart;
    private final Runnable onFinish;

    /**
     * Create a new StartEndTask with no timeout.
     *
     * @param onStart the callback to run when the task starts
     * @param onEnd   the callback to run when the task finishes
     */
    public StartEndTask(Runnable onStart, Runnable onEnd) {
        this(INFINITE_TIMEOUT, onStart, onEnd);
    }

    /**
     * Create a new StartEndTask with a timeout.
     *
     * @param timeoutSeconds the timeout for the task
     * @param onStart        the callback to run when the task starts
     * @param onFinish       the callback to run when the task finishes
     */
    public StartEndTask(Measure<Time> timeoutSeconds, Runnable onStart, Runnable onFinish) {
        super(timeoutSeconds);
        this.onStart = onStart;
        this.onFinish = onFinish;
    }

    @Override
    protected void init() {
        onStart.run();
    }

    @Override
    protected void periodic() {
        // no-op
    }

    @Override
    protected void onFinish() {
        onFinish.run();
    }

    @Override
    protected boolean isTaskFinished() {
        // Relying on timeout or interruption to finish
        return false;
    }
}
