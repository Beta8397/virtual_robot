package org.murraybridgebunyips.bunyipslib.tasks;

import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

/**
 * Define a callback to run when this task is started, and when it is finished. Optionally mention a timeout.
 *
 * @author Lucas Bubner, 2024
 */
public class StartEndTask extends Task {
    private final Runnable onStart;
    private final Runnable onFinish;

    public StartEndTask(Runnable onStart, Runnable onEnd) {
        this(0.0, onStart, onEnd);
    }

    public StartEndTask(double timeoutSeconds, Runnable onStart, Runnable onFinish) {
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
