package org.murraybridgebunyips.bunyipslib.tasks;

import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

/**
 * A task to run for a timeout.
 * <p>
 * {@code new RunForTask(5, () -> arm.spin());}
 */
public class RunForTask extends Task {
    private final Runnable callback;

    public RunForTask(double timeout, Runnable callback) {
        super(timeout);
        this.callback = callback;
    }

    @Override
    public void init() {
        // noop
    }

    @Override
    public void run() {
        callback.run();
    }

    @Override
    public void onFinish() {
        // noop
    }

    @Override
    public boolean isTaskFinished() {
        // Timeout will handle this automatically
        return false;
    }
}
