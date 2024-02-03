package org.murraybridgebunyips.bunyipslib.tasks;

import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.tasks.bases.RunForeverTask;

/**
 * A task to run continuously and will never finish.
 * <p>
 * {@code new ContinuousTask(() -> addTelemetry("I will run forever"));}
 */
public class ContinuousTask extends RunForeverTask {
    private final Runnable callback;

    public ContinuousTask(Runnable callback) {
        this.callback = callback;
    }

    public ContinuousTask(Runnable callback, BunyipsSubsystem dependency, boolean shouldOverrideConflictingTasks) {
        super(dependency, shouldOverrideConflictingTasks);
        this.callback = callback;
    }

    @Override
    public void periodic() {
        callback.run();
    }

    @Override
    public void init() {
        // no-op
    }

    @Override
    public void onFinish() {
        // no-op
    }
}
