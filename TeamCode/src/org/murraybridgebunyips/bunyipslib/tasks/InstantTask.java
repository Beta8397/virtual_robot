package org.murraybridgebunyips.bunyipslib.tasks;

import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.tasks.bases.OnceTask;

/**
 * A task to run a callback before immediately completing.
 * <p>
 * {@code new InstantTask(() -> addTelemetry("Hello world"));}
 */
public class InstantTask extends OnceTask {
    private final Runnable callback;

    public InstantTask(Runnable callback) {
        this.callback = callback;
    }

    public InstantTask(Runnable callback, BunyipsSubsystem dependencySubsystem, boolean shouldOverrideConflictingTasks) {
        super(dependencySubsystem, shouldOverrideConflictingTasks);
        this.callback = callback;
    }

    @Override
    public void runOnce() {
        callback.run();
    }
}
