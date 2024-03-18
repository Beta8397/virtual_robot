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

    /**
     * Run the given callback immediately.
     *
     * @param callback The callback to run
     */
    public InstantTask(Runnable callback) {
        this.callback = callback;
    }

    /**
     * Run the given callback immediately, with a dependency on the given subsystem.
     *
     * @param callback                       The callback to run.
     * @param dependencySubsystem            The subsystem to run this on.
     * @param shouldOverrideConflictingTasks Whether to override conflicting tasks.
     */
    public InstantTask(Runnable callback, BunyipsSubsystem dependencySubsystem, boolean shouldOverrideConflictingTasks) {
        super(dependencySubsystem, shouldOverrideConflictingTasks);
        this.callback = callback;
    }

    @Override
    public void runOnce() {
        callback.run();
    }
}
