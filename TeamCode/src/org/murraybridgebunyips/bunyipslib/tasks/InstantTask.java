package org.murraybridgebunyips.bunyipslib.tasks;

import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.tasks.bases.RunOnceTask;

/**
 * A task to run a callback before immediately completing.
 * <p>
 * {@code new InstantTask(() -> addTelemetry("Hello world"));}
 */
public class InstantTask extends RunOnceTask {
    private final Runnable callback;

    public InstantTask(Runnable callback, BunyipsSubsystem... dependencies) {
        this.callback = callback;
        for (BunyipsSubsystem dependency : dependencies) {
            addDependency(dependency);
        }
    }

    @Override
    public void runOnce() {
        callback.run();
    }
}
