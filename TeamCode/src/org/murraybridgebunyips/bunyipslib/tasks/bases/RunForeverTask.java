package org.murraybridgebunyips.bunyipslib.tasks.bases;

/**
 * A task that runs forever.
 */
public abstract class RunForeverTask extends Task {
    public RunForeverTask() {
        super(0);
    }

    @Override
    public final boolean isTaskFinished() {
        // Will never finish
        return false;
    }
}
