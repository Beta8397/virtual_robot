package org.murraybridgebunyips.bunyipslib.tasks.bases;

/**
 * A task that runs forever to no finish condition, only able to be interrupted by being finished manually.
 * This is the general class to implement default tasks in.
 */
public abstract class ForeverTask extends Task {
    @Override
    protected final boolean isTaskFinished() {
        // Will never finish automatically
        return false;
    }
}
