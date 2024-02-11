package org.murraybridgebunyips.bunyipslib.tasks.bases;

import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;

/**
 * A task that runs forever.
 */
public abstract class RunForeverTask extends RunNoTimeoutTask {
    protected RunForeverTask() {
        super();
    }

    protected RunForeverTask(BunyipsSubsystem dependencySubsystem, boolean shouldOverrideConflictingTasks) {
        super(dependencySubsystem, shouldOverrideConflictingTasks);
    }

    @Override
    public final boolean isTaskFinished() {
        // Will never finish
        return false;
    }
}
