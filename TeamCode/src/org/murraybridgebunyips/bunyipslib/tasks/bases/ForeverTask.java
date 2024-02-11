package org.murraybridgebunyips.bunyipslib.tasks.bases;

import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;

/**
 * A task that runs forever.
 */
public abstract class ForeverTask extends NoTimeoutTask {
    protected ForeverTask() {
        super();
    }

    protected ForeverTask(BunyipsSubsystem dependencySubsystem, boolean shouldOverrideConflictingTasks) {
        super(dependencySubsystem, shouldOverrideConflictingTasks);
    }

    @Override
    public final boolean isTaskFinished() {
        // Will never finish
        return false;
    }
}
