package org.murraybridgebunyips.bunyipslib.tasks.bases;

import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;

/**
 * A task that runs forever.
 */
public abstract class ForeverTask extends NoTimeoutTask {
    protected ForeverTask() {
    }

    protected ForeverTask(BunyipsSubsystem dependencySubsystem, boolean override) {
        super(dependencySubsystem, override);
    }

    @Override
    protected final boolean isTaskFinished() {
        // Will never finish
        return false;
    }
}
