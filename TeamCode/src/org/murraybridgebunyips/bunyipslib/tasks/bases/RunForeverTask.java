package org.murraybridgebunyips.bunyipslib.tasks.bases;

import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;

/**
 * A task that runs forever.
 */
public abstract class RunForeverTask extends Task {
    public RunForeverTask() {
        super(0.0);
    }

    public RunForeverTask(BunyipsSubsystem dependencySubsystem, boolean shouldOverrideConflictingTasks) {
       super(0.0, dependencySubsystem, shouldOverrideConflictingTasks);
    }

    @Override
    public final boolean isTaskFinished() {
        // Will never finish
        return false;
    }
}
