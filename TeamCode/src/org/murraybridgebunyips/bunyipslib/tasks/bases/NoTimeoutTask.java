package org.murraybridgebunyips.bunyipslib.tasks.bases;

import androidx.annotation.NonNull;

import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;

/**
 * A task that runs with no timeout, finish condition must be controlled by the user.
 */
public abstract class NoTimeoutTask extends Task {
    protected NoTimeoutTask() {
        super(0.0);
    }

    protected NoTimeoutTask(@NonNull BunyipsSubsystem dependencySubsystem, boolean shouldOverrideConflictingTasks) {
        super(0.0, dependencySubsystem, shouldOverrideConflictingTasks);
    }
}
