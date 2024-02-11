package org.murraybridgebunyips.bunyipslib.tasks.bases;

import androidx.annotation.NonNull;

import org.murraybridgebunyips.bunyipslib.BunyipsOpMode;
import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;

/**
 * RunOnceTask interface with BunyipsOpMode dependency injection.
 */
public abstract class BunyipsRunOnceTask extends RunOnceTask {
    protected final BunyipsOpMode opMode;

    protected BunyipsRunOnceTask(BunyipsOpMode opMode) {
        this.opMode = opMode;
    }

    protected BunyipsRunOnceTask(BunyipsOpMode opMode, @NonNull BunyipsSubsystem dependencySubsystem, boolean shouldOverrideConflictingTasks) {
        super(dependencySubsystem, shouldOverrideConflictingTasks);
        this.opMode = opMode;
    }
}
