package org.murraybridgebunyips.bunyipslib.tasks.bases;

import org.murraybridgebunyips.bunyipslib.BunyipsOpMode;
import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;

/**
 * RunNoTimeoutTask interface with BunyipsOpMode dependency injection.
 */
public abstract class BunyipsNoTimeoutTask extends RunNoTimeoutTask {
    protected final BunyipsOpMode opMode;

    protected BunyipsNoTimeoutTask(BunyipsOpMode opMode) {
        super();
        this.opMode = opMode;
    }

    protected BunyipsNoTimeoutTask(BunyipsOpMode opMode, BunyipsSubsystem dependencySubsystem, boolean shouldOverrideConflictingTasks) {
        super(dependencySubsystem, shouldOverrideConflictingTasks);
        this.opMode = opMode;
    }
}
