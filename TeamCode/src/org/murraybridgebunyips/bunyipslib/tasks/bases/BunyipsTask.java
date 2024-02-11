package org.murraybridgebunyips.bunyipslib.tasks.bases;

import androidx.annotation.NonNull;

import org.murraybridgebunyips.bunyipslib.BunyipsOpMode;
import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;

/**
 * Task interface with BunyipsOpMode dependency injection.
 */
public abstract class BunyipsTask extends Task {
    protected final BunyipsOpMode opMode;

    protected BunyipsTask(BunyipsOpMode opMode, double timeoutSeconds) {
        super(timeoutSeconds);
        this.opMode = opMode;
    }

    protected BunyipsTask(BunyipsOpMode opMode, double timeoutSeconds, @NonNull BunyipsSubsystem dependencySubsystem, boolean shouldOverrideConflictingTasks) {
        super(timeoutSeconds, dependencySubsystem, shouldOverrideConflictingTasks);
        this.opMode = opMode;
    }
}
