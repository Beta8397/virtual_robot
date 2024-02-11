package org.murraybridgebunyips.bunyipslib.tasks.bases;

import androidx.annotation.NonNull;

import org.murraybridgebunyips.bunyipslib.BunyipsOpMode;
import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;

/**
 * RunForeverTask interface with BunyipsOpMode dependency injection.
 */
public abstract class BunyipsRunForeverTask extends RunForeverTask {
    protected final BunyipsOpMode opMode;

    protected BunyipsRunForeverTask(BunyipsOpMode opMode) {
        this.opMode = opMode;
    }

    protected BunyipsRunForeverTask(BunyipsOpMode opMode, @NonNull BunyipsSubsystem dependencySubsystem, boolean shouldOverrideConflictingTasks) {
        super(dependencySubsystem, shouldOverrideConflictingTasks);
        this.opMode = opMode;
    }
}
