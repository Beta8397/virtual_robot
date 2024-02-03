package org.murraybridgebunyips.bunyipslib.tasks.bases

import org.murraybridgebunyips.bunyipslib.BunyipsOpMode
import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem

/**
 * RunOnceTask interface with BunyipsOpMode dependency injection.
 */
abstract class BunyipsRunOnceTask(open val opMode: BunyipsOpMode) : RunOnceTask() {
    constructor(
        opMode: BunyipsOpMode,
        dependencySubsystem: BunyipsSubsystem,
        shouldOverrideConflictingTasks: Boolean
    ) : this(opMode) {
        addDependency(dependencySubsystem)
        overrideOnConflict = shouldOverrideConflictingTasks
    }
}
