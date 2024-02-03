package org.murraybridgebunyips.bunyipslib.tasks.bases

import org.murraybridgebunyips.bunyipslib.BunyipsOpMode
import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem

/**
 * RunForeverTask interface with BunyipsOpMode dependency injection.
 */
abstract class BunyipsRunForeverTask(open val opMode: BunyipsOpMode) : RunForeverTask() {
    constructor(
        opMode: BunyipsOpMode,
        dependencySubsystem: BunyipsSubsystem,
        shouldOverrideConflictingTasks: Boolean
    ) : this(opMode) {
        addDependency(dependencySubsystem)
        overrideOnConflict = shouldOverrideConflictingTasks
    }
}
