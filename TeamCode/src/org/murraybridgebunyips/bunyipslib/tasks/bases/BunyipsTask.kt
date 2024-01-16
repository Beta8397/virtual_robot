package org.murraybridgebunyips.bunyipslib.tasks.bases

import org.murraybridgebunyips.bunyipslib.BunyipsOpMode
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task

/**
 * Task interface with BunyipsOpMode dependency injection.
 */
abstract class BunyipsTask(open val opMode: BunyipsOpMode, time: Double) : Task(time) {
    constructor(opMode: BunyipsOpMode) : this(opMode, 0.0)
}