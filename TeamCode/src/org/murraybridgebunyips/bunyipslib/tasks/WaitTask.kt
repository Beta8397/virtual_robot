package org.murraybridgebunyips.bunyipslib.tasks

import org.murraybridgebunyips.bunyipslib.Text.round
import org.murraybridgebunyips.bunyipslib.external.units.Measure
import org.murraybridgebunyips.bunyipslib.external.units.Time
import org.murraybridgebunyips.bunyipslib.external.units.Units.Seconds
import org.murraybridgebunyips.bunyipslib.tasks.bases.RobotTask
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task

/**
 * Task to wait for a specific amount of time.
 */
class WaitTask(time: Measure<Time>, private val showTelemetry: Boolean = true) : Task(time), RobotTask {
    constructor(time: Measure<Time>) : this(time, true)

    override fun init() {
        // no-op
    }

    override fun periodic() {
        if (showTelemetry)
            opMode.addTelemetry("Waiting %/% seconds...", round(deltaTime.inUnit(Seconds), 1), timeout.inUnit(Seconds))
    }

    override fun isTaskFinished(): Boolean {
        return false
    }

    override fun onFinish() {
        // no-op
    }
}