package org.murraybridgebunyips.bunyipslib.tasks

import org.murraybridgebunyips.bunyipslib.Text.round
import org.murraybridgebunyips.bunyipslib.tasks.bases.RobotTask
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task

/**
 * Task to wait for a specific amount of time.
 */
class WaitTask(time: Double, private val showTelemetry: Boolean = true) : Task(time), RobotTask {
    constructor(time: Double) : this(time, true)

    override fun init() {
        // no-op
    }

    override fun periodic() {
        if (showTelemetry)
            opMode.addTelemetry("Waiting %/% seconds...", round(deltaTime, 1), timeout)
    }

    override fun isTaskFinished(): Boolean {
        return false
    }

    override fun onFinish() {
        // no-op
    }
}