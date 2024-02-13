package org.murraybridgebunyips.bunyipslib.tasks

import org.murraybridgebunyips.bunyipslib.tasks.bases.RobotTask
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task

/**
 * Task to wait for a specific amount of time.
 */
class WaitTask(time: Double) : Task(time), RobotTask {
    override fun init() {
        return
    }

    override fun periodic() {
        opMode.addTelemetry("Waiting % seconds...", timeout)
        opMode.idle()
    }

    override fun isTaskFinished(): Boolean {
        return false
    }

    override fun onFinish() {
        return
    }
}