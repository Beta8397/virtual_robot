package org.murraybridgebunyips.bunyipslib.tasks

import org.murraybridgebunyips.bunyipslib.tasks.bases.RobotTask
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task

/**
 * Relay a message to the driver station for a specific time.
 */
class MessageTask(time: Double, private val message: String) :
    Task(time), RobotTask {

    protected override fun init() {
        return
    }

    override fun periodic() {
        opMode.addTelemetry(message)
    }

    override fun isTaskFinished(): Boolean {
        return false
    }

    override fun onFinish() {
        return
    }
}