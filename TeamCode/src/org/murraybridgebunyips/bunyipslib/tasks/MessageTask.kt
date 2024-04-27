package org.murraybridgebunyips.bunyipslib.tasks

import org.murraybridgebunyips.bunyipslib.Text.round
import org.murraybridgebunyips.bunyipslib.external.units.Measure
import org.murraybridgebunyips.bunyipslib.external.units.Time
import org.murraybridgebunyips.bunyipslib.external.units.Units.Seconds
import org.murraybridgebunyips.bunyipslib.tasks.bases.RobotTask
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task

/**
 * Relay a message to the driver station for a specific time.
 */
class MessageTask(time: Measure<Time>, private val message: String) :
    Task(time), RobotTask {

    override fun init() {
        // no-op
    }

    override fun periodic() {
        opMode.addTelemetry("%/%s: %", round(deltaTime.inUnit(Seconds), 1), timeout.inUnit(Seconds), message)
    }

    override fun isTaskFinished(): Boolean {
        return false
    }

    override fun onFinish() {
        // no-op
    }
}