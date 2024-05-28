package org.murraybridgebunyips.bunyipslib.tasks

import org.firstinspires.ftc.robotcore.external.Telemetry.Item
import org.murraybridgebunyips.bunyipslib.Text.formatString
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
    private lateinit var item: Item

    init {
        withName("Message")
    }

    private fun buildString(): String {
        return formatString("%/%s: %", round(deltaTime.inUnit(Seconds), 1), timeout.inUnit(Seconds), message)
    }

    override fun init() {
        item = opMode.addRetainedTelemetry(buildString())
    }

    override fun periodic() {
        item.setValue(buildString())
    }

    override fun isTaskFinished(): Boolean {
        return false
    }

    override fun onFinish() {
        opMode.telemetry.removeRetained(item)
    }
}