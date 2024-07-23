package org.murraybridgebunyips.bunyipslib.tasks

import org.firstinspires.ftc.robotcore.external.Telemetry.Item
import org.murraybridgebunyips.bunyipslib.Text.formatString
import org.murraybridgebunyips.bunyipslib.Text.round
import org.murraybridgebunyips.bunyipslib.external.units.Measure
import org.murraybridgebunyips.bunyipslib.external.units.Time
import org.murraybridgebunyips.bunyipslib.external.units.Units.Seconds
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task

/**
 * Relay a message in telemetry for a specific amount of time.
 */
class MessageTask(time: Measure<Time>, private val message: String) : Task(time) {
    private var item: Item? = null

    init {
        withName("Message")
    }

    private fun buildString(): String {
        return formatString("%/%s: %", round(deltaTime.inUnit(Seconds), 1), timeout.inUnit(Seconds), message)
    }

    override fun init() {
        item = opMode.telemetry.addRetained(buildString()).item
    }

    override fun periodic() {
        item?.setValue(buildString())
    }

    override fun isTaskFinished(): Boolean {
        return false
    }

    override fun onFinish() {
        if (item != null)
            opMode.telemetry.remove(item!!)
    }
}