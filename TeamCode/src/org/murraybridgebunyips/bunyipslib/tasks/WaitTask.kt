package org.murraybridgebunyips.bunyipslib.tasks

import org.murraybridgebunyips.bunyipslib.Text.round
import org.murraybridgebunyips.bunyipslib.external.units.Measure
import org.murraybridgebunyips.bunyipslib.external.units.Time
import org.murraybridgebunyips.bunyipslib.external.units.Units.Seconds
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task

/**
 * Task to wait for a specific amount of time.
 */
class WaitTask(time: Measure<Time>, private val showTelemetry: Boolean = true) : Task(time) {
    constructor(time: Measure<Time>) : this(time, true)

    // Special utility constructors for this specific application
    constructor(magnitude: Double, unit: Time) : this(unit.of(magnitude), true)
    constructor(magnitude: Double, unit: Time, showTelemetry: Boolean) : this(unit.of(magnitude), showTelemetry)

    init {
        withName("Wait")
    }

    override fun periodic() {
        if (showTelemetry)
            opMode.telemetry.add("Waiting %/% seconds...", round(deltaTime.inUnit(Seconds), 1), timeout.inUnit(Seconds))
    }

    override fun isTaskFinished(): Boolean {
        return false
    }
}