package org.firstinspires.ftc.teamcode.common.tasks

import org.firstinspires.ftc.teamcode.common.BunyipsOpMode

/**
 * Relay a message to the driver station for a specific time.
 */
class MessageTask(opMode: BunyipsOpMode, time: Double, private val message: String) :
    Task(opMode, time), AutoTask {
    override fun init() {
        return
    }

    override fun run() {
        opMode.addTelemetry(message)
    }

    override fun isTaskFinished(): Boolean {
        return false
    }

    override fun onFinish() {
        return
    }
}