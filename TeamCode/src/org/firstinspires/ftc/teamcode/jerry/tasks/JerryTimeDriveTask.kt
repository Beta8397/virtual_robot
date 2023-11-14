package org.firstinspires.ftc.teamcode.jerry.tasks

import org.firstinspires.ftc.teamcode.common.BunyipsOpMode
import org.firstinspires.ftc.teamcode.common.tasks.AutoTask
import org.firstinspires.ftc.teamcode.common.tasks.Task
import org.firstinspires.ftc.teamcode.jerry.components.JerryDrive

/**
 * Base drive task which will run XYR speed for a given time
 * Only used for tests and as a failsafe, do not use in actual OpMode as field positioning data is lost
 * Use JerryPrecisionDriveTask instead!
 * @see JerryPrecisionDriveTask
 */
class JerryTimeDriveTask(
    opMode: BunyipsOpMode,
    time: Double,
    private val drive: JerryDrive?,
    private val x: Double,
    private val y: Double,
    private val r: Double
) : Task(opMode, time), AutoTask {
    override fun init() {
        return
    }

    override fun run() {
        drive?.setSpeedUsingController(x, -y, r)
        drive?.update()
    }

    override fun isTaskFinished(): Boolean {
        return false
    }

    override fun onFinish() {
        drive?.stop()
    }
}