package org.firstinspires.ftc.teamcode.common.tasks

/**
 * Structure of an Autonomous Task to be used during the autonomous period.
 */
interface AutoTask {
    fun init()
    fun run()
    fun isFinished(): Boolean
    fun onFinish()
}