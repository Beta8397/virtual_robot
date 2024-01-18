package org.murraybridgebunyips.bunyipslib.tasks.bases

/**
 * Structure of a Task to execute through the command system.
 */
interface RobotTask {
    fun init()
    fun run()
    fun isFinished(): Boolean
    fun pollFinished(): Boolean
    fun onFinish()
}