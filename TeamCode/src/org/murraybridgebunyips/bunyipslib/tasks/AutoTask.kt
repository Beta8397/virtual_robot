package org.murraybridgebunyips.bunyipslib.tasks

/**
 * Structure of an Autonomous Task to be used during the autonomous period or as part of an autonomous routine.
 */
interface AutoTask {
    fun init()
    fun run()
    fun isFinished(): Boolean
    fun onFinish()
}