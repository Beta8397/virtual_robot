package org.murraybridgebunyips.bunyipslib.tasks.bases

/**
 * Structure of a Task to execute through the command system.
 */
interface RobotTask {
    /**
     * Runs every loop of the command system.
     */
    fun run()

    /**
     * Returns true if the task is finished, without checking the condition for the task's completion.
     */
    fun isFinished(): Boolean

    /**
     * Returns true if the task is finished, polling actively for the condition of the task's completion.
     */
    fun pollFinished(): Boolean
}