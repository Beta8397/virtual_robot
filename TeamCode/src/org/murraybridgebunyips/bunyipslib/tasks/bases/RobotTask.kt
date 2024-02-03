package org.murraybridgebunyips.bunyipslib.tasks.bases

/**
 * Structure of a Task to execute through the command system.
 */
interface RobotTask {
    /**
     * Get the name of this task. By default, it will be the class simple name, but you can override this method to
     * provide a custom name.
     */
    fun getName(): String {
        return this.javaClass.simpleName
    }

    fun init()
    fun run()
    fun isFinished(): Boolean
    fun pollFinished(): Boolean
    fun onFinish()
}