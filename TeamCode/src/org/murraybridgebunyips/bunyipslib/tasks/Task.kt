package org.murraybridgebunyips.bunyipslib.tasks

import org.murraybridgebunyips.bunyipslib.BunyipsOpMode

/**
 * A task - an action that can be run as a scheduled routine to loop through actions
 * until a criteria or timeout is met. This is very similar to FRC WPILib "Command-based" programming.
 * @author Lucas Bubner, 2022-2023
 */
abstract class Task : AutoTask {
    protected var time: Double
    private var startTime = 0.0

    protected var opMode: BunyipsOpMode

    @Volatile
    var taskFinished = false

    private var finisherFired = false

    /**
     * @param time Maximum timeout (sec) of the task. If set to zero this will serve as an indefinite init-task.
     */
    constructor(opMode: BunyipsOpMode, time: Double) {
        this.time = time
        this.opMode = opMode
    }

    /**
     * Overloading base Task allows any tasks that may not want to use a time restriction,
     * such as an init-loop task. This will perform the same as a task also defined with a time of 0 seconds.
     * Note that tasks with no time restriction can only be used in the INIT phase.
     *
     * It is not recommended to use this constructor for tasks that will be used in an activeLoop.
     * If this is desired, you may be better off using BunyipsOpMode standalone, as tasks
     * are blocking and will not allow other tasks to run until they are finished.
     */
    constructor(opMode: BunyipsOpMode) {
        this.opMode = opMode
        time = 0.0
    }

    /**
     * Define code to run once, when the task is started.
     */
    abstract override fun init()

    /**
     * To run as an activeLoop during this task's duration.
     * Somewhere in your polling loop you must call isFinished() to determine when the task is finished.
     * (AutonomousBunyipsOpMode will handle this automatically and checking isFinished() is not required)
     * @see onFinish()
     */
    abstract override fun run()

    /**
     * Finalising function to run once the task is finished.
     */
    abstract override fun onFinish()

    /**
     * Return a boolean to this method to add custom criteria if a task should be considered finished.
     * @return bool expression indicating whether the task is finished or not, timeout and OpMode state are handled automatically.
     */
    abstract fun isTaskFinished(): Boolean

    final override fun isFinished(): Boolean {
        if (taskFinished) {
            if (!finisherFired)
                onFinish()
            finisherFired = true
            return true
        }
        if (startTime == 0.0) {
            init()
            startTime = currentTime
        }
        // Finish tasks that exceed a time limit, if the OpMode is stopped, or if the task is
        // set to run indefinitely and the OpMode is not in init-phase.
        // In order to prevent an infinite running task we prohibit indefinite tasks outside of init
        if ((time != 0.0 && currentTime > startTime + time) || (time == 0.0 && !opMode.opModeInInit())) {
            if (!finisherFired)
                onFinish()
            finisherFired = true
            taskFinished = true
        }
        return taskFinished || isTaskFinished()
    }

    /**
     * Reset a task to an uninitialised and unfinished state.
     */
    fun reset() {
        startTime = 0.0
        taskFinished = false
        finisherFired = false
    }

    /**
     * @return Whether the task is currently running
     */
    fun isRunning(): Boolean {
        return startTime != 0.0 && !taskFinished
    }

    protected val currentTime: Double
        get() = System.nanoTime() / NANOS_IN_SECONDS

    protected val deltaTime: Double
        get() = currentTime - startTime

    companion object {
        const val NANOS_IN_SECONDS = 1000000000.0
    }
}