package org.murraybridgebunyips.bunyipslib.tasks.bases

import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem

/**
 * A task, or command is an action that can be performed by a robot. This has been designed
 * to reflect closely the command-based programming style used in FRC, while still being
 * reflective of the past nature of how the Task system was implemented in BunyipsLib.
 * @author Lucas Bubner, 2022-2024
 */
abstract class Task(timeoutSeconds: Double) : RobotTask {
    private var overrideOnConflict: Boolean? = null

    fun shouldOverrideOnConflict(): Boolean? {
        return overrideOnConflict
    }

    fun addDependency(dependencySubsystem: BunyipsSubsystem) {
        dependencySubsystem.addDependencyFromTask(this.hashCode())
        if (overrideOnConflict == null)
            overrideOnConflict = false
    }

    constructor(timeoutSeconds: Double, dependencySubsystem: BunyipsSubsystem, shouldOverrideOtherTasks: Boolean) : this(timeoutSeconds) {
        addDependency(dependencySubsystem)
        overrideOnConflict = shouldOverrideOtherTasks
    }

    /**
     * Maximum timeout (sec) of the task. If set to 0 this will serve as an indefinite task, and
     * will only finish when isTaskFinished() returns true.
     */
    var timeout: Double = timeoutSeconds

    @Volatile
    var taskFinished = false
        private set

    private var startTime = 0.0
    private var finisherFired = false

    /**
     * Define code to run once, when the task is started.
     */
    abstract override fun init()

    /**
     * To run as an activeLoop during this task's duration.
     * Somewhere in your polling loop you must call isFinished() to determine when the task is finished, and to fire init() and onFinish() accordingly.
     * Both Scheduler and AutonomousBunyipsOpMode will handle this for you.
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

    /**
     * Must be called in a polling loop before run() to ensure init() is called first.
     */
    final override fun isFinished(): Boolean {
        if (startTime == 0.0) {
            init()
            startTime = currentTime
        }
        if (taskFinished || isTaskFinished()) {
            if (!finisherFired)
                onFinish()
            finisherFired = true
            taskFinished = true
            return true
        }
        // Finish tasks that exceed a time limit defined by the task
        if (timeout != 0.0 && currentTime > startTime + timeout) {
            if (!finisherFired)
                onFinish()
            finisherFired = true
            taskFinished = true
        }
        return taskFinished && finisherFired
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
     * Force a task to finish immediately.
     */
    fun finishNow() {
        taskFinished = true
    }

    /**
     * Force a task to finish immediately, and fire the onFinish() method without waiting
     * for the next polling loop. This method is useful when your task is about to die and
     * needs to finish up immediately.
     */
    fun forceFinish() {
        taskFinished = true
        finisherFired = true
        onFinish()
    }

    /**
     * @return Whether the task is currently running (calls to run() should be made)
     */
    val isRunning: Boolean = startTime != 0.0 && !taskFinished

    val currentTime: Double
        get() = System.nanoTime() / NANOS_IN_SECONDS

    val deltaTime: Double
        get() = currentTime - startTime

    companion object {
        const val NANOS_IN_SECONDS = 1000000000.0
    }
}