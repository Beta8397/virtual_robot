package org.murraybridgebunyips.bunyipslib.tasks.bases

import org.murraybridgebunyips.bunyipslib.BunyipsOpMode
import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem
import org.murraybridgebunyips.bunyipslib.MovingAverageTimer.NANOS_IN_SECONDS

/**
 * A task, or command is an action that can be performed by a robot. This has been designed
 * to reflect closely the command-based programming style used in FRC, while still being
 * reflective of the past nature of how the Task system was implemented in BunyipsLib.
 * @author Lucas Bubner, 2024
 */
abstract class Task(timeoutSeconds: Double) : RobotTask {
    // Since the OpMode is static, this means ** ALL tasks MUST be instantiated in the init phase **, and not
    // in the constructor/member fields. You will experience extremely strange behaviour if you do not follow this.
    // This should not be a problem as all tasks are usually instantiated in the init phase anyway as tasks usually need
    // subsystems which need motors that are only available at runtime.
    /**
     * The OpMode instance that this task is running in.
     */
    @JvmField
    protected val opMode: BunyipsOpMode = BunyipsOpMode.instance

    private var overrideOnConflict: Boolean? = null
    private var name = this.javaClass.simpleName
    private var mutedReport = false

    /**
     * Mute task reports from the Scheduler.
     */
    fun withMutedReports(): Task {
        mutedReport = true
        return this
    }

    /**
     * @return Whether this task is muted or not.
     */
    fun isMuted(): Boolean {
        return mutedReport
    }

    /**
     * @return Whether this task should override other tasks in the queue if they conflict with this task. Will return
     *         null if this task has no dependencies.
     */
    fun shouldOverrideOnConflict(): Boolean? {
        return overrideOnConflict
    }

    constructor(
        timeoutSeconds: Double,
        dependencySubsystem: BunyipsSubsystem,
        shouldOverrideConflictingTasks: Boolean
    ) : this(timeoutSeconds) {
        dependencySubsystem.addDependencyFromTask(this.hashCode())
        overrideOnConflict = shouldOverrideConflictingTasks
    }

    /**
     * Set the name of this task to be displayed in the OpMode.
     */
    fun withName(name: String?): Task {
        if (name == null) {
            return this
        }
        this.name = name
        return this
    }

    /**
     * Get the name of this task. By default, it will be the class simple name, but you can override this method to
     * provide a custom name.
     * @return String representing the name of this task.
     */
    override fun toString(): String {
        return name
    }

    /**
     * Maximum timeout (sec) of the task. If set to 0 this will serve as an indefinite task, and
     * will only finish when isTaskFinished() returns true.
     */
    var timeout: Double = timeoutSeconds

    /**
     * Set the timeout of this task dynamically and return the task.
     */
    fun withTimeout(timeout: Double): Task {
        this.timeout = timeout
        return this
    }

    /**
     * Whether the task is finished or not via timeout or custom condition. Will be true regardless of the finisher
     * being fired or not, as some tasks will handle this via finishNow().
     */
    @Volatile
    var taskFinished = false
        private set

    private var startTime = 0L
    private var finisherFired = false

    /**
     * Define code to run once, when the task is started.
     */
    protected abstract fun init()

    /**
     * To run as an active loop during this task's duration.
     */
    protected abstract fun periodic()

    /**
     * Should be called by your polling loop to run the task and manage all state properly.
     */
    final override fun run() {
        if (startTime == 0L) {
            init()
            startTime = System.nanoTime()
            // Must poll finished on the first iteration to ensure that the task does not overrun
            pollFinished()
        }
        // Here we check the taskFinished condition but don't poll updateFinishedState(), to ensure that the task is only
        // updated with latest finish information at the user's discretion
        if (taskFinished && !finisherFired) {
            onFinish()
            finisherFired = true
        }
        periodic()
    }

    /**
     * Finalising function to run once the task is finished.
     */
    protected abstract fun onFinish()

    /**
     * Return a boolean to this method to add custom criteria if a task should be considered finished.
     * @return bool expression indicating whether the task is finished or not, timeout and OpMode state are handled automatically.
     */
    protected abstract fun isTaskFinished(): Boolean

    /**
     * Called when the task is reset. Override this method to add custom reset behaviour, such as resetting any
     * internal state variables such as iterators or lists.
     */
    protected open fun onReset() {
    }

    /**
     * Query (but not update) the finished state of the task. This will return true if the task is finished and the
     * finisher has been fired.
     */
    final override fun isFinished(): Boolean {
        return taskFinished && finisherFired
    }

    /**
     * Update and query the state of the task if it is finished. This will return true if the task is finished and the
     * finisher has been fired.
     */
    final override fun pollFinished(): Boolean {
        // Early return
        if (taskFinished) return finisherFired

        // Finish on user defined task finished condition, or by timeout
        taskFinished =
            (timeout != 0.0 && startTime != 0L && System.nanoTime() > startTime + (timeout * NANOS_IN_SECONDS))
                    || isTaskFinished()

        // run() will handle firing the finisher, in which case we can return true and the polling loop can stop
        return taskFinished && finisherFired
    }

    /**
     * Reset a task to an uninitialised and unfinished state.
     */
    fun reset() {
        startTime = 0L
        taskFinished = false
        finisherFired = false
        onReset()
    }

    /**
     * Tell a task to finish on the next iteration.
     */
    fun finish() {
        taskFinished = true
    }

    /**
     * Force a task to finish immediately, and fire the onFinish() method without waiting
     * for the next polling loop. This method is useful when your task needs to die and
     * needs to finish up immediately.
     */
    fun finishNow() {
        taskFinished = true
        finisherFired = true
        onFinish()
    }

    /**
     * @return Whether the task is currently running (i.e. has been started (`init()` called) and not finished).
     */
    val isRunning: Boolean
        get() = startTime != 0L && !isFinished()

    /**
     * Time in seconds since the task was started.
     */
    val deltaTime: Double
        get() {
            if (startTime == 0L)
                return 0.0
            return (System.nanoTime() - startTime) / NANOS_IN_SECONDS
        }
}