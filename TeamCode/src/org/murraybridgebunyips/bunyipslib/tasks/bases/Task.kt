package org.murraybridgebunyips.bunyipslib.tasks.bases

import org.murraybridgebunyips.bunyipslib.BunyipsComponent
import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem
import org.murraybridgebunyips.bunyipslib.external.units.Measure
import org.murraybridgebunyips.bunyipslib.external.units.Time
import org.murraybridgebunyips.bunyipslib.external.units.Units.Nanoseconds
import org.murraybridgebunyips.bunyipslib.external.units.Units.Seconds
import java.util.Optional

/**
 * A task, or command is an action that can be performed by a robot. This has been designed
 * to reflect closely the command-based programming style used in FRC, while still being
 * reflective of the past nature of how the Task system was implemented in BunyipsLib.
 * @author Lucas Bubner, 2024
 */
abstract class Task(
    /**
     * Maximum timeout of the task. If set to 0 magnitude this will serve as an indefinite task, and
     * will only finish when isTaskFinished() returns true.
     */
    var timeout: Measure<Time>
) : BunyipsComponent(), RobotTask {
    private var overrideDependency: Boolean = false
    private var dependency: BunyipsSubsystem? = null
    private var mutedReport = false

    private var name = this.javaClass.simpleName

    /**
     * Return whether this task has a dependency on a subsystem or not.
     */
    fun hasDependency(): Boolean {
        return dependency != null
    }

    /**
     * Get the subsystem reference that this task is dependent on.
     * Will return an Optional where if it is not present, this task is not dependent on any subsystem.
     */
    fun getDependency(): Optional<BunyipsSubsystem> {
        return Optional.ofNullable(dependency)
    }

    /**
     * Mute task reports from the Scheduler.
     */
    fun withMutedReports(): Task {
        mutedReport = true
        return this
    }

    /**
     * @return Whether this task is muted from subsystem reports or not.
     */
    fun isMuted(): Boolean {
        return mutedReport
    }

    /**
     * @return Whether this task should override other tasks in the queue if they conflict with this task. Will only
     *         apply if this task has a dependency (see hasDependency, getDependency).
     */
    fun isOverriding(): Boolean {
        return overrideDependency
    }

    constructor(
        timeout: Measure<Time>,
        dependencySubsystem: BunyipsSubsystem,
        override: Boolean
    ) : this(timeout) {
        dependency = dependencySubsystem
        overrideDependency = override
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
     * Get a verbose string representation of this task, including all of its properties.
     */
    fun toVerboseString(): String {
        return name + "[${if (taskFinished) "FINISHED" else "READY"}, ${if (isRunning) deltaTime else "NOT RUNNING"}/${if (timeout.magnitude() == 0.0) "INDEFINITE" else "$timeout"}, ${if (dependency != null) "DEPENDENT ON <${dependency?.javaClass?.simpleName}>" else "INDEPENDENT"}, ${if (overrideDependency) "OVERRIDING" else "NON-OVERRIDING"}, ${if (mutedReport) "MUTED" else "REPORTING"}]"
    }

    /**
     * Set the timeout of this task dynamically and return the task.
     */
    fun withTimeout(timeout: Measure<Time>): Task {
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
        // Here we check the taskFinished condition but don't call pollFinished(), to ensure that the task is only
        // updated with latest finish information at the user's discretion (excluding the first-call requirement)
        if (taskFinished && !finisherFired) {
            onFinish()
            finisherFired = true
        }
        // Don't run the task if it is finished as a safety guard
        if (isFinished()) return
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

        val startCalled = startTime != 0L
        val timeoutFinished = timeout.magnitude() != 0.0 && System.nanoTime() > startTime + timeout.inUnit(Nanoseconds)
        val userCondition = isTaskFinished()

        taskFinished = startCalled && (timeoutFinished || userCondition)

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
     * needs to finish up immediately. If your finisher has already been fired, this method
     * will do nothing but ensure that the task is marked as finished.
     */
    fun finishNow() {
        taskFinished = true
        if (!finisherFired)
            onFinish()
        finisherFired = true
    }

    /**
     * @return Whether the task is currently running (i.e. has been started (`init()` called) and not finished).
     */
    val isRunning: Boolean
        get() = startTime != 0L && !isFinished()

    /**
     * Time in seconds since the task was started.
     */
    val deltaTime: Measure<Time>
        get() {
            if (startTime == 0L)
                return Nanoseconds.of(0.0)
            return Nanoseconds.of((System.nanoTime() - startTime).toDouble())
        }

    companion object {
        /**
         * Timeout value for an infinite task that will run forever.
         */
        @JvmField
        val INFINITE_TIMEOUT: Measure<Time> = Seconds.zero()
    }
}