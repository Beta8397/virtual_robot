package org.murraybridgebunyips.bunyipslib;

import static org.murraybridgebunyips.bunyipslib.Text.round;

import org.murraybridgebunyips.bunyipslib.tasks.IdleTask;
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

import java.util.ArrayList;

/**
 * Base class for all robot subsystems.
 * Integrates with the Task system to allow for task-based command scheduling.
 *
 * @author Lucas Bubner, 2024
 * @see Scheduler
 */
public abstract class BunyipsSubsystem extends BunyipsComponent {
    private final ArrayList<Integer> dependencies = new ArrayList<>();
    private Task currentTask;
    private Task defaultTask = new IdleTask();
    private boolean mutedReports;

    /**
     * Get the current task for this subsystem.
     * If the current task is null or finished, the default task will be returned.
     *
     * @return The current task
     */
    public Task getCurrentTask() {
        if (currentTask == null || currentTask.isFinished()) {
            currentTask = defaultTask;
        }
        return currentTask;
    }

    /**
     * Call to mute the Scheduler from reporting task status for this subsystem.
     */
    public void muteTaskReports() {
        mutedReports = true;
    }

    /**
     * Set the default task for this subsystem, which will be run when no other task is running.
     *
     * @param defaultTask The task to set as the default task
     */
    public final void setDefaultTask(Task defaultTask) {
        this.defaultTask = defaultTask;
    }

    /**
     * Set the current task to the given task.
     *
     * @param currentTask The task to set as the current task
     * @return whether the task was successfully set or ignored
     */
    public final boolean setCurrentTask(Task currentTask) {
        if (this.currentTask == currentTask)
            return true;

        // Lockout if a task is currently running that is not the default task
        if (this.currentTask != defaultTask) {
            // Override if the task is designed to override
            // shouldOverrideOnConflict might be null if it is a non-command task
            if (Boolean.TRUE.equals(currentTask.shouldOverrideOnConflict())) {
                setHighPriorityCurrentTask(currentTask);
                return true;
            }
            Dbg.log(getClass(), "Ignored task change: %->%", this.currentTask.getName(), currentTask.getName());
            return false;
        }

        currentTask.reset();
        // Default task technically can't finish, but it can be interrupted, so we will just run the finish callback
        if (this.currentTask == defaultTask)
            defaultTask.onFinish();
        Dbg.logd(getClass(), "Task changed: %->%", this.currentTask.getName(), currentTask.getName());
        this.currentTask = currentTask;
        return true;
    }

    /**
     * Add a dependency from another task to this subsystem.
     *
     * @param taskHashCode The hash code of the task to add as a dependency
     */
    public final void addDependencyFromTask(int taskHashCode) {
        dependencies.add(taskHashCode);
    }

    /**
     * Get the dependencies for this subsystem.
     *
     * @return The dependencies for this subsystem
     */
    public final ArrayList<Integer> getTaskDependencies() {
        return dependencies;
    }

    /**
     * Set the current task to the given task, overriding any current task.
     *
     * @param currentTask The task to set as the current task
     */
    public final void setHighPriorityCurrentTask(Task currentTask) {
        // Task will be cancelled abruptly, run the finish callback now
        if (this.currentTask != defaultTask) {
            Dbg.warn(getClass(), "Task changed: %(INT)->%", this.currentTask.getName(), currentTask.getName());
            this.currentTask.forceFinish();
        }
        currentTask.reset();
        // Default task technically can't finish, but it can be interrupted, so we will just run the finish callback
        if (this.currentTask == defaultTask)
            defaultTask.onFinish();
        this.currentTask = currentTask;
    }

    /**
     * Update the subsystem and run the current task, if tasks are not set up this will be identical to update().
     */
    public final void run() {
        Task task = getCurrentTask();
        if (task != null) {
            if (task == defaultTask && defaultTask.pollFinished()) {
                throw new EmergencyStop("Default task should never finish!");
            }
            task.run();
            // Update the state of isFinished() after running the task as it may have changed
            task.pollFinished();
            if (!mutedReports) {
                Scheduler.addSubsystemTaskReport(
                        getClass().getSimpleName(),
                        task.getName(),
                        round(task.getDeltaTime(), 1)
                );
            }
        }
        update();
    }

    /**
     * To be updated periodically on every hardware loop.
     */
    public abstract void update();
}
