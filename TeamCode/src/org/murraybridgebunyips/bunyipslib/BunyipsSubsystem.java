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

    public final void setDefaultTask(Task defaultTask) {
        this.defaultTask = defaultTask;
    }

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
            Dbg.warn("Attempted to set a task while another task was running in %, this was ignored.", getClass().getSimpleName());
            return false;
        }

        currentTask.reset();
        // Default task technically can't finish, but it can be interrupted, so we will just run the finish callback
        if (this.currentTask == defaultTask)
            defaultTask.onFinish();
        this.currentTask = currentTask;
        return true;
    }

    public final void addDependencyFromTask(int taskHashCode) {
        dependencies.add(taskHashCode);
    }

    public final ArrayList<Integer> getTaskDependencies() {
        return dependencies;
    }

    public final void setHighPriorityCurrentTask(Task currentTask) {
        // Task will be cancelled abruptly, run the finish callback now
        if (this.currentTask != defaultTask) {
            Dbg.warn("A high-priority task has forcefully interrupted the current task in %.", getClass().getSimpleName());
            this.currentTask.forceFinish();
        }
        currentTask.reset();
        // Default task technically can't finish, but it can be interrupted, so we will just run the finish callback
        if (this.currentTask == defaultTask)
            defaultTask.onFinish();
        this.currentTask = currentTask;
    }

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

    public abstract void update();
}
