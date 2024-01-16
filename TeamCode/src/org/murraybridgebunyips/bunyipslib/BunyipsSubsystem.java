package org.murraybridgebunyips.bunyipslib;

import androidx.annotation.NonNull;

import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

import java.util.ArrayList;

public abstract class BunyipsSubsystem extends BunyipsComponent {
    private final ArrayList<Integer> dependencies = new ArrayList<>();
    private Task currentTask;
    private Task defaultTask;

    protected BunyipsSubsystem(@NonNull BunyipsOpMode opMode) {
        super(opMode);
    }

    public Task getCurrentTask() {
        if (currentTask == null || currentTask.isFinished()) {
            currentTask = defaultTask;
        }
        return currentTask;
    }

    public final void setDefaultTask(Task defaultTask) {
        this.defaultTask = defaultTask;
    }

    public final boolean setCurrentTask(Task currentTask) {
        if (this.currentTask == currentTask)
            return true;

        // Lockout if a task is currently running that is not the default task
        if (currentTask != defaultTask) {
            // Override if the task is designed to override
            // shouldOverrideOnConflict might be null if it is a non-command task
            if (Boolean.TRUE.equals(currentTask.shouldOverrideOnConflict())) {
                setHighPriorityCurrentTask(currentTask);
                return true;
            }
            Dbg.warn("Attempted to set a task while another task was running in %, this was ignored.", this.getClass().getCanonicalName());
            return false;
        }

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
        if (currentTask != defaultTask) {
            Dbg.warn("A high-priority task has forcefully interrupted the current task in %.", this.getClass().getCanonicalName());
            currentTask.forceFinish();
        }
        this.currentTask = currentTask;
    }

    public final void run() {
        Task task = getCurrentTask();
        if (task != null) {
            if (task == defaultTask && defaultTask.isFinished()) {
                throw new EmergencyStop("Default task should never finish!");
            }
            task.run();
        }
        update();
    }

    public abstract void update();
}
