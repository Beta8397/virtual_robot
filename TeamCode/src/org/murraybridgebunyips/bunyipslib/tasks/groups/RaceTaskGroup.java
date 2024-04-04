package org.murraybridgebunyips.bunyipslib.tasks.groups;

import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

/**
 * A group of tasks that runs all at once, until one of them is finished.
 * Once a single task is finished, all other tasks are cancelled.
 *
 * @author Lucas Bubner, 2024
 */
public class RaceTaskGroup extends TaskGroup {
    /**
     * Create a new RaceTaskGroup with tasks.
     *
     * @param tasks The tasks to run together
     */
    public RaceTaskGroup(Task... tasks) {
        super(tasks);
    }

    @Override
    public final void periodic() {
        for (Task task : tasks) {
            executeTask(task);
        }
    }

    @Override
    public final boolean isTaskFinished() {
        for (Task task : tasks) {
            if (task.pollFinished()) {
                finishAllTasks();
                // Get us out of here
                finishNow();
                return true;
            }
        }
        return false;
    }
}
