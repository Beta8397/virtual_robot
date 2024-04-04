package org.murraybridgebunyips.bunyipslib.tasks.groups;

import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

/**
 * A group of tasks that runs all at once, until the first queued task is finished.
 * Once this task is finished, all other tasks are cancelled if not completed.
 * The task that will be the decider for the deadline is the first task passed as an argument.
 *
 * @author Lucas Bubner, 2024
 */
public class DeadlineTaskGroup extends TaskGroup {
    /**
     * Create a new DeadlineTaskGroup with tasks.
     *
     * @param tasks The tasks to run together
     */
    public DeadlineTaskGroup(Task... tasks) {
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
        Task firstTask = tasks.get(0);
        boolean firstTaskFinished = firstTask != null && firstTask.pollFinished();
        if (firstTaskFinished) {
            // Finish everything and evacuate
            finishAllTasks();
            finishNow();
        }
        return firstTaskFinished;
    }
}
