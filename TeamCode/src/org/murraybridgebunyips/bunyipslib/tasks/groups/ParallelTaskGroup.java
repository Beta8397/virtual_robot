package org.murraybridgebunyips.bunyipslib.tasks.groups;

import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

/**
 * A group of tasks that runs all at once, until they are all finished.
 *
 * @author Lucas Bubner, 2024
 */
public class ParallelTaskGroup extends TaskGroup {
    /**
     * Create a new ParallelTaskGroup with tasks.
     *
     * @param tasks The tasks to run together
     */
    public ParallelTaskGroup(Task... tasks) {
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
            if (!task.pollFinished()) return false;
        }
        // No point in waiting for the next loop if we're all done
        finishNow();
        return true;
    }
}
