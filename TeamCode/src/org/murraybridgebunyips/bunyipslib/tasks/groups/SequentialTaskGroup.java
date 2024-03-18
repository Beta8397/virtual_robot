package org.murraybridgebunyips.bunyipslib.tasks.groups;

import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

import java.util.Iterator;

/**
 * A group of tasks that runs one after the other, until they are all finished.
 *
 * @author Lucas Bubner, 2024
 */
public class SequentialTaskGroup extends TaskGroup {
    private final Iterator<Task> taskIterator;
    private Task currentTask;

    /**
     * Create a new SequentialTaskGroup with tasks.
     *
     * @param tasks The tasks to run in sequence
     */
    public SequentialTaskGroup(Task... tasks) {
        super(tasks);
        taskIterator = super.tasks.iterator();
        currentTask = taskIterator.next();
    }

    @Override
    public final void periodic() {
        if (currentTask.isFinished()) {
            if (!taskIterator.hasNext()) {
                finishNow();
                return;
            }
            currentTask = taskIterator.next();
        } else {
            executeTask(currentTask);
        }
    }

    @Override
    public final boolean isTaskFinished() {
        for (Task task : tasks) {
            if (!task.pollFinished()) return false;
        }
        return true;
    }
}
