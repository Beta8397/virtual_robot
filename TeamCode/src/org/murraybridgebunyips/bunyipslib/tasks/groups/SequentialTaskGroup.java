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
            currentTask.run();
        }
    }

    @Override
    public final boolean isTaskFinished() {
        for (Task task : tasks) {
            if (!task.isFinished()) return false;
        }
        return true;
    }
}
