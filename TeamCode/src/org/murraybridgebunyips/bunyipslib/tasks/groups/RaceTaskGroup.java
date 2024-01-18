package org.murraybridgebunyips.bunyipslib.tasks.groups;

import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

/**
 * A group of tasks that runs all at once, until one of them is finished.
 * Once a single task is finished, all other tasks are cancelled.
 *
 * @author Lucas Bubner, 2024
 */
public class RaceTaskGroup extends TaskGroup {
    @Override
    public final void periodic() {
        for (Task task : tasks) {
            task.run();
        }
    }

    @Override
    public final boolean isTaskFinished() {
        for (Task task : tasks) {
            if (task.isFinished()) return true;
        }
        return false;
    }
}
