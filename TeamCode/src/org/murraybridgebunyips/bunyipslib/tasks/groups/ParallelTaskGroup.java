package org.murraybridgebunyips.bunyipslib.tasks.groups;

import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

/**
 * A group of tasks that runs all at once, until they are all finished.
 *
 * @author Lucas Bubner, 2024
 */
public class ParallelTaskGroup extends TaskGroup {
    @Override
    public final void periodic() {
        for (Task task : tasks) {
            task.run();
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
