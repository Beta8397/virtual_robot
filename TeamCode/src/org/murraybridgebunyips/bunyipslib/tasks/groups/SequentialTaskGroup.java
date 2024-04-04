package org.murraybridgebunyips.bunyipslib.tasks.groups;

import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

/**
 * A group of tasks that runs one after the other, until they are all finished.
 *
 * @author Lucas Bubner, 2024
 */
public class SequentialTaskGroup extends TaskGroup {
    private int taskIndex;
    private Task currentTask;

    /**
     * Create a new SequentialTaskGroup with tasks.
     *
     * @param tasks The tasks to run in sequence
     */
    public SequentialTaskGroup(Task... tasks) {
        super(tasks);
        currentTask = this.tasks.get(0);
    }

    @Override
    public final void periodic() {
        if (currentTask.pollFinished()) {
            taskIndex++;
            if (taskIndex >= tasks.size()) {
                finish();
                return;
            }
            currentTask = tasks.get(taskIndex);
        } else {
            executeTask(currentTask);
        }
    }

    @Override
    protected final void onReset() {
        super.onReset();
        taskIndex = 0;
        currentTask = tasks.get(0);
    }

    @Override
    protected boolean isTaskFinished() {
        return taskIndex >= tasks.size() && currentTask.isFinished();
    }
}
