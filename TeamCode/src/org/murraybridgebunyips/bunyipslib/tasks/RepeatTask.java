package org.murraybridgebunyips.bunyipslib.tasks;

import org.murraybridgebunyips.bunyipslib.tasks.bases.ForeverTask;
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

/**
 * A task that will auto-reset and repeat itself after it is completed.
 *
 * @author Lucas Bubner, 2024
 */
public class RepeatTask extends ForeverTask {
    private final Task task;

    /**
     * Create a new RepeatTask with the given task.
     *
     * @param task The task to repeat after it is completed.
     */
    public RepeatTask(Task task) {
        this.task = task;
    }

    @Override
    protected void init() {
        task.run();
    }

    @Override
    protected void periodic() {
        if (task.pollFinished()) {
            task.reset();
        }
        task.run();
    }

    @Override
    protected void onFinish() {
        task.finishNow();
    }
}
