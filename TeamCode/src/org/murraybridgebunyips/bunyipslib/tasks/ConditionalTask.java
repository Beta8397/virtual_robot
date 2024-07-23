package org.murraybridgebunyips.bunyipslib.tasks;

import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

import java.util.function.BooleanSupplier;

/**
 * Two tasks that run based on a dynamically evaluated condition.
 *
 * @author Lucas Bubner, 2024
 */
public class ConditionalTask extends Task {
    private final Task trueTask;
    private final Task falseTask;
    private final BooleanSupplier condition;

    /**
     * Create a new conditional task with the given tasks and condition.
     *
     * @param trueTask  the task to run if the condition is true
     * @param falseTask the task to run if the condition is false
     * @param condition the condition to evaluate
     */
    public ConditionalTask(Task trueTask, Task falseTask, BooleanSupplier condition) {
        this.trueTask = trueTask;
        this.falseTask = falseTask;
        this.condition = condition;
        withName("Conditional " + trueTask + " / " + falseTask);
    }

    /**
     * Create a new conditional task with the given callbacks and condition.
     *
     * @param onTrue    the callback to run if the condition is true
     * @param onFalse   the callback to run if the condition is false
     * @param condition the condition to evaluate
     */
    public ConditionalTask(Runnable onTrue, Runnable onFalse, BooleanSupplier condition) {
        this(new RunTask(onTrue), new RunTask(onFalse), condition);
    }

    @Override
    protected void init() {
        if (condition.getAsBoolean()) {
            trueTask.run();
        } else {
            falseTask.run();
        }
    }

    @Override
    protected void periodic() {
        if (condition.getAsBoolean()) {
            trueTask.run();
        } else {
            falseTask.run();
        }
    }

    @Override
    protected boolean isTaskFinished() {
        return trueTask.pollFinished() || falseTask.pollFinished();
    }

    @Override
    protected void onReset() {
        trueTask.reset();
        falseTask.reset();
    }
}
