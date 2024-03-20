package org.murraybridgebunyips.bunyipslib.tasks;

import org.murraybridgebunyips.bunyipslib.tasks.bases.NoTimeoutTask;
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

import java.util.HashMap;
import java.util.function.Supplier;

/**
 * Select a task to run based on a hashmap of states and a supplier of states.
 */
public class SelectTask<T> extends NoTimeoutTask {
    private final Supplier<T> stateSupplier;
    private final HashMap<T, Task> tasks = new HashMap<>();

    /**
     * Create a new select task with the given state supplier.
     * @param stateSupplier the supplier of states to check in the when() method
     */
    public SelectTask(Supplier<T> stateSupplier) {
        this.stateSupplier = stateSupplier;
    }

    /**
     * Add a task to run when the state is equal to the given state.
     * @param state the state to run the task on
     * @param task the task to run
     * @return this task
     */
    public SelectTask<T> when(T state, Task task) {
        tasks.put(state, task);
        return this;
    }

    @Override
    protected void init() {
        Task task = tasks.get(stateSupplier.get());
        if (task != null) {
            task.run();
        }
    }

    @Override
    protected void periodic() {
        Task task = tasks.get(stateSupplier.get());
        if (task != null) {
            task.run();
        }
    }

    @Override
    protected void onFinish() {
        // no-op
    }

    @Override
    protected boolean isTaskFinished() {
        Task task = tasks.get(stateSupplier.get());
        return task != null && task.pollFinished();
    }

    @Override
    protected void onReset() {
        for (Task task : tasks.values()) {
            task.reset();
        }
    }
}
