package org.murraybridgebunyips.bunyipslib.tasks.groups;

import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;

/**
 * A group of tasks.
 * Users must be careful to ensure they do not allocate tasks that use the same subsystems when
 * running in parallel (all task groups except SequentialTaskGroup), otherwise hardware will
 * try to take commands from multiple tasks at once. In WPILib, this is handled by mentioning
 * requirements, but the nature of the Task system doesn't demand the need to integrate this.
 *
 * @author Lucas Bubner, 2024
 */
public abstract class TaskGroup extends Task {
    protected final ArrayDeque<Task> tasks = new ArrayDeque<>();
    private final ArrayList<Task> attachedTasks = new ArrayList<>();
    private final ArrayList<BunyipsSubsystem> subsystems = new ArrayList<>();

    protected TaskGroup(Task... tasks) {
        super(0.0);
        this.tasks.addAll(Arrays.asList(tasks));
    }

    /**
     * Add subsystems to the task group to determine which subsystems each task should be attached.
     *
     * @param dependencies the subsystems to add, in order of task addition
     * @return the task group
     */
    public final TaskGroup withSubsystems(BunyipsSubsystem... dependencies) {
        subsystems.addAll(Arrays.asList(dependencies));
        return this;
    }

    protected final void executeTask(Task task) {
        // Do not manage a task if it is already attached to a subsystem being managed there
        if (attachedTasks.contains(task)) return;
        for (BunyipsSubsystem subsystem : subsystems) {
            if (subsystem.getTaskDependencies().contains(task.hashCode())) {
                subsystem.setCurrentTask(task);
                attachedTasks.add(task);
                return;
            }
        }
        // Otherwise we can just run the task outright
        task.run();
    }

    protected final void finishAllTasksExcluding(Task excluded) {
        for (Task task : tasks) {
            if (task != excluded) {
                task.finishNow();
            }
        }
    }

    @Override
    public final void init() {
        // no-op
    }

    @Override
    public final void onFinish() {
        // no-op
    }

    @Override
    protected void onReset() {
        for (Task task : tasks) {
            task.reset();
        }
    }
}
