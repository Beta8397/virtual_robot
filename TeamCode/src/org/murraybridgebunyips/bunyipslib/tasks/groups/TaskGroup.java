package org.murraybridgebunyips.bunyipslib.tasks.groups;

import static org.murraybridgebunyips.bunyipslib.Text.round;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.Seconds;

import org.murraybridgebunyips.bunyipslib.EmergencyStop;
import org.murraybridgebunyips.bunyipslib.Scheduler;
import org.murraybridgebunyips.bunyipslib.tasks.bases.NoTimeoutTask;
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;

/**
 * A group of tasks.
 * Users must be careful to ensure they do not allocate tasks that use the same subsystems when
 * running in parallel (all task groups except SequentialTaskGroup), otherwise hardware will
 * try to take commands from multiple tasks at once, overriding each other and causing unpredictable behaviour.
 *
 * @author Lucas Bubner, 2024
 */
public abstract class TaskGroup extends NoTimeoutTask {
    protected final ArrayList<Task> tasks = new ArrayList<>();
    private final HashSet<Task> attachedTasks = new HashSet<>();

    protected TaskGroup(Task... tasks) {
        this.tasks.addAll(Arrays.asList(tasks));
        if (tasks.length == 0) {
            throw new EmergencyStop("TaskGroup created with no tasks.");
        }
    }

    protected final void executeTask(Task task) {
        if (task.isFinished()) return;
        // Do not manage a task if it is already attached to a subsystem being managed there
        if (attachedTasks.contains(task)) return;
        task.getDependency().ifPresent(dependency -> {
            dependency.setCurrentTask(task);
            attachedTasks.add(task);
        });
        // Otherwise we can just run the task outright
        if (!task.hasDependency()) {
            Scheduler.addTaskReport(toString(), task.toString(), round(task.getDeltaTime().in(Seconds), 1), task.getTimeout().in(Seconds));
            task.run();
        }
    }

    protected final void finishAllTasks() {
        for (Task task : tasks) {
            task.finishNow();
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
        attachedTasks.clear();
    }
}
