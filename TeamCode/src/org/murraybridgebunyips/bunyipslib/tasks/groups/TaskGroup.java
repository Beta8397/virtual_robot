package org.murraybridgebunyips.bunyipslib.tasks.groups;

import static org.murraybridgebunyips.bunyipslib.Text.round;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.Seconds;

import org.murraybridgebunyips.bunyipslib.AutonomousBunyipsOpMode;
import org.murraybridgebunyips.bunyipslib.EmergencyStop;
import org.murraybridgebunyips.bunyipslib.Scheduler;
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
public abstract class TaskGroup extends Task {
    protected final ArrayList<Task> tasks = new ArrayList<>();
    private final HashSet<Task> attachedTasks = new HashSet<>();

    protected TaskGroup(Task... tasks) {
        // Try to extract the highest timeout to be the timeout of this task group, however if one is infinite
        // then the group is infinite
        super(Arrays.stream(tasks).anyMatch(t -> t.getTimeout().magnitude() == 0.0) ? INFINITE_TIMEOUT :
                Seconds.of(Arrays.stream(tasks).mapToDouble(t -> t.getTimeout().in(Seconds)).max().orElse(0.0)));
        this.tasks.addAll(Arrays.asList(tasks));
        if (tasks.length == 0) {
            throw new EmergencyStop(getClass().getSimpleName() + " created with no tasks.");
        }
        StringBuilder taskNames = new StringBuilder();
        taskNames.append(getClass().getSimpleName().replace("TaskGroup", ""));
        taskNames.append(": ");
        for (int i = 0; i < tasks.length - 1; i++) {
            taskNames.append(tasks[i]).append(", ");
        }
        taskNames.append(tasks[tasks.length - 1]);
        withName(taskNames.toString());
    }

    /**
     * Log the creation of this task group in the OpMode telemetry.
     * Called internally by {@link AutonomousBunyipsOpMode}.
     */
    public void logCreation() {
        String groupName = toString();
        String taskGroup = getClass().getSimpleName();
        // Avoid printing the group name if it is the same as the task group name
        if (!groupName.equals(taskGroup)) {
            opMode.telemetry.log("<font color='gray'>%:</font> % created with % tasks.", groupName, taskGroup, tasks.size());
        } else {
            opMode.telemetry.log("<font color='gray'>%:</font> Created with % tasks.", taskGroup, tasks.size());
        }
        // List subtasks
        for (Task task : tasks) {
            opMode.telemetry.log("&nbsp;&nbsp;-> <font color='gray'>%<i>(t=%)</i></font>", task.toString(), task.getTimeout().magnitude() != 0.0 ? round(task.getTimeout().in(Seconds), 1) + "s" : "∞");
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
            Scheduler.addTaskReport(toString(), false, task.toString(), round(task.getDeltaTime().in(Seconds), 1), task.getTimeout().in(Seconds));
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
