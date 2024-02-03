package org.murraybridgebunyips.bunyipslib.tasks.groups;

import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

import java.util.ArrayDeque;
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

    protected TaskGroup(Task... tasks) {
        super(0.0);
        this.tasks.addAll(Arrays.asList(tasks));
    }

    @Override
    public final void init() {
        // no-op
    }

    @Override
    public final void onFinish() {
        // no-op
    }
}
