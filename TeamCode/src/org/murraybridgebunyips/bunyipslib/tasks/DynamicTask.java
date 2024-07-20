package org.murraybridgebunyips.bunyipslib.tasks;

import androidx.annotation.Nullable;

import org.murraybridgebunyips.bunyipslib.tasks.bases.NoTimeoutTask;
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

import java.util.function.Supplier;

/**
 * Represents a task that is constructed at runtime. This is useful for tasks that have runtime requirements
 * that cannot be determined when this task is made, such as RoadRunner tasks that run on a non-structural basis.
 * <p>
 * Note some caveats of using this task is that timeout and other information related to this task is unknown until
 * this task is run once, in which this data will be updated to match the task that was built. If this task
 * is to run on a subsystem, it <b>must be declared on the inner task</b>, as DynamicTask does not have enough information
 * to know where to run, since these allocations are done at construction.
 *
 * @author Lucas Bubner, 2024
 */
public class DynamicTask extends NoTimeoutTask {
    private final Supplier<Task> lazyTask;

    @Nullable
    private Task builtTask;

    /**
     * Construct a new DynamicTask to run.
     *
     * @param lazyTask the task to construct and run when the DynamicTask starts running.
     */
    public DynamicTask(Supplier<Task> lazyTask) {
        this.lazyTask = lazyTask;
        // We're not actually a task, so we'll let the inner task manage reports
        withMutedReports();
        withName("Dynamic (Pending construction)");
    }

    @Override
    protected void init() {
        builtTask = lazyTask.get();
        setTimeout(builtTask.getTimeout());
        withName(builtTask.toString());
    }

    @Override
    protected void periodic() {
        if (builtTask == null) return;
        builtTask.run();
    }

    @Override
    protected void onFinish() {
        // no-op
    }

    @Override
    protected boolean isTaskFinished() {
        if (builtTask == null) return false;
        return builtTask.pollFinished();
    }
}
