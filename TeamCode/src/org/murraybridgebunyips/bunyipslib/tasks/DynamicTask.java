package org.murraybridgebunyips.bunyipslib.tasks;

import static org.murraybridgebunyips.bunyipslib.Text.getCallingUserCodeFunction;
import static org.murraybridgebunyips.bunyipslib.Text.html;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.Seconds;

import androidx.annotation.NonNull;

import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.Dbg;
import org.murraybridgebunyips.bunyipslib.external.units.Measure;
import org.murraybridgebunyips.bunyipslib.external.units.Time;
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
public class DynamicTask extends Task {
    private static final String UNCONSTRUCTED_NAME = "Pending construction (dyn)";
    private final Supplier<Task> lazyTask;
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
        super.withName(UNCONSTRUCTED_NAME);
    }

    @Override
    protected void init() {
        builtTask = lazyTask.get();
        String name = builtTask.toString();
        Measure<Time> timeout = builtTask.getTimeout();
        Dbg.logd(getClass(), "built -> % (t=%)", name, timeout.magnitude() <= 0 ? "inf" : timeout.in(Seconds) + "s");
        if (UNCONSTRUCTED_NAME.equals(toString()))
            super.withName(name);
        if (getTimeout().equals(INFINITE_TIMEOUT))
            setTimeout(timeout);
        builtTask.getDependency().ifPresent((dep) ->
                dep.setHighPriorityCurrentTask(builtTask));
    }

    @Override
    protected void periodic() {
        if (builtTask == null) return;
        if (!builtTask.hasDependency())
            builtTask.run();
    }

    @Override
    protected void onFinish() {
        if (builtTask == null) return;
        builtTask.finishNow();
        builtTask.getDependency().ifPresent(BunyipsSubsystem::cancelCurrentTask);
    }

    @Override
    protected void onReset() {
        if (builtTask == null) return;
        builtTask.reset();
        withName(UNCONSTRUCTED_NAME);
        withTimeout(INFINITE_TIMEOUT);
        builtTask = null;
    }

    @Override
    protected boolean isTaskFinished() {
        if (builtTask == null) return false;
        return builtTask.hasDependency() ? builtTask.isFinished() : builtTask.pollFinished();
    }

    /**
     * Set the name of this DynamicTask. Note that "(dyn)" will be appended to indicate this task is not constructed.
     * If the task is constructed, this method will no-op. Use the wrapped Task to set a name.
     */
    @NonNull
    @Override
    public final Task withName(String name) {
        if (builtTask != null)
            return this;
        super.withName(name + " (dyn)");
        return this;
    }

    /**
     * Subsystems cannot be attached to DynamicTasks. This method will no-op and log an error.
     */
    @NonNull
    @Override
    public final Task onSubsystem(@NonNull BunyipsSubsystem subsystem, boolean override) {
        Dbg.error(getCallingUserCodeFunction(), "Dynamic tasks are not designed to be attached to a subsystem, as the internal task will be scheduled to subsystems instead.");
        opMode.telemetry.log(getCallingUserCodeFunction(), html().color("red", "error: ").text("dynamic tasks cannot be attached to subsystems!"));
        return this;
    }
}
