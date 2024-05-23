package org.murraybridgebunyips.bunyipslib.tasks.bases;

import static org.murraybridgebunyips.bunyipslib.external.units.Units.Milliseconds;

import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;

/**
 * A task that runs once and then immediately completes.
 */
public abstract class OnceTask extends Task {
    protected OnceTask() {
        // For OnceTasks, we can't have an infinite timeout but we can use a very very short one instead
        // This is so the schedulers do not mistake this task as for one that will end up running forever,
        // as all OnceTasks will run only once. This also helps telemetry decide how long a task will execute for.
        super(Milliseconds.of(1));
    }

    protected OnceTask(BunyipsSubsystem dependencySubsystem, boolean override) {
        super(Milliseconds.of(1), dependencySubsystem, override);
    }

    @Override
    protected final void init() {
        runOnce();
    }

    @Override
    protected final void periodic() {
    }

    @Override
    protected final boolean isTaskFinished() {
        // OnceTasks may sometimes have their timeouts adjusted at runtime
        return getTimeout().lte(Milliseconds.of(1));
    }

    @Override
    protected final void onFinish() {
    }

    /**
     * Code to run once.
     */
    protected abstract void runOnce();
}
