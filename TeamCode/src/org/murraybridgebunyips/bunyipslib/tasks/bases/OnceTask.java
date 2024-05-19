package org.murraybridgebunyips.bunyipslib.tasks.bases;

import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;

import static org.murraybridgebunyips.bunyipslib.external.units.Units.Milliseconds;

/**
 * A task that runs once and then immediately completes.
 */
public abstract class OnceTask extends Task {
    protected OnceTask() {
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
