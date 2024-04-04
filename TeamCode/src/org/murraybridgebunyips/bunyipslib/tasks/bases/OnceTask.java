package org.murraybridgebunyips.bunyipslib.tasks.bases;

import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;

/**
 * A task that runs once and then immediately completes.
 */
public abstract class OnceTask extends Task {
    protected OnceTask() {
        super(0.0);
    }

    protected OnceTask(BunyipsSubsystem dependencySubsystem, boolean override) {
        super(0.0, dependencySubsystem, override);
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
        return getTimeout() == 0.0;
    }

    @Override
    protected final void onFinish() {
    }

    /**
     * Code to run once.
     */
    protected abstract void runOnce();
}
